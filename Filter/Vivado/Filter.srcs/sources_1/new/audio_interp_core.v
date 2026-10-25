`timescale 1ns/1ps
`include "mixed_wordlength_params.vh"
//  clk_audio_512fs = 512 * 44.1 kHz = 22.5792 MHz
//  输出采样率： 128 * 44.1 kHz = 5.6448 MHz

module audio_interp_core #(
    parameter integer IN_W               = 24,
    parameter integer OUT_W              = 24,
    parameter integer STAGE0_DATA_W      = `WL_STAGE0_DATA_W,
    parameter integer HALFBAND1_DATA_W   = `WL_HALFBAND1_DATA_W,
    parameter integer HALFBAND2_DATA_W   = `WL_HALFBAND2_DATA_W,
    parameter integer CIC_DATA_W         = `WL_CIC_DATA_W,
    parameter integer CIC_COMP_DATA_W    = `WL_CIC_COMP_DATA_W,
    parameter integer STAGE0_COEF_W      = `WL_STAGE0_COEF_W,
    parameter integer STAGE0_COEF_FRAC   = `WL_STAGE0_COEF_FRAC,
    parameter integer STAGE0_ACC_W       = `WL_STAGE0_ACC_W,
    parameter integer HALFBAND_COEF_W    = `WL_HALFBAND_COEF_W,
    parameter integer HALFBAND_COEF_FRAC = `WL_HALFBAND_COEF_FRAC,
    parameter integer HALFBAND_ACC_W     = `WL_HALFBAND_ACC_W,
    parameter integer CIC_ACC_W          = `WL_CIC_ACC_W,
    parameter integer CIC_COMP_COEF_W    = `WL_CIC_COMP_COEF_W,
    parameter integer CIC_COMP_COEF_FRAC = `WL_CIC_COMP_COEF_FRAC,
    parameter integer CIC_COMP_ACC_W     = `WL_CIC_COMP_ACC_W,
    parameter integer FIFO_DEPTH         = 32,
    parameter integer OUTPUT_ALIGN_DELAY = 192
)(
    input  wire                    clk_audio_512fs,
    input  wire                    rst_n,

    input  wire signed [IN_W-1:0]  pcm_in,
    input  wire                    pcm_in_valid,
    output wire                    pcm_in_ready,

    output reg  signed [OUT_W-1:0] pcm_out,
    output reg                     pcm_out_valid,

    // 粘滞异常状态，供顶层 ILA 和板上调试使用。
    // [3] 三级共享 MAC 调度溢出
    // [2] 低速补偿 FIR 输入溢出
    // [1] 补偿速率 FIFO 下溢
    // [0] 补偿速率 FIFO 上溢
    output wire [3:0]              core_fault_flags
);

    // 固定滤波器参数
    localparam integer STAGE0_PH0_TAPS = 1024;
    localparam integer STAGE0_PH1_TAPS = 1023;
    localparam integer HB_PH0_TAPS     = 48;
    localparam integer HB_PH1_TAPS     = 47;
    localparam integer CIC_COMP_TAPS   = 31;

    localparam integer CIC_RATE       = 16;
    localparam integer CIC_STAGES     = 4;
    localparam integer CIC_GAIN_SHIFT = 12; // 16^(4-1) = 4096 = 2^12

    // 样本域输出对齐延时的安全上限
    localparam integer MAX_OUTPUT_ALIGN_DELAY  = 320;
    localparam integer OUTPUT_ALIGN_DELAY_SAFE =
        (OUTPUT_ALIGN_DELAY < 0) ? 0 :
        ((OUTPUT_ALIGN_DELAY > MAX_OUTPUT_ALIGN_DELAY) ? MAX_OUTPUT_ALIGN_DELAY : OUTPUT_ALIGN_DELAY);
    localparam integer OUT_DELAY_DEPTH = (MAX_OUTPUT_ALIGN_DELAY < 1) ? 1 : MAX_OUTPUT_ALIGN_DELAY;

    // 512Fs 到 128Fs 的输出槽位产生器
    // 每 4 个 512Fs 时钟周期产生 1 个最终输出样本槽位
    reg [1:0] slot_cnt;
    reg       slot_ce;

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            slot_cnt <= 2'd0;
            slot_ce  <= 1'b0;
        end else begin
            if (slot_cnt == 2'd0) begin
                slot_cnt <= 2'd3;
                slot_ce  <= 1'b1;
            end else begin
                slot_cnt <= slot_cnt - 2'd1;
                slot_ce  <= 1'b0;
            end
        end
    end

    // rate_cnt 当前位于一个 44.1 kHz 输入样本周期内的第几个 128x 输出槽位
    reg [6:0] rate_cnt;

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            rate_cnt <= 7'd0;
        end else if (slot_ce) begin
            if (rate_cnt == 7'd127)
                rate_cnt <= 7'd0;
            else
                rate_cnt <= rate_cnt + 7'd1;
        end
    end

    //   ce0：2Fs 输出槽位
    //   ce1：4Fs 输出槽位
    //   ce2：8Fs 输出槽位
    wire ce0 = slot_ce && (rate_cnt[5:0] == 6'b000000);
    wire ce1 = slot_ce && (rate_cnt[4:0] == 5'b00000);
    wire ce2 = slot_ce && (rate_cnt[3:0] == 4'b0000);

    // 多相 FIR 的相位跟踪寄存器
    // 偶相从前一级速率域消耗一个新输入样本；奇相产生插入样本
    reg [2:0] phase_ref;

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            phase_ref <= 3'b000;
        end else begin
            if (ce0) phase_ref[0] <= ~phase_ref[0];
            if (ce1) phase_ref[1] <= ~phase_ref[1];
            if (ce2) phase_ref[2] <= ~phase_ref[2];
        end
    end

    wire stage0_in_slot = ce0 && (phase_ref[0] == 1'b0);
    wire stage1_in_slot = ce1 && (phase_ref[1] == 1'b0);
    wire stage2_in_slot = ce2 && (phase_ref[2] == 1'b0);

    // 外部输入握手
    assign pcm_in_ready = stage0_in_slot;

    // 各级数据连线
    wire signed [STAGE0_DATA_W-1:0]   stage0_dout;
    wire signed [HALFBAND1_DATA_W-1:0] stage1_dout;
    wire signed [HALFBAND2_DATA_W-1:0] stage2_dout;
    wire signed [CIC_COMP_DATA_W-1:0]  comp_dout;
    wire signed [CIC_DATA_W-1:0]       cic_dout;

    wire stage0_dout_valid;
    wire stage1_dout_valid;
    wire stage2_dout_valid;
    wire comp_dout_valid;
    wire cic_dout_valid;

    wire signed [STAGE0_DATA_W-1:0]    fifo01_dout;
    wire signed [HALFBAND1_DATA_W-1:0] fifo12_dout;
    wire fifo01_empty;
    wire fifo01_full;
    wire fifo12_empty;
    wire fifo12_full;

    // Stage0 使用独立四路 DSP48 对称 MAC，在 128 个 512Fs 时钟内完成
    // 一个 1024/1023-tap 多相分支。后续两个 Halfband 继续共享原单 MAC。
    wire stage0_schedule_overflow;
    wire shared_schedule_overflow;
    wire signed [STAGE0_DATA_W-1:0] shared_stage0_unused;
    wire shared_stage0_unused_valid;

    fir_stage0_parallel #(
        .IN_W          (IN_W),
        .OUT_W         (STAGE0_DATA_W),
        .COEF_W        (STAGE0_COEF_W),
        .COEF_FRAC     (STAGE0_COEF_FRAC),
        .ACC_W         (STAGE0_ACC_W),
        .PH0_TAPS      (STAGE0_PH0_TAPS),
        .PH1_TAPS      (STAGE0_PH1_TAPS),
        .DEPTH         (STAGE0_PH0_TAPS),
        .ADDR_W        (10),
        .COEF_FILE_EVEN("coef_stage0_even.mem"),
        .COEF_FILE_ODD ("coef_stage0_odd.mem")
    ) u_stage0_parallel (
        .clk              (clk_audio_512fs),
        .rst_n            (rst_n),
        .clr              (1'b0),
        .ce_out           (ce0),
        .din              (pcm_in),
        .din_valid        (pcm_in_valid && stage0_in_slot),
        .dout             (stage0_dout),
        .dout_valid       (stage0_dout_valid),
        .schedule_overflow(stage0_schedule_overflow)
    );

    fir_shared_mac #(
        .STAGE0_IN_W             (IN_W),
        .STAGE0_OUT_W            (STAGE0_DATA_W),
        .HB1_IN_W                (STAGE0_DATA_W),
        .HB1_OUT_W               (HALFBAND1_DATA_W),
        .HB2_IN_W                (HALFBAND1_DATA_W),
        .HB2_OUT_W               (HALFBAND2_DATA_W),
        .STAGE0_COEF_W           (STAGE0_COEF_W),
        .STAGE0_COEF_FRAC        (STAGE0_COEF_FRAC),
        .HB_COEF_W               (HALFBAND_COEF_W),
        .HB_COEF_FRAC            (HALFBAND_COEF_FRAC),
        .STAGE0_ACC_W            (STAGE0_ACC_W),
        .HB_ACC_W                (HALFBAND_ACC_W),
        .STAGE0_PH0_TAPS         (1),
        .STAGE0_PH1_TAPS         (1),
        .HB_PH0_TAPS             (HB_PH0_TAPS),
        .HB_PH1_TAPS             (HB_PH1_TAPS),
        .STAGE0_COEF_FILE_EVEN   ("coef_stage0_even.mem"),
        .STAGE0_COEF_FILE_ODD    ("coef_stage0_odd.mem"),
        .HB_COEF_FILE_EVEN       ("coef_halfband_even.mem"),
        .HB_COEF_FILE_ODD        ("coef_halfband_odd.mem")
    ) u_shared_fir (
        .clk                    (clk_audio_512fs),
        .rst_n                  (rst_n),
        .clr                    (1'b0),
        .ce_stage0              (1'b0),
        .stage0_din             ({IN_W{1'b0}}),
        .stage0_din_valid       (1'b0),
        .stage0_dout            (shared_stage0_unused),
        .stage0_dout_valid      (shared_stage0_unused_valid),
        .ce_hb1                 (ce1),
        .hb1_din                (fifo01_dout),
        .hb1_din_valid          (stage1_in_slot && !fifo01_empty),
        .hb1_dout               (stage1_dout),
        .hb1_dout_valid         (stage1_dout_valid),
        .ce_hb2                 (ce2),
        .hb2_din                (fifo12_dout),
        .hb2_din_valid          (stage2_in_slot && !fifo12_empty),
        .hb2_dout               (stage2_dout),
        .hb2_dout_valid         (stage2_dout_valid),
        .schedule_overflow      (shared_schedule_overflow)
    );

    sample_fifo #(
        .WIDTH(STAGE0_DATA_W),
        .DEPTH(FIFO_DEPTH)
    ) u_fifo01 (
        .clk   (clk_audio_512fs),
        .rst_n (rst_n),
        .clr   (1'b0),
        .din   (stage0_dout),
        .wr_en (stage0_dout_valid),
        .rd_en (stage1_in_slot && !fifo01_empty),
        .dout  (fifo01_dout),
        .full  (fifo01_full),
        .empty (fifo01_empty)
    );

    sample_fifo #(
        .WIDTH(HALFBAND1_DATA_W),
        .DEPTH(FIFO_DEPTH)
    ) u_fifo12 (
        .clk   (clk_audio_512fs),
        .rst_n (rst_n),
        .clr   (1'b0),
        .din   (stage1_dout),
        .wr_en (stage1_dout_valid),
        .rd_en (stage2_in_slot && !fifo12_empty),
        .dout  (fifo12_dout),
        .full  (fifo12_full),
        .empty (fifo12_empty)
    );

    // 低速前置 CIC 补偿 FIR：工作在 352.8 kHz（8Fs），
    // 31 tap 对称后由一个 MAC 在相邻输入之间完成 16 次乘加。
    wire comp_input_overflow;

    cic_shared_mac #(
        .IN_W       (HALFBAND2_DATA_W),
        .COEF_W     (CIC_COMP_COEF_W),
        .COEF_FRAC  (CIC_COMP_COEF_FRAC),
        .OUT_W      (CIC_COMP_DATA_W),
        .ACC_W      (CIC_COMP_ACC_W),
        .TAPS       (CIC_COMP_TAPS),
        .COEF_FILE  ("coef_cic_comp.mem")
    ) u_comp_lowrate (
        .clk            (clk_audio_512fs),
        .rst_n          (rst_n),
        .clr            (1'b0),
        .din            (stage2_dout),
        .din_valid      (stage2_dout_valid),
        .dout           (comp_dout),
        .dout_valid     (comp_dout_valid),
        .input_overflow (comp_input_overflow)
    );

    // 共享 MAC 会使 Stage2/补偿器的“完成时刻”在平均 8Fs 周期附近轻微摆动。
    // CIC 必须严格每 16 个 128Fs 槽位接收一个新样本，不能直接跟随完成脉冲。
    // 因此加入小型弹性 FIFO：先预存两个样本，再由固定 ce2 节拍匀速读出。
    wire signed [CIC_COMP_DATA_W-1:0] comp_fifo_dout;
    wire comp_fifo_full;
    wire comp_fifo_empty;
    reg  [1:0] comp_prefill_count;
    reg        comp_rate_started;
    reg        comp_rate_underflow;
    reg        comp_rate_overflow;

    assign core_fault_flags = {
        (stage0_schedule_overflow | shared_schedule_overflow),
        comp_input_overflow,
        comp_rate_underflow,
        comp_rate_overflow
    };

    wire comp_fifo_read = ce2 && comp_rate_started && !comp_fifo_empty;
    wire signed [CIC_COMP_DATA_W-1:0] cic_input_data = comp_fifo_dout;
    wire cic_input_valid = comp_fifo_read;

    sample_fifo #(
        .WIDTH(CIC_COMP_DATA_W),
        .DEPTH(8)
    ) u_comp_rate_fifo (
        .clk   (clk_audio_512fs),
        .rst_n (rst_n),
        .clr   (1'b0),
        .din   (comp_dout),
        .wr_en (comp_dout_valid),
        .rd_en (comp_fifo_read),
        .dout  (comp_fifo_dout),
        .full  (comp_fifo_full),
        .empty (comp_fifo_empty)
    );

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            comp_prefill_count  <= 2'd0;
            comp_rate_started   <= 1'b0;
            comp_rate_underflow <= 1'b0;
            comp_rate_overflow  <= 1'b0;
        end else begin
            if (!comp_rate_started && comp_dout_valid && !comp_fifo_full) begin
                if (comp_prefill_count == 2'd1) begin
                    comp_prefill_count <= 2'd2;
                    comp_rate_started  <= 1'b1;
                end else begin
                    comp_prefill_count <= comp_prefill_count + 1'b1;
                end
            end

            if (comp_dout_valid && comp_fifo_full)
                comp_rate_overflow <= 1'b1;
            if (ce2 && comp_rate_started && comp_fifo_empty)
                comp_rate_underflow <= 1'b1;
        end
    end

    // 352.8 kHz -> 5.6448 MHz，16 倍插值 CIC。
    cic_interp_core #(
        .IN_W      (CIC_COMP_DATA_W),
        .OUT_W     (CIC_DATA_W),
        .ACC_W     (CIC_ACC_W),
        .RATE      (CIC_RATE),
        .STAGES    (CIC_STAGES),
        .GAIN_SHIFT(CIC_GAIN_SHIFT)
    ) u_cic (
        .clk       (clk_audio_512fs),
        .rst_n     (rst_n),
        .clr       (1'b0),
        .ce_out    (slot_ce),
        .din       (cic_input_data),
        .din_valid (cic_input_valid),
        .dout      (cic_dout),
        .dout_valid(cic_dout_valid)
    );

    // 可选样本域输出对齐延时
    reg signed [CIC_DATA_W-1:0] out_delay_line [0:OUT_DELAY_DEPTH-1];
    integer dly_i;

    localparam integer OUT_DELAY_INDEX = (OUTPUT_ALIGN_DELAY_SAFE == 0) ? 0 : (OUTPUT_ALIGN_DELAY_SAFE - 1);
    localparam integer OUTPUT_RESTORE_SHIFT =
        (OUT_W > CIC_DATA_W) ? (OUT_W - CIC_DATA_W) : 0;

    wire signed [CIC_DATA_W-1:0] aligned_cic_sample =
        (OUTPUT_ALIGN_DELAY_SAFE == 0) ? cic_dout : out_delay_line[OUT_DELAY_INDEX];
    wire signed [OUT_W-1:0] restored_comp_sample;

    generate
        if (OUT_W >= CIC_DATA_W) begin : g_restore_output_scale
            wire signed [OUT_W-1:0] comp_extended;
            assign comp_extended =
                {{(OUT_W-CIC_DATA_W){aligned_cic_sample[CIC_DATA_W-1]}},
                  aligned_cic_sample};
            assign restored_comp_sample = comp_extended <<< OUTPUT_RESTORE_SHIFT;
        end else begin : g_truncate_output_scale
            assign restored_comp_sample = aligned_cic_sample[OUT_W-1:0];
        end
    endgenerate

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            pcm_out       <= {OUT_W{1'b0}};
            pcm_out_valid <= 1'b0;
            for (dly_i = 0; dly_i < OUT_DELAY_DEPTH; dly_i = dly_i + 1)
                out_delay_line[dly_i] <= {CIC_DATA_W{1'b0}};
        end else begin
            pcm_out_valid <= cic_dout_valid;

            if (cic_dout_valid) begin
                pcm_out <= restored_comp_sample;

                out_delay_line[0] <= cic_dout;
                for (dly_i = 1; dly_i < OUT_DELAY_DEPTH; dly_i = dly_i + 1)
                    out_delay_line[dly_i] <= out_delay_line[dly_i-1];
            end
        end
    end

endmodule
