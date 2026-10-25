`timescale 1ns/1ps

// 单级 CIC 插值核心。
// 结构为：低速梳状差分级 -> 零插值 -> 高速积分级。
// 当前版本已按你的要求恢复为原版统一 ACC_W 内部位宽结构：
// 所有梳状级和积分级共用同一个 ACC_W。
// 在当前工程中，ACC_W 由 audio_interp_core 通过 mixed_wordlength_params.vh 传入，
// 因此联合字长搜索后可按级间样本宽度自动缩短，而不是固定使用最早的 64 bit。
module cic_interp_core #(
    parameter integer IN_W       = 24,
    parameter integer OUT_W      = 24,
    parameter integer ACC_W      = 64,
    parameter integer RATE       = 16,
    parameter integer STAGES     = 4,
    parameter integer GAIN_SHIFT = 12
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    clr,
    input  wire                    ce_out,
    input  wire signed [IN_W-1:0]  din,
    input  wire                    din_valid,
    output reg  signed [OUT_W-1:0] dout,
    output reg                     dout_valid
);

    integer i;

    // 梳状差分级延时寄存器和当前差分结果。
    reg signed [ACC_W-1:0] comb_delay [0:STAGES-1];
    reg signed [ACC_W-1:0] comb_work  [0:STAGES-1];

    // 积分级状态寄存器和当前积分结果。
    reg signed [ACC_W-1:0] integ_reg  [0:STAGES-1];
    reg signed [ACC_W-1:0] integ_work [0:STAGES-1];

    // 当前低速输入样本还需要继续产生的 128x 输出槽位数。
    reg [15:0] burst_remain;
    reg        burst_active;

    reg signed [ACC_W-1:0] din_ext;
    reg signed [ACC_W-1:0] cic_drive;
    reg signed [ACC_W-1:0] acc_rounded;
    reg signed [OUT_W-1:0] dout_next;

    localparam signed [ACC_W-1:0] ONE_ACC   = {{(ACC_W-1){1'b0}}, 1'b1};
    localparam integer DATA_DROP_SHIFT = (IN_W > OUT_W) ? (IN_W - OUT_W) : 0;
    localparam integer TOTAL_SHIFT     = GAIN_SHIFT + DATA_DROP_SHIFT;
    localparam signed [ACC_W-1:0] OUT_MAX   = (ONE_ACC <<< (OUT_W-1)) - 1'b1;
    localparam signed [ACC_W-1:0] OUT_MIN   = -(ONE_ACC <<< (OUT_W-1));
    localparam signed [ACC_W-1:0] ROUND_CST =
        (TOTAL_SHIFT > 0) ? (ONE_ACC <<< (TOTAL_SHIFT-1)) : {ACC_W{1'b0}};

    // 低速梳状差分级。
    always @(*) begin
        din_ext = {{(ACC_W-IN_W){din[IN_W-1]}}, din};

        comb_work[0] = din_ext - comb_delay[0];
        for (i = 1; i < STAGES; i = i + 1) begin
            comb_work[i] = comb_work[i-1] - comb_delay[i];
        end
    end

    // 梳状级和积分级之间的零插值。
    always @(*) begin
        if (din_valid) begin
            cic_drive = comb_work[STAGES-1];
        end else begin
            cic_drive = {ACC_W{1'b0}};
        end
    end

    // 高速积分级，以及最后的缩放和饱和输出。
    always @(*) begin
        integ_work[0] = integ_reg[0] + cic_drive;
        for (i = 1; i < STAGES; i = i + 1) begin
            integ_work[i] = integ_reg[i] + integ_work[i-1];
        end

        if (TOTAL_SHIFT == 0) begin
            acc_rounded = integ_work[STAGES-1];
        end else if (integ_work[STAGES-1] >= 0) begin
            acc_rounded = (integ_work[STAGES-1] + ROUND_CST) >>> TOTAL_SHIFT;
        end else begin
            acc_rounded = -(((-integ_work[STAGES-1]) + ROUND_CST) >>> TOTAL_SHIFT);
        end

        if (acc_rounded > OUT_MAX) begin
            dout_next = OUT_MAX[OUT_W-1:0];
        end else if (acc_rounded < OUT_MIN) begin
            dout_next = OUT_MIN[OUT_W-1:0];
        end else begin
            dout_next = acc_rounded[OUT_W-1:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout         <= {OUT_W{1'b0}};
            dout_valid   <= 1'b0;
            burst_remain <= 16'd0;
            burst_active <= 1'b0;

            for (i = 0; i < STAGES; i = i + 1) begin
                comb_delay[i] <= {ACC_W{1'b0}};
                integ_reg[i]  <= {ACC_W{1'b0}};
            end
        end else if (clr) begin
            dout         <= {OUT_W{1'b0}};
            dout_valid   <= 1'b0;
            burst_remain <= 16'd0;
            burst_active <= 1'b0;

            for (i = 0; i < STAGES; i = i + 1) begin
                comb_delay[i] <= {ACC_W{1'b0}};
                integ_reg[i]  <= {ACC_W{1'b0}};
            end
        end else begin
            dout_valid <= 1'b0;

            if (ce_out) begin
                if (din_valid) begin
                    comb_delay[0] <= din_ext;
                    for (i = 1; i < STAGES; i = i + 1) begin
                        comb_delay[i] <= comb_work[i-1];
                    end

                    integ_reg[0] <= integ_work[0];
                    for (i = 1; i < STAGES; i = i + 1) begin
                        integ_reg[i] <= integ_work[i];
                    end

                    dout       <= dout_next;
                    dout_valid <= 1'b1;

                    if (RATE > 1) begin
                        burst_remain <= RATE - 1;
                        burst_active <= 1'b1;
                    end else begin
                        burst_remain <= 16'd0;
                        burst_active <= 1'b0;
                    end
                end else if (burst_active) begin
                    integ_reg[0] <= integ_work[0];
                    for (i = 1; i < STAGES; i = i + 1) begin
                        integ_reg[i] <= integ_work[i];
                    end

                    dout       <= dout_next;
                    dout_valid <= 1'b1;

                    if (burst_remain == 16'd1) begin
                        burst_remain <= 16'd0;
                        burst_active <= 1'b0;
                    end else begin
                        burst_remain <= burst_remain - 16'd1;
                        burst_active <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
