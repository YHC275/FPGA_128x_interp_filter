`timescale 1ns/1ps

// 三级 2x 插值 FIR 的统一共享 MAC 引擎。
//
// 设计目标：
// 1. Stage0、Halfband1、Halfband2 保留各自的相位、系数和历史样本；
// 2. 三个滤波级只共享一条“对称预加 + 乘法 + 累加”数据通路；
// 3. 使用静态优先级调度（HB2 > HB1 > Stage0）满足多速率截止期；
// 4. 历史样本以循环地址保存，不再每次整体移动延时数组；
// 5. 两份内容相同的分布式 RAM 提供对称抽头的双异步读端口。
//
// 当前工程中的平均 MAC 占用率约为：
//   Stage0 12.7% + HB1 18.75% + HB2 37.5% = 68.95%，
// 因此一个 MAC 可以在 512Fs 时钟域内完成三级运算。
module fir_shared_mac #(
    parameter integer STAGE0_IN_W       = 24,
    parameter integer STAGE0_OUT_W      = 19,
    parameter integer HB1_IN_W          = 19,
    parameter integer HB1_OUT_W         = 19,
    parameter integer HB2_IN_W          = 19,
    parameter integer HB2_OUT_W         = 19,
    parameter integer STAGE0_COEF_W     = 16,
    parameter integer STAGE0_COEF_FRAC  = 15,
    parameter integer HB_COEF_W         = 16,
    parameter integer HB_COEF_FRAC      = 15,
    parameter integer STAGE0_ACC_W      = 41,
    parameter integer HB_ACC_W          = 36,
    parameter integer STAGE0_PH0_TAPS   = 65,
    parameter integer STAGE0_PH1_TAPS   = 64,
    parameter integer HB_PH0_TAPS       = 48,
    parameter integer HB_PH1_TAPS       = 47,
    parameter STAGE0_COEF_FILE_EVEN     = "coef_stage0_even.mem",
    parameter STAGE0_COEF_FILE_ODD      = "coef_stage0_odd.mem",
    parameter HB_COEF_FILE_EVEN         = "coef_halfband_even.mem",
    parameter HB_COEF_FILE_ODD          = "coef_halfband_odd.mem"
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         clr,

    input  wire                         ce_stage0,
    input  wire signed [STAGE0_IN_W-1:0] stage0_din,
    input  wire                         stage0_din_valid,
    output reg  signed [STAGE0_OUT_W-1:0] stage0_dout,
    output reg                          stage0_dout_valid,

    input  wire                         ce_hb1,
    input  wire signed [HB1_IN_W-1:0]  hb1_din,
    input  wire                         hb1_din_valid,
    output reg  signed [HB1_OUT_W-1:0] hb1_dout,
    output reg                          hb1_dout_valid,

    input  wire                         ce_hb2,
    input  wire signed [HB2_IN_W-1:0]  hb2_din,
    input  wire                         hb2_din_valid,
    output reg  signed [HB2_OUT_W-1:0] hb2_dout,
    output reg                          hb2_dout_valid,

    // 调度溢出为粘滞标志。正常设计中应始终为 0，可供 ILA/仿真检查。
    output reg                          schedule_overflow
);

    localparam integer MAX_SAMPLE_W =
        (STAGE0_IN_W >= HB1_IN_W && STAGE0_IN_W >= HB2_IN_W) ? (STAGE0_IN_W + 1) :
        ((HB1_IN_W >= HB2_IN_W) ? (HB1_IN_W + 1) : (HB2_IN_W + 1));
    localparam integer MAX_COEF_W =
        (STAGE0_COEF_W >= HB_COEF_W) ? STAGE0_COEF_W : HB_COEF_W;
    localparam integer MAX_ACC_W =
        (STAGE0_ACC_W >= HB_ACC_W) ? STAGE0_ACC_W : HB_ACC_W;
    localparam integer PRODUCT_W = MAX_SAMPLE_W + MAX_COEF_W;

    localparam [1:0] JOB_STAGE0 = 2'd0;
    localparam [1:0] JOB_HB1    = 2'd1;
    localparam [1:0] JOB_HB2    = 2'd2;

    localparam integer STAGE0_SHIFT = STAGE0_COEF_FRAC +
        ((STAGE0_IN_W > STAGE0_OUT_W) ? (STAGE0_IN_W - STAGE0_OUT_W) : 0);
    localparam integer HB1_SHIFT = HB_COEF_FRAC +
        ((HB1_IN_W > HB1_OUT_W) ? (HB1_IN_W - HB1_OUT_W) : 0);
    localparam integer HB2_SHIFT = HB_COEF_FRAC +
        ((HB2_IN_W > HB2_OUT_W) ? (HB2_IN_W - HB2_OUT_W) : 0);

    localparam signed [MAX_ACC_W-1:0] ONE_ACC =
        {{(MAX_ACC_W-1){1'b0}}, 1'b1};
    localparam signed [MAX_ACC_W-1:0] STAGE0_OUT_MAX =
        (ONE_ACC <<< (STAGE0_OUT_W-1)) - 1'b1;
    localparam signed [MAX_ACC_W-1:0] STAGE0_OUT_MIN =
        -(ONE_ACC <<< (STAGE0_OUT_W-1));
    localparam signed [MAX_ACC_W-1:0] HB1_OUT_MAX =
        (ONE_ACC <<< (HB1_OUT_W-1)) - 1'b1;
    localparam signed [MAX_ACC_W-1:0] HB1_OUT_MIN =
        -(ONE_ACC <<< (HB1_OUT_W-1));
    localparam signed [MAX_ACC_W-1:0] HB2_OUT_MAX =
        (ONE_ACC <<< (HB2_OUT_W-1)) - 1'b1;
    localparam signed [MAX_ACC_W-1:0] HB2_OUT_MIN =
        -(ONE_ACC <<< (HB2_OUT_W-1));

    // 每个相位的完整系数仍按原 .mem 文件加载；对称乘加只访问前半部分。
    reg signed [STAGE0_COEF_W-1:0] stage0_coef_even [0:STAGE0_PH0_TAPS-1];
    reg signed [STAGE0_COEF_W-1:0] stage0_coef_odd  [0:STAGE0_PH1_TAPS-1];
    reg signed [HB_COEF_W-1:0]     hb_coef_even     [0:HB_PH0_TAPS-1];
    reg signed [HB_COEF_W-1:0]     hb_coef_odd      [0:HB_PH1_TAPS-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < STAGE0_PH0_TAPS; init_i = init_i + 1) begin
            stage0_coef_even[init_i] = {STAGE0_COEF_W{1'b0}};
        end
        for (init_i = 0; init_i < STAGE0_PH1_TAPS; init_i = init_i + 1)
            stage0_coef_odd[init_i] = {STAGE0_COEF_W{1'b0}};
        for (init_i = 0; init_i < HB_PH0_TAPS; init_i = init_i + 1) begin
            hb_coef_even[init_i] = {HB_COEF_W{1'b0}};
        end
        for (init_i = 0; init_i < HB_PH1_TAPS; init_i = init_i + 1)
            hb_coef_odd[init_i] = {HB_COEF_W{1'b0}};

        // Pro 版本把超长 Stage0 移到独立四路 MAC。实例化本模块时会把
        // Stage0 tap 参数置 1 并关闭 ce，因而无需重复加载 2047-tap ROM。
        if (STAGE0_PH0_TAPS > 1)
            $readmemh(STAGE0_COEF_FILE_EVEN, stage0_coef_even);
        if (STAGE0_PH1_TAPS > 1)
            $readmemh(STAGE0_COEF_FILE_ODD,  stage0_coef_odd);
        $readmemh(HB_COEF_FILE_EVEN,     hb_coef_even);
        $readmemh(HB_COEF_FILE_ODD,      hb_coef_odd);
    end

    // 当前循环缓冲区头指针和有效历史样本数。
    reg [6:0] stage0_head;
    reg [6:0] stage0_fill;
    reg [5:0] hb1_head, hb2_head;
    reg [5:0] hb1_fill, hb2_fill;

    // 每一级的多相相位。
    reg stage0_phase, hb1_phase, hb2_phase;

    // 三组历史样本均采用标准分布式 RAM 推断模板。
    wire stage0_mem_we = rst_n && !clr && ce_stage0 && !stage0_phase && stage0_din_valid;
    wire [6:0] stage0_mem_waddr = (stage0_head == 0) ?
        STAGE0_PH0_TAPS - 1 : stage0_head - 1'b1;
    reg [6:0] stage0_mem_raddr_a, stage0_mem_raddr_b;
    wire signed [STAGE0_IN_W-1:0] stage0_mem_rdata_a, stage0_mem_rdata_b;

    wire hb1_mem_we = rst_n && !clr && ce_hb1 && !hb1_phase && hb1_din_valid;
    wire [5:0] hb1_mem_waddr = (hb1_head == 0) ?
        HB_PH0_TAPS - 1 : hb1_head - 1'b1;
    reg [5:0] hb1_mem_raddr_a, hb1_mem_raddr_b;
    wire signed [HB1_IN_W-1:0] hb1_mem_rdata_a, hb1_mem_rdata_b;

    wire hb2_mem_we = rst_n && !clr && ce_hb2 && !hb2_phase && hb2_din_valid;
    wire [5:0] hb2_mem_waddr = (hb2_head == 0) ?
        HB_PH0_TAPS - 1 : hb2_head - 1'b1;
    reg [5:0] hb2_mem_raddr_a, hb2_mem_raddr_b;
    wire signed [HB2_IN_W-1:0] hb2_mem_rdata_a, hb2_mem_rdata_b;

    cir_read_ram #(
        .DATA_W(STAGE0_IN_W), .DEPTH(STAGE0_PH0_TAPS), .ADDR_W(7)
    ) u_stage0_delay_ram (
        .clk(clk), .we(stage0_mem_we), .waddr(stage0_mem_waddr), .din(stage0_din),
        .raddr_a(stage0_mem_raddr_a), .raddr_b(stage0_mem_raddr_b),
        .dout_a(stage0_mem_rdata_a), .dout_b(stage0_mem_rdata_b)
    );

    cir_read_ram #(
        .DATA_W(HB1_IN_W), .DEPTH(HB_PH0_TAPS), .ADDR_W(6)
    ) u_hb1_delay_ram (
        .clk(clk), .we(hb1_mem_we), .waddr(hb1_mem_waddr), .din(hb1_din),
        .raddr_a(hb1_mem_raddr_a), .raddr_b(hb1_mem_raddr_b),
        .dout_a(hb1_mem_rdata_a), .dout_b(hb1_mem_rdata_b)
    );

    cir_read_ram #(
        .DATA_W(HB2_IN_W), .DEPTH(HB_PH0_TAPS), .ADDR_W(6)
    ) u_hb2_delay_ram (
        .clk(clk), .we(hb2_mem_we), .waddr(hb2_mem_waddr), .din(hb2_din),
        .raddr_a(hb2_mem_raddr_a), .raddr_b(hb2_mem_raddr_b),
        .dout_a(hb2_mem_rdata_a), .dout_b(hb2_mem_rdata_b)
    );

    // 每一级最多挂起一个任务；根据当前固定速率调度不会发生覆盖。
    reg pending_stage0, pending_hb1, pending_hb2;
    reg pending_stage0_phase, pending_hb1_phase, pending_hb2_phase;
    reg [6:0] pending_stage0_head, pending_stage0_fill;
    reg [5:0] pending_hb1_head, pending_hb1_fill;
    reg [5:0] pending_hb2_head, pending_hb2_fill;

    // 当前正在执行的任务上下文。
    reg busy;
    reg [1:0] job_stage;
    reg job_phase;
    reg [6:0] job_head;
    reg [6:0] job_fill;
    reg [7:0] job_op_idx;
    reg [7:0] job_ops_total;
    reg signed [MAX_ACC_W-1:0] acc_reg;

    integer taps_active;
    integer pair_ops;
    integer left_logical;
    integer right_logical;
    integer left_physical;
    integer right_physical;

    reg signed [MAX_SAMPLE_W-1:0] sample_left_ext;
    reg signed [MAX_SAMPLE_W-1:0] sample_right_ext;
    reg signed [MAX_SAMPLE_W-1:0] sample_value;
    reg signed [MAX_COEF_W-1:0] coef_value;

    // 整个模块只保留这一处乘法表达式，对应一个 DSP48。
    (* use_dsp = "yes" *)
    wire signed [PRODUCT_W-1:0] mac_product = sample_value * coef_value;

    reg signed [MAX_ACC_W-1:0] mac_term;
    reg signed [MAX_ACC_W-1:0] acc_next;
    reg signed [MAX_ACC_W-1:0] rounded_stage0;
    reg signed [MAX_ACC_W-1:0] rounded_hb1;
    reg signed [MAX_ACC_W-1:0] rounded_hb2;
    reg signed [STAGE0_OUT_W-1:0] final_stage0;
    reg signed [HB1_OUT_W-1:0] final_hb1;
    reg signed [HB2_OUT_W-1:0] final_hb2;

    // 当本拍恰好取走某一级的旧请求时，允许同级新请求无缝接替，不能误报溢出。
    wire select_hb2 = !busy && pending_hb2;
    wire select_hb1 = !busy && !pending_hb2 && pending_hb1;
    wire select_stage0 = !busy && !pending_hb2 && !pending_hb1 && pending_stage0;

    // 当前任务的循环地址读取、对称预加、乘法和舍入饱和组合逻辑。
    always @(*) begin
        taps_active = 1;
        pair_ops = 0;
        left_logical = 0;
        right_logical = 0;
        left_physical = 0;
        right_physical = 0;
        sample_left_ext = {MAX_SAMPLE_W{1'b0}};
        sample_right_ext = {MAX_SAMPLE_W{1'b0}};
        sample_value = {MAX_SAMPLE_W{1'b0}};
        coef_value = {MAX_COEF_W{1'b0}};
        stage0_mem_raddr_a = 7'd0;
        stage0_mem_raddr_b = 7'd0;
        hb1_mem_raddr_a = 6'd0;
        hb1_mem_raddr_b = 6'd0;
        hb2_mem_raddr_a = 6'd0;
        hb2_mem_raddr_b = 6'd0;

        if (busy) begin
            case (job_stage)
                JOB_STAGE0: taps_active = job_phase ? STAGE0_PH1_TAPS : STAGE0_PH0_TAPS;
                JOB_HB1:    taps_active = job_phase ? HB_PH1_TAPS : HB_PH0_TAPS;
                default:    taps_active = job_phase ? HB_PH1_TAPS : HB_PH0_TAPS;
            endcase

            pair_ops = taps_active / 2;
            left_logical = job_op_idx;
            right_logical = taps_active - 1 - job_op_idx;

            left_physical = job_head + left_logical;
            right_physical = job_head + right_logical;

            case (job_stage)
                JOB_STAGE0: begin
                    if (left_physical >= STAGE0_PH0_TAPS)
                        left_physical = left_physical - STAGE0_PH0_TAPS;
                    if (right_physical >= STAGE0_PH0_TAPS)
                        right_physical = right_physical - STAGE0_PH0_TAPS;

                    stage0_mem_raddr_a = left_physical[6:0];
                    stage0_mem_raddr_b = right_physical[6:0];

                    if (left_logical < job_fill)
                        sample_left_ext = {{(MAX_SAMPLE_W-STAGE0_IN_W){stage0_mem_rdata_a[STAGE0_IN_W-1]}},
                                           stage0_mem_rdata_a};
                    if (right_logical < job_fill)
                        sample_right_ext = {{(MAX_SAMPLE_W-STAGE0_IN_W){stage0_mem_rdata_b[STAGE0_IN_W-1]}},
                                            stage0_mem_rdata_b};

                    if (job_phase)
                        coef_value = {{(MAX_COEF_W-STAGE0_COEF_W){stage0_coef_odd[job_op_idx][STAGE0_COEF_W-1]}},
                                      stage0_coef_odd[job_op_idx]};
                    else
                        coef_value = {{(MAX_COEF_W-STAGE0_COEF_W){stage0_coef_even[job_op_idx][STAGE0_COEF_W-1]}},
                                      stage0_coef_even[job_op_idx]};
                end

                JOB_HB1: begin
                    if (left_physical >= HB_PH0_TAPS)
                        left_physical = left_physical - HB_PH0_TAPS;
                    if (right_physical >= HB_PH0_TAPS)
                        right_physical = right_physical - HB_PH0_TAPS;

                    hb1_mem_raddr_a = left_physical[5:0];
                    hb1_mem_raddr_b = right_physical[5:0];

                    if (left_logical < job_fill)
                        sample_left_ext = {{(MAX_SAMPLE_W-HB1_IN_W){hb1_mem_rdata_a[HB1_IN_W-1]}},
                                           hb1_mem_rdata_a};
                    if (right_logical < job_fill)
                        sample_right_ext = {{(MAX_SAMPLE_W-HB1_IN_W){hb1_mem_rdata_b[HB1_IN_W-1]}},
                                            hb1_mem_rdata_b};

                    if (job_phase)
                        coef_value = {{(MAX_COEF_W-HB_COEF_W){hb_coef_odd[job_op_idx][HB_COEF_W-1]}},
                                      hb_coef_odd[job_op_idx]};
                    else
                        coef_value = {{(MAX_COEF_W-HB_COEF_W){hb_coef_even[job_op_idx][HB_COEF_W-1]}},
                                      hb_coef_even[job_op_idx]};
                end

                default: begin
                    if (left_physical >= HB_PH0_TAPS)
                        left_physical = left_physical - HB_PH0_TAPS;
                    if (right_physical >= HB_PH0_TAPS)
                        right_physical = right_physical - HB_PH0_TAPS;

                    hb2_mem_raddr_a = left_physical[5:0];
                    hb2_mem_raddr_b = right_physical[5:0];

                    if (left_logical < job_fill)
                        sample_left_ext = {{(MAX_SAMPLE_W-HB2_IN_W){hb2_mem_rdata_a[HB2_IN_W-1]}},
                                           hb2_mem_rdata_a};
                    if (right_logical < job_fill)
                        sample_right_ext = {{(MAX_SAMPLE_W-HB2_IN_W){hb2_mem_rdata_b[HB2_IN_W-1]}},
                                            hb2_mem_rdata_b};

                    if (job_phase)
                        coef_value = {{(MAX_COEF_W-HB_COEF_W){hb_coef_odd[job_op_idx][HB_COEF_W-1]}},
                                      hb_coef_odd[job_op_idx]};
                    else
                        coef_value = {{(MAX_COEF_W-HB_COEF_W){hb_coef_even[job_op_idx][HB_COEF_W-1]}},
                                      hb_coef_even[job_op_idx]};
                end
            endcase

            if ((taps_active % 2) != 0 && job_op_idx == pair_ops)
                sample_value = sample_left_ext;
            else
                sample_value = sample_left_ext + sample_right_ext;
        end

        mac_term = {{(MAX_ACC_W-PRODUCT_W){mac_product[PRODUCT_W-1]}}, mac_product};
        acc_next = acc_reg + mac_term;

        if (STAGE0_SHIFT == 0)
            rounded_stage0 = acc_next;
        else if (acc_next >= 0)
            rounded_stage0 = (acc_next + (ONE_ACC <<< (STAGE0_SHIFT-1))) >>> STAGE0_SHIFT;
        else
            rounded_stage0 = -(((-acc_next) + (ONE_ACC <<< (STAGE0_SHIFT-1))) >>> STAGE0_SHIFT);

        if (HB1_SHIFT == 0)
            rounded_hb1 = acc_next;
        else if (acc_next >= 0)
            rounded_hb1 = (acc_next + (ONE_ACC <<< (HB1_SHIFT-1))) >>> HB1_SHIFT;
        else
            rounded_hb1 = -(((-acc_next) + (ONE_ACC <<< (HB1_SHIFT-1))) >>> HB1_SHIFT);

        if (HB2_SHIFT == 0)
            rounded_hb2 = acc_next;
        else if (acc_next >= 0)
            rounded_hb2 = (acc_next + (ONE_ACC <<< (HB2_SHIFT-1))) >>> HB2_SHIFT;
        else
            rounded_hb2 = -(((-acc_next) + (ONE_ACC <<< (HB2_SHIFT-1))) >>> HB2_SHIFT);

        if (rounded_stage0 > STAGE0_OUT_MAX)
            final_stage0 = STAGE0_OUT_MAX[STAGE0_OUT_W-1:0];
        else if (rounded_stage0 < STAGE0_OUT_MIN)
            final_stage0 = STAGE0_OUT_MIN[STAGE0_OUT_W-1:0];
        else
            final_stage0 = rounded_stage0[STAGE0_OUT_W-1:0];

        if (rounded_hb1 > HB1_OUT_MAX)
            final_hb1 = HB1_OUT_MAX[HB1_OUT_W-1:0];
        else if (rounded_hb1 < HB1_OUT_MIN)
            final_hb1 = HB1_OUT_MIN[HB1_OUT_W-1:0];
        else
            final_hb1 = rounded_hb1[HB1_OUT_W-1:0];

        if (rounded_hb2 > HB2_OUT_MAX)
            final_hb2 = HB2_OUT_MAX[HB2_OUT_W-1:0];
        else if (rounded_hb2 < HB2_OUT_MIN)
            final_hb2 = HB2_OUT_MIN[HB2_OUT_W-1:0];
        else
            final_hb2 = rounded_hb2[HB2_OUT_W-1:0];
    end

    // 统一请求捕获、任务调度和 MAC 状态更新。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage0_dout <= {STAGE0_OUT_W{1'b0}};
            hb1_dout <= {HB1_OUT_W{1'b0}};
            hb2_dout <= {HB2_OUT_W{1'b0}};
            stage0_dout_valid <= 1'b0;
            hb1_dout_valid <= 1'b0;
            hb2_dout_valid <= 1'b0;
            schedule_overflow <= 1'b0;

            stage0_head <= 7'd0;
            stage0_fill <= 7'd0;
            hb1_head <= 6'd0;
            hb1_fill <= 6'd0;
            hb2_head <= 6'd0;
            hb2_fill <= 6'd0;
            stage0_phase <= 1'b0;
            hb1_phase <= 1'b0;
            hb2_phase <= 1'b0;

            pending_stage0 <= 1'b0;
            pending_hb1 <= 1'b0;
            pending_hb2 <= 1'b0;
            pending_stage0_phase <= 1'b0;
            pending_hb1_phase <= 1'b0;
            pending_hb2_phase <= 1'b0;
            pending_stage0_head <= 7'd0;
            pending_stage0_fill <= 7'd0;
            pending_hb1_head <= 6'd0;
            pending_hb1_fill <= 6'd0;
            pending_hb2_head <= 6'd0;
            pending_hb2_fill <= 6'd0;

            busy <= 1'b0;
            job_stage <= JOB_STAGE0;
            job_phase <= 1'b0;
            job_head <= 7'd0;
            job_fill <= 7'd0;
            job_op_idx <= 8'd0;
            job_ops_total <= 8'd0;
            acc_reg <= {MAX_ACC_W{1'b0}};
        end else if (clr) begin
            stage0_dout <= {STAGE0_OUT_W{1'b0}};
            hb1_dout <= {HB1_OUT_W{1'b0}};
            hb2_dout <= {HB2_OUT_W{1'b0}};
            stage0_dout_valid <= 1'b0;
            hb1_dout_valid <= 1'b0;
            hb2_dout_valid <= 1'b0;
            schedule_overflow <= 1'b0;
            stage0_head <= 7'd0;
            stage0_fill <= 7'd0;
            hb1_head <= 6'd0;
            hb1_fill <= 6'd0;
            hb2_head <= 6'd0;
            hb2_fill <= 6'd0;
            stage0_phase <= 1'b0;
            hb1_phase <= 1'b0;
            hb2_phase <= 1'b0;
            pending_stage0 <= 1'b0;
            pending_hb1 <= 1'b0;
            pending_hb2 <= 1'b0;
            busy <= 1'b0;
            job_op_idx <= 8'd0;
            acc_reg <= {MAX_ACC_W{1'b0}};
        end else begin
            stage0_dout_valid <= 1'b0;
            hb1_dout_valid <= 1'b0;
            hb2_dout_valid <= 1'b0;

            // 先推进当前任务或从旧的挂起队列中选择下一任务。
            if (busy) begin
                if (job_op_idx == job_ops_total - 1'b1) begin
                    busy <= 1'b0;
                    job_op_idx <= 8'd0;
                    acc_reg <= acc_next;
                    case (job_stage)
                        JOB_STAGE0: begin
                            stage0_dout <= final_stage0;
                            stage0_dout_valid <= 1'b1;
                        end
                        JOB_HB1: begin
                            hb1_dout <= final_hb1;
                            hb1_dout_valid <= 1'b1;
                        end
                        default: begin
                            hb2_dout <= final_hb2;
                            hb2_dout_valid <= 1'b1;
                        end
                    endcase
                end else begin
                    acc_reg <= acc_next;
                    job_op_idx <= job_op_idx + 1'b1;
                end
            end else if (pending_hb2) begin
                pending_hb2 <= 1'b0;
                busy <= 1'b1;
                job_stage <= JOB_HB2;
                job_phase <= pending_hb2_phase;
                job_head <= pending_hb2_head;
                job_fill <= pending_hb2_fill;
                job_op_idx <= 8'd0;
                job_ops_total <= pending_hb2_phase ?
                    ((HB_PH1_TAPS/2) + (HB_PH1_TAPS%2)) :
                    ((HB_PH0_TAPS/2) + (HB_PH0_TAPS%2));
                acc_reg <= {MAX_ACC_W{1'b0}};
            end else if (pending_hb1) begin
                pending_hb1 <= 1'b0;
                busy <= 1'b1;
                job_stage <= JOB_HB1;
                job_phase <= pending_hb1_phase;
                job_head <= pending_hb1_head;
                job_fill <= pending_hb1_fill;
                job_op_idx <= 8'd0;
                job_ops_total <= pending_hb1_phase ?
                    ((HB_PH1_TAPS/2) + (HB_PH1_TAPS%2)) :
                    ((HB_PH0_TAPS/2) + (HB_PH0_TAPS%2));
                acc_reg <= {MAX_ACC_W{1'b0}};
            end else if (pending_stage0) begin
                pending_stage0 <= 1'b0;
                busy <= 1'b1;
                job_stage <= JOB_STAGE0;
                job_phase <= pending_stage0_phase;
                job_head <= pending_stage0_head;
                job_fill <= pending_stage0_fill;
                job_op_idx <= 8'd0;
                job_ops_total <= pending_stage0_phase ?
                    ((STAGE0_PH1_TAPS/2) + (STAGE0_PH1_TAPS%2)) :
                    ((STAGE0_PH0_TAPS/2) + (STAGE0_PH0_TAPS%2));
                acc_reg <= {MAX_ACC_W{1'b0}};
            end

            // 捕获本拍新请求。请求放在调度清除逻辑之后，使同级新请求不会丢失。
            if (ce_stage0) begin
                if (pending_stage0 && !select_stage0)
                    schedule_overflow <= 1'b1;
                pending_stage0 <= 1'b1;
                pending_stage0_phase <= stage0_phase;
                stage0_phase <= ~stage0_phase;

                if (!stage0_phase && stage0_din_valid) begin
                    if (stage0_head == 0)
                        pending_stage0_head <= STAGE0_PH0_TAPS - 1;
                    else
                        pending_stage0_head <= stage0_head - 1'b1;

                    if (stage0_head == 0) begin
                        stage0_head <= STAGE0_PH0_TAPS - 1;
                    end else begin
                        stage0_head <= stage0_head - 1'b1;
                    end

                    if (stage0_fill < STAGE0_PH0_TAPS) begin
                        stage0_fill <= stage0_fill + 1'b1;
                        pending_stage0_fill <= stage0_fill + 1'b1;
                    end else begin
                        pending_stage0_fill <= stage0_fill;
                    end
                end else begin
                    pending_stage0_head <= stage0_head;
                    pending_stage0_fill <= stage0_fill;
                end
            end

            if (ce_hb1) begin
                if (pending_hb1 && !select_hb1)
                    schedule_overflow <= 1'b1;
                pending_hb1 <= 1'b1;
                pending_hb1_phase <= hb1_phase;
                hb1_phase <= ~hb1_phase;

                if (!hb1_phase && hb1_din_valid) begin
                    if (hb1_head == 0)
                        pending_hb1_head <= HB_PH0_TAPS - 1;
                    else
                        pending_hb1_head <= hb1_head - 1'b1;

                    if (hb1_head == 0) begin
                        hb1_head <= HB_PH0_TAPS - 1;
                    end else begin
                        hb1_head <= hb1_head - 1'b1;
                    end

                    if (hb1_fill < HB_PH0_TAPS) begin
                        hb1_fill <= hb1_fill + 1'b1;
                        pending_hb1_fill <= hb1_fill + 1'b1;
                    end else begin
                        pending_hb1_fill <= hb1_fill;
                    end
                end else begin
                    pending_hb1_head <= hb1_head;
                    pending_hb1_fill <= hb1_fill;
                end
            end

            if (ce_hb2) begin
                if (pending_hb2 && !select_hb2)
                    schedule_overflow <= 1'b1;
                pending_hb2 <= 1'b1;
                pending_hb2_phase <= hb2_phase;
                hb2_phase <= ~hb2_phase;

                if (!hb2_phase && hb2_din_valid) begin
                    if (hb2_head == 0)
                        pending_hb2_head <= HB_PH0_TAPS - 1;
                    else
                        pending_hb2_head <= hb2_head - 1'b1;

                    if (hb2_head == 0) begin
                        hb2_head <= HB_PH0_TAPS - 1;
                    end else begin
                        hb2_head <= hb2_head - 1'b1;
                    end

                    if (hb2_fill < HB_PH0_TAPS) begin
                        hb2_fill <= hb2_fill + 1'b1;
                        pending_hb2_fill <= hb2_fill + 1'b1;
                    end else begin
                        pending_hb2_fill <= hb2_fill;
                    end
                end else begin
                    pending_hb2_head <= hb2_head;
                    pending_hb2_fill <= hb2_fill;
                end
            end
        end
    end

endmodule
