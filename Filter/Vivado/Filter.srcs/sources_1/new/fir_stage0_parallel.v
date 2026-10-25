`timescale 1ns/1ps

// Pro 版第一级 2x 多相插值 FIR。
//
// 2047 个严格对称抽头拆成 1024/1023 两个多相分支。每个输出相位
// 需要 512 次对称乘加；4 条 DSP48 MAC 通路每拍处理 4 对样本，
// 128 拍完成，低于相邻 2Fs 槽位之间的 256 个 512Fs 时钟周期。
//
// 为给四条通路提供 8 个异步读端口，历史样本循环 RAM 复制四份；
// 每份 RAM 内容完全相同，分别提供一条通路的左右对称读端口。
module fir_stage0_parallel #(
    parameter integer IN_W          = 24,
    parameter integer OUT_W         = 24,
    parameter integer COEF_W        = 18,
    parameter integer COEF_FRAC     = 17,
    parameter integer ACC_W         = 48,
    parameter integer PH0_TAPS      = 1024,
    parameter integer PH1_TAPS      = 1023,
    parameter integer DEPTH         = 1024,
    parameter integer ADDR_W        = 10,
    parameter COEF_FILE_EVEN        = "coef_stage0_even.mem",
    parameter COEF_FILE_ODD         = "coef_stage0_odd.mem"
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       clr,
    input  wire                       ce_out,
    input  wire signed [IN_W-1:0]     din,
    input  wire                       din_valid,
    output reg  signed [OUT_W-1:0]    dout,
    output reg                        dout_valid,
    output reg                        schedule_overflow
);

    localparam integer LANES = 4;
    localparam integer SAMPLE_SUM_W = IN_W + 1;
    localparam integer PRODUCT_W = SAMPLE_SUM_W + COEF_W;
    localparam integer TOTAL_SHIFT = COEF_FRAC +
        ((IN_W > OUT_W) ? (IN_W - OUT_W) : 0);
    localparam [6:0] LAST_GROUP = 7'd127;

    localparam signed [ACC_W-1:0] ONE_ACC =
        {{(ACC_W-1){1'b0}}, 1'b1};
    localparam signed [ACC_W-1:0] OUT_MAX =
        (ONE_ACC <<< (OUT_W-1)) - 1'b1;
    localparam signed [ACC_W-1:0] OUT_MIN =
        -(ONE_ACC <<< (OUT_W-1));
    localparam signed [ACC_W-1:0] ROUND_CST =
        (TOTAL_SHIFT > 0) ? (ONE_ACC <<< (TOTAL_SHIFT-1)) : {ACC_W{1'b0}};

    reg signed [COEF_W-1:0] coef_even [0:PH0_TAPS-1];
    reg signed [COEF_W-1:0] coef_odd  [0:PH1_TAPS-1];

    integer init_i;
    initial begin
        if (PH0_TAPS != 1024 || PH1_TAPS != 1023 || DEPTH != 1024)
            $error("fir_stage0_parallel requires PH0/PH1/DEPTH = 1024/1023/1024");
        for (init_i = 0; init_i < PH0_TAPS; init_i = init_i + 1)
            coef_even[init_i] = {COEF_W{1'b0}};
        for (init_i = 0; init_i < PH1_TAPS; init_i = init_i + 1)
            coef_odd[init_i] = {COEF_W{1'b0}};
        $readmemh(COEF_FILE_EVEN, coef_even);
        $readmemh(COEF_FILE_ODD,  coef_odd);
    end

    // head 指向当前最新输入样本；fill 用于在复位后屏蔽尚未写入的 RAM。
    reg [ADDR_W-1:0] head;
    reg [ADDR_W:0]   fill;
    reg              phase;
    reg              busy;
    reg              job_phase;
    reg [ADDR_W-1:0] job_head;
    reg [ADDR_W:0]   job_fill;
    reg [6:0]        group_idx;
    reg signed [ACC_W-1:0] acc_reg;

    wire [ADDR_W-1:0] new_head = (head == {ADDR_W{1'b0}}) ?
        DEPTH - 1 : head - 1'b1;
    wire mem_we = rst_n && !clr && !phase && ce_out && !busy && din_valid;

    reg [ADDR_W-1:0] raddr_a0, raddr_b0;
    reg [ADDR_W-1:0] raddr_a1, raddr_b1;
    reg [ADDR_W-1:0] raddr_a2, raddr_b2;
    reg [ADDR_W-1:0] raddr_a3, raddr_b3;
    wire signed [IN_W-1:0] rdata_a0, rdata_b0;
    wire signed [IN_W-1:0] rdata_a1, rdata_b1;
    wire signed [IN_W-1:0] rdata_a2, rdata_b2;
    wire signed [IN_W-1:0] rdata_a3, rdata_b3;

    cir_read_ram #(.DATA_W(IN_W), .DEPTH(DEPTH), .ADDR_W(ADDR_W)) u_hist0 (
        .clk(clk), .we(mem_we), .waddr(new_head), .din(din),
        .raddr_a(raddr_a0), .raddr_b(raddr_b0), .dout_a(rdata_a0), .dout_b(rdata_b0));
    cir_read_ram #(.DATA_W(IN_W), .DEPTH(DEPTH), .ADDR_W(ADDR_W)) u_hist1 (
        .clk(clk), .we(mem_we), .waddr(new_head), .din(din),
        .raddr_a(raddr_a1), .raddr_b(raddr_b1), .dout_a(rdata_a1), .dout_b(rdata_b1));
    cir_read_ram #(.DATA_W(IN_W), .DEPTH(DEPTH), .ADDR_W(ADDR_W)) u_hist2 (
        .clk(clk), .we(mem_we), .waddr(new_head), .din(din),
        .raddr_a(raddr_a2), .raddr_b(raddr_b2), .dout_a(rdata_a2), .dout_b(rdata_b2));
    cir_read_ram #(.DATA_W(IN_W), .DEPTH(DEPTH), .ADDR_W(ADDR_W)) u_hist3 (
        .clk(clk), .we(mem_we), .waddr(new_head), .din(din),
        .raddr_a(raddr_a3), .raddr_b(raddr_b3), .dout_a(rdata_a3), .dout_b(rdata_b3));

    wire [9:0] op0 = {1'b0, group_idx, 2'b00};
    wire [9:0] op1 = op0 + 10'd1;
    wire [9:0] op2 = op0 + 10'd2;
    wire [9:0] op3 = op0 + 10'd3;

    function [ADDR_W-1:0] wrap_addr;
        input [ADDR_W-1:0] base;
        input [10:0] logical_index;
        integer tmp;
        begin
            tmp = base + logical_index;
            if (tmp >= DEPTH)
                tmp = tmp - DEPTH;
            wrap_addr = tmp[ADDR_W-1:0];
        end
    endfunction

    reg [10:0] right0, right1, right2, right3;
    reg signed [SAMPLE_SUM_W-1:0] sample_sum0, sample_sum1;
    reg signed [SAMPLE_SUM_W-1:0] sample_sum2, sample_sum3;
    reg signed [COEF_W-1:0] coef0, coef1, coef2, coef3;

    reg signed [SAMPLE_SUM_W-1:0] left0, left1, left2, left3;
    reg signed [SAMPLE_SUM_W-1:0] right_data0, right_data1;
    reg signed [SAMPLE_SUM_W-1:0] right_data2, right_data3;

    // 地址、启动填充屏蔽、对称预加和系数选择。
    always @(*) begin
        right0 = (job_phase ? 11'd1022 : 11'd1023) - op0;
        right1 = (job_phase ? 11'd1022 : 11'd1023) - op1;
        right2 = (job_phase ? 11'd1022 : 11'd1023) - op2;
        right3 = (job_phase ? 11'd1022 : 11'd1023) - op3;

        raddr_a0 = wrap_addr(job_head, {1'b0, op0});
        raddr_b0 = wrap_addr(job_head, right0);
        raddr_a1 = wrap_addr(job_head, {1'b0, op1});
        raddr_b1 = wrap_addr(job_head, right1);
        raddr_a2 = wrap_addr(job_head, {1'b0, op2});
        raddr_b2 = wrap_addr(job_head, right2);
        raddr_a3 = wrap_addr(job_head, {1'b0, op3});
        raddr_b3 = wrap_addr(job_head, right3);

        left0 = ({1'b0, op0} < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_a0[IN_W-1]}}, rdata_a0} : {SAMPLE_SUM_W{1'b0}};
        left1 = ({1'b0, op1} < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_a1[IN_W-1]}}, rdata_a1} : {SAMPLE_SUM_W{1'b0}};
        left2 = ({1'b0, op2} < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_a2[IN_W-1]}}, rdata_a2} : {SAMPLE_SUM_W{1'b0}};
        left3 = ({1'b0, op3} < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_a3[IN_W-1]}}, rdata_a3} : {SAMPLE_SUM_W{1'b0}};

        right_data0 = (right0 < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_b0[IN_W-1]}}, rdata_b0} : {SAMPLE_SUM_W{1'b0}};
        right_data1 = (right1 < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_b1[IN_W-1]}}, rdata_b1} : {SAMPLE_SUM_W{1'b0}};
        right_data2 = (right2 < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_b2[IN_W-1]}}, rdata_b2} : {SAMPLE_SUM_W{1'b0}};
        right_data3 = (right3 < job_fill) ?
            {{(SAMPLE_SUM_W-IN_W){rdata_b3[IN_W-1]}}, rdata_b3} : {SAMPLE_SUM_W{1'b0}};

        // 奇相 op=511 是 1023-tap 分支的中心抽头，只读取一次样本。
        sample_sum0 = (job_phase && op0 == 10'd511) ? left0 : left0 + right_data0;
        sample_sum1 = (job_phase && op1 == 10'd511) ? left1 : left1 + right_data1;
        sample_sum2 = (job_phase && op2 == 10'd511) ? left2 : left2 + right_data2;
        sample_sum3 = (job_phase && op3 == 10'd511) ? left3 : left3 + right_data3;

        coef0 = job_phase ? coef_odd[op0] : coef_even[op0];
        coef1 = job_phase ? coef_odd[op1] : coef_even[op1];
        coef2 = job_phase ? coef_odd[op2] : coef_even[op2];
        coef3 = job_phase ? coef_odd[op3] : coef_even[op3];
    end

    (* use_dsp = "yes" *) wire signed [PRODUCT_W-1:0] product0 = sample_sum0 * coef0;
    (* use_dsp = "yes" *) wire signed [PRODUCT_W-1:0] product1 = sample_sum1 * coef1;
    (* use_dsp = "yes" *) wire signed [PRODUCT_W-1:0] product2 = sample_sum2 * coef2;
    (* use_dsp = "yes" *) wire signed [PRODUCT_W-1:0] product3 = sample_sum3 * coef3;

    wire signed [ACC_W-1:0] product0_ext =
        {{(ACC_W-PRODUCT_W){product0[PRODUCT_W-1]}}, product0};
    wire signed [ACC_W-1:0] product1_ext =
        {{(ACC_W-PRODUCT_W){product1[PRODUCT_W-1]}}, product1};
    wire signed [ACC_W-1:0] product2_ext =
        {{(ACC_W-PRODUCT_W){product2[PRODUCT_W-1]}}, product2};
    wire signed [ACC_W-1:0] product3_ext =
        {{(ACC_W-PRODUCT_W){product3[PRODUCT_W-1]}}, product3};
    wire signed [ACC_W-1:0] group_sum =
        product0_ext + product1_ext + product2_ext + product3_ext;
    wire signed [ACC_W-1:0] acc_after_group =
        (group_idx == 7'd0) ? group_sum : (acc_reg + group_sum);

    reg signed [ACC_W-1:0] rounded_value;
    reg signed [OUT_W-1:0] saturated_value;
    always @(*) begin
        if (TOTAL_SHIFT == 0)
            rounded_value = acc_after_group;
        else if (acc_after_group >= 0)
            rounded_value = (acc_after_group + ROUND_CST) >>> TOTAL_SHIFT;
        else
            rounded_value = -(((-acc_after_group) + ROUND_CST) >>> TOTAL_SHIFT);

        if (rounded_value > OUT_MAX)
            saturated_value = OUT_MAX[OUT_W-1:0];
        else if (rounded_value < OUT_MIN)
            saturated_value = OUT_MIN[OUT_W-1:0];
        else
            saturated_value = rounded_value[OUT_W-1:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head              <= {ADDR_W{1'b0}};
            fill              <= {(ADDR_W+1){1'b0}};
            phase             <= 1'b0;
            busy              <= 1'b0;
            job_phase         <= 1'b0;
            job_head          <= {ADDR_W{1'b0}};
            job_fill          <= {(ADDR_W+1){1'b0}};
            group_idx         <= 7'd0;
            acc_reg           <= {ACC_W{1'b0}};
            dout              <= {OUT_W{1'b0}};
            dout_valid        <= 1'b0;
            schedule_overflow <= 1'b0;
        end else if (clr) begin
            head              <= {ADDR_W{1'b0}};
            fill              <= {(ADDR_W+1){1'b0}};
            phase             <= 1'b0;
            busy              <= 1'b0;
            job_phase         <= 1'b0;
            job_head          <= {ADDR_W{1'b0}};
            job_fill          <= {(ADDR_W+1){1'b0}};
            group_idx         <= 7'd0;
            acc_reg           <= {ACC_W{1'b0}};
            dout              <= {OUT_W{1'b0}};
            dout_valid        <= 1'b0;
            schedule_overflow <= 1'b0;
        end else begin
            dout_valid <= 1'b0;

            if (busy) begin
                acc_reg <= acc_after_group;
                if (group_idx == LAST_GROUP) begin
                    busy       <= 1'b0;
                    group_idx  <= 7'd0;
                    dout       <= saturated_value;
                    dout_valid <= 1'b1;
                end else begin
                    group_idx <= group_idx + 1'b1;
                end
            end

            if (ce_out) begin
                if (busy) begin
                    schedule_overflow <= 1'b1;
                end else begin
                    busy      <= 1'b1;
                    job_phase <= phase;
                    phase     <= ~phase;
                    group_idx <= 7'd0;
                    acc_reg   <= {ACC_W{1'b0}};

                    if (!phase && din_valid) begin
                        head     <= new_head;
                        job_head <= new_head;
                        if (fill < DEPTH) begin
                            fill     <= fill + 1'b1;
                            job_fill <= fill + 1'b1;
                        end else begin
                            job_fill <= fill;
                        end
                    end else begin
                        job_head <= head;
                        job_fill <= fill;
                    end
                end
            end
        end
    end

endmodule
