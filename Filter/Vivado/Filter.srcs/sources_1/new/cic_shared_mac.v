`timescale 1ns/1ps

// 低速 CIC 补偿 FIR，工作在 8Fs 采样率。
// 31 阶对称系数折叠成 16 次乘法，由一个 MAC 逐拍完成。
module cic_shared_mac #(
    parameter integer IN_W       = 19,
    parameter integer COEF_W     = 9,
    parameter integer COEF_FRAC  = 10,
    parameter integer OUT_W      = 19,
    parameter integer ACC_W      = 31,
    parameter integer TAPS       = 31,
    parameter COEF_FILE          = "coef_cic_comp.mem"
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    clr,
    input  wire signed [IN_W-1:0]  din,
    input  wire                    din_valid,
    output reg  signed [OUT_W-1:0] dout,
    output reg                     dout_valid,
    output reg                     input_overflow
);

    localparam integer SAMPLE_W = IN_W + 1;
    localparam integer PRODUCT_W = SAMPLE_W + COEF_W;
    localparam integer PAIR_OPS = TAPS / 2;
    localparam integer TOTAL_OPS = PAIR_OPS + (TAPS % 2);
    localparam integer DATA_DROP_SHIFT = (IN_W > OUT_W) ? (IN_W - OUT_W) : 0;
    localparam integer TOTAL_SHIFT = COEF_FRAC + DATA_DROP_SHIFT;

    localparam signed [ACC_W-1:0] ONE_ACC = {{(ACC_W-1){1'b0}}, 1'b1};
    localparam signed [ACC_W-1:0] OUT_MAX = (ONE_ACC <<< (OUT_W-1)) - 1'b1;
    localparam signed [ACC_W-1:0] OUT_MIN = -(ONE_ACC <<< (OUT_W-1));

    reg signed [COEF_W-1:0] coef_rom [0:TAPS-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < TAPS; init_i = init_i + 1) begin
            coef_rom[init_i] = {COEF_W{1'b0}};
        end
        $readmemh(COEF_FILE, coef_rom);
    end

    reg [5:0] head;
    reg [5:0] fill_count;

    // 历史样本保存在循环分布式 RAM 中。
    wire delay_mem_we = rst_n && !clr && din_valid;
    wire [5:0] delay_mem_waddr = (head == 0) ? TAPS - 1 : head - 1'b1;
    reg [5:0] delay_mem_raddr_a, delay_mem_raddr_b;
    wire signed [IN_W-1:0] delay_mem_rdata_a, delay_mem_rdata_b;

    cir_read_ram #(
        .DATA_W(IN_W), 
        .DEPTH(TAPS), 
        .ADDR_W(6)
    ) u_delay_ram (
        .clk(clk), 
        .we(delay_mem_we), 
        .waddr(delay_mem_waddr), 
        .din(din),
        .raddr_a(delay_mem_raddr_a), 
        .raddr_b(delay_mem_raddr_b),
        .dout_a(delay_mem_rdata_a), 
        .dout_b(delay_mem_rdata_b)
    );

    reg pending;
    reg [5:0] pending_head;
    reg [5:0] pending_fill;

    reg busy;
    reg [5:0] job_head;
    reg [5:0] job_fill;
    reg [5:0] op_idx;
    reg signed [ACC_W-1:0] acc_reg;

    integer left_logical;
    integer right_logical;
    integer left_physical;
    integer right_physical;
    reg signed [SAMPLE_W-1:0] sample_left;
    reg signed [SAMPLE_W-1:0] sample_right;
    reg signed [SAMPLE_W-1:0] sample_value;
    reg signed [COEF_W-1:0] coef_value;

    // 这里只有一个乘法运算，综合后使用一个 DSP48。
    (* use_dsp = "yes" *)
    wire signed [PRODUCT_W-1:0] mac_product = sample_value * coef_value;

    reg signed [ACC_W-1:0] mac_term;
    reg signed [ACC_W-1:0] acc_next;
    reg signed [ACC_W-1:0] rounded;
    reg signed [OUT_W-1:0] final_sample;

    always @(*) begin
        // 空闲时保持固定值，开始任务后再读取延时线并进行乘加。
        left_logical = 0;
        right_logical = 0;
        left_physical = 0;
        right_physical = 0;
        delay_mem_raddr_a = 6'd0;
        delay_mem_raddr_b = 6'd0;
        sample_left = {SAMPLE_W{1'b0}};
        sample_right = {SAMPLE_W{1'b0}};
        sample_value = {SAMPLE_W{1'b0}};
        coef_value = {COEF_W{1'b0}};
        mac_term = {ACC_W{1'b0}};
        acc_next = acc_reg;
        rounded = acc_reg;
        final_sample = {OUT_W{1'b0}};

        if (busy) begin
            left_logical = op_idx;
            right_logical = TAPS - 1 - op_idx;
            left_physical = job_head + left_logical;
            right_physical = job_head + right_logical;

            if (left_physical >= TAPS)
                left_physical = left_physical - TAPS;
            if (right_physical >= TAPS)
                right_physical = right_physical - TAPS;

            delay_mem_raddr_a = left_physical[5:0];
            delay_mem_raddr_b = right_physical[5:0];

            if (left_logical < job_fill)
                sample_left = {delay_mem_rdata_a[IN_W-1], delay_mem_rdata_a};
            if (right_logical < job_fill)
                sample_right = {delay_mem_rdata_b[IN_W-1], delay_mem_rdata_b};

            if ((TAPS % 2) != 0 && op_idx == PAIR_OPS)
                sample_value = sample_left;
            else
                sample_value = sample_left + sample_right;

            coef_value = coef_rom[op_idx];
            mac_term = {{(ACC_W-PRODUCT_W){mac_product[PRODUCT_W-1]}}, mac_product};
            acc_next = acc_reg + mac_term;

            if (TOTAL_SHIFT == 0)
                rounded = acc_next;
            else if (acc_next >= 0)
                rounded = (acc_next + (ONE_ACC <<< (TOTAL_SHIFT-1))) >>> TOTAL_SHIFT;
            else
                rounded = -(((-acc_next) + (ONE_ACC <<< (TOTAL_SHIFT-1))) >>> TOTAL_SHIFT);

            if (rounded > OUT_MAX)
                final_sample = OUT_MAX[OUT_W-1:0];
            else if (rounded < OUT_MIN)
                final_sample = OUT_MIN[OUT_W-1:0];
            else
                final_sample = rounded[OUT_W-1:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head <= 6'd0;
            fill_count <= 6'd0;
            pending <= 1'b0;
            pending_head <= 6'd0;
            pending_fill <= 6'd0;
            busy <= 1'b0;
            job_head <= 6'd0;
            job_fill <= 6'd0;
            op_idx <= 6'd0;
            acc_reg <= {ACC_W{1'b0}};
            dout <= {OUT_W{1'b0}};
            dout_valid <= 1'b0;
            input_overflow <= 1'b0;
        end else if (clr) begin
            head <= 6'd0;
            fill_count <= 6'd0;
            pending <= 1'b0;
            busy <= 1'b0;
            op_idx <= 6'd0;
            acc_reg <= {ACC_W{1'b0}};
            dout <= {OUT_W{1'b0}};
            dout_valid <= 1'b0;
            input_overflow <= 1'b0;
        end else begin
            dout_valid <= 1'b0;

            if (busy) begin
                if (op_idx == TOTAL_OPS - 1) begin
                    busy <= 1'b0;
                    op_idx <= 6'd0;
                    acc_reg <= acc_next;
                    dout <= final_sample;
                    dout_valid <= 1'b1;
                end else begin
                    op_idx <= op_idx + 1'b1;
                    acc_reg <= acc_next;
                end
            end else if (pending) begin
                pending <= 1'b0;
                busy <= 1'b1;
                job_head <= pending_head;
                job_fill <= pending_fill;
                op_idx <= 6'd0;
                acc_reg <= {ACC_W{1'b0}};
            end

            if (din_valid) begin
                if (pending || busy)
                    input_overflow <= 1'b1;
                pending <= 1'b1;

                if (head == 0) begin
                    head <= TAPS - 1;
                    pending_head <= TAPS - 1;
                end else begin
                    head <= head - 1'b1;
                    pending_head <= head - 1'b1;
                end

                if (fill_count < TAPS) begin
                    fill_count <= fill_count + 1'b1;
                    pending_fill <= fill_count + 1'b1;
                end else begin
                    pending_fill <= fill_count;
                end
            end
        end
    end

endmodule
