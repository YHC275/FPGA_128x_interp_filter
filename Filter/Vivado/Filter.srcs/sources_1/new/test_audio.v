`timescale 1ns/1ps
// 循环播放 test_audio_input.mem
// 输出接口对接 audio_interp_core 的 pcm_in / pcm_in_valid / pcm_in_ready
module test_audio #(
    parameter integer DATA_W    = 24,
    parameter integer MEM_DEPTH = 13552,
    parameter         MEM_FILE  = "test_audio_input.mem"
)(
    input  wire                     clk,
    input  wire                     rst_n,

    output wire signed [DATA_W-1:0] pcm_out,
    output wire                     pcm_valid,
    input  wire                     pcm_ready
);

    function integer clog2;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2 = (i == 0) ? 1 : i;
        end
    endfunction

    localparam integer ADDR_W = clog2(MEM_DEPTH);

    function [ADDR_W-1:0] addr_inc;
        input [ADDR_W-1:0] addr;
        begin
            if (addr == MEM_DEPTH - 1)
                addr_inc = {ADDR_W{1'b0}};
            else
                addr_inc = addr + {{(ADDR_W-1){1'b0}}, 1'b1};
        end
    endfunction

    reg  [ADDR_W-1:0] rom_addr;
    reg               rom_en;
    wire [DATA_W-1:0] rom_dout;

    xpm_memory_sprom #(
        .ADDR_WIDTH_A       (ADDR_W),
        .AUTO_SLEEP_TIME    (0),
        .CASCADE_HEIGHT     (0),
        .ECC_MODE           ("no_ecc"),
        .MEMORY_INIT_FILE   (MEM_FILE),
        .MEMORY_INIT_PARAM  ("0"),
        .MEMORY_OPTIMIZATION("true"),
        .MEMORY_PRIMITIVE   ("block"),
        .MEMORY_SIZE        (MEM_DEPTH * DATA_W),
        .MESSAGE_CONTROL    (0),
        .READ_DATA_WIDTH_A  (DATA_W),
        .READ_LATENCY_A     (1),
        .READ_RESET_VALUE_A ("0"),
        .RST_MODE_A         ("SYNC"),
        .USE_MEM_INIT       (1),
        .USE_MEM_INIT_MMI   (0),
        .WAKEUP_TIME        ("disable_sleep")
    ) u_audio_rom_xpm (
        .clka           (clk),
        .rsta           (1'b0),
        .ena            (rom_en),
        .regcea         (1'b1),
        .addra          (rom_addr),
        .douta          (rom_dout),
        .sleep          (1'b0),
        .injectsbiterra (1'b0),
        .injectdbiterra (1'b0),
        .sbiterra       (),
        .dbiterra       ()
    );

   
    // cur_sample  ：当前送给插值核心的样本
    // next_sample ：下一次 pcm_ready 来时切换过去的样本
    // fetch_addr  ：下一次需要从 ROM 请求的地址
    localparam [2:0] S_REQ0       = 3'd0;
    localparam [2:0] S_REQ1       = 3'd1;
    localparam [2:0] S_LOAD0      = 3'd2;
    localparam [2:0] S_LOAD1_RUN  = 3'd3;
    localparam [2:0] S_RUN        = 3'd4;
    localparam [2:0] S_WAIT_NEXT  = 3'd5;
    localparam [2:0] S_LOAD_NEXT  = 3'd6;

    reg [2:0] state;
    reg [ADDR_W-1:0] fetch_addr;

    reg signed [DATA_W-1:0] cur_sample;
    reg signed [DATA_W-1:0] next_sample;
    reg                     valid_reg;

    assign pcm_out   = cur_sample;
    assign pcm_valid = valid_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_REQ0;
            rom_addr         <= {ADDR_W{1'b0}};
            rom_en           <= 1'b0;
            fetch_addr       <= {ADDR_W{1'b0}};
            cur_sample       <= {DATA_W{1'b0}};
            next_sample      <= {DATA_W{1'b0}};
            valid_reg        <= 1'b0;
        end else begin
            // 默认不读 ROM；只有初始化和每次消耗一个样本后才读一次
            rom_en <= 1'b0;

            case (state)
                // 请求第 0 个样本
                S_REQ0: begin
                    rom_addr   <= {ADDR_W{1'b0}};
                    rom_en     <= 1'b1;
                    fetch_addr <= addr_inc({ADDR_W{1'b0}});
                    valid_reg  <= 1'b0;
                    state      <= S_REQ1;
                end

                // 等待第 0 个样本，同时请求第 1 个样本
                S_REQ1: begin
                    rom_addr   <= fetch_addr;
                    rom_en     <= 1'b1;
                    fetch_addr <= addr_inc(fetch_addr);
                    state      <= S_LOAD0;
                end

                // 此时 rom_dout 为第 0 个样本
                S_LOAD0: begin
                    cur_sample <= rom_dout;
                    state      <= S_LOAD1_RUN;
                end

                // 此时 rom_dout 为第 1 个样本，开始正常运行
                S_LOAD1_RUN: begin
                    next_sample <= rom_dout;
                    valid_reg   <= 1'b1;
                    state       <= S_RUN;
                end

                S_RUN: begin
                    if (pcm_ready) begin
                        // 插值核心取走当前样本，立即切换到预取好的下一个样本
                        cur_sample <= next_sample;
                        valid_reg  <= 1'b0;

                        // 再请求后续样本，供下一次 pcm_ready 使用
                        rom_addr   <= fetch_addr;
                        rom_en     <= 1'b1;
                        fetch_addr <= addr_inc(fetch_addr);

                        // 同步 ROM 在下一个时钟沿更新输出，不能立即取数
                        state <= S_WAIT_NEXT;
                    end
                end

                S_WAIT_NEXT: begin
                    // 本拍由 ROM 完成输出更新；状态机下一拍再读取新数据
                    state <= S_LOAD_NEXT;
                end

                S_LOAD_NEXT: begin
                    // 此时 rom_dout 已经对应刚才请求的地址
                    next_sample <= rom_dout;
                    valid_reg   <= 1'b1;
                    state       <= S_RUN;
                end

                default: begin
                    state <= S_REQ0;
                end
            endcase
        end
    end

endmodule
