`timescale 1ns/1ps

module data_output #(
    parameter integer OUT_W = 24
)(
    input  wire                    clk_audio_512fs,
    input  wire                    rst_n,

    input  wire signed [OUT_W-1:0] pcm_out,
    input  wire                    pcm_out_valid,

    output wire [7:0]              da_d,
    output wire                    da_clk
);

    reg [7:0] da_data_reg;
    reg [1:0] da_clk_phase;
    reg       da_clk_reg;

    // 将有符号24 bit PCM数据转换为无符号偏移二进制8 bit DAC输入数据
    wire [7:0] pcm_to_da = pcm_out[OUT_W-1 -: 8] ^ 8'h80;

    // pcm_out_valid是128Fs输出有效脉冲，系统逻辑时钟为512Fs
    // 对512Fs时钟进行4分频后，可为DAC的每个有效样点提供一个采样时钟
    wire [1:0] da_clk_phase_next = pcm_out_valid ? 2'd0 : (da_clk_phase + 2'd1);

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            da_data_reg  <= 8'h80;
            da_clk_phase <= 2'd0;
            da_clk_reg   <= 1'b0;
        end else begin
            da_clk_phase <= da_clk_phase_next;
            da_clk_reg   <= da_clk_phase_next[1];

            // 在DAC时钟下降沿所对应的相位更新输出数据
            if (pcm_out_valid) begin
                da_data_reg <= pcm_to_da;
            end
        end
    end

    assign da_d   = da_data_reg;
    assign da_clk = da_clk_reg;

endmodule