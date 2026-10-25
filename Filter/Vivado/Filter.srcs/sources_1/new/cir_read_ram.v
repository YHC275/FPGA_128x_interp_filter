`timescale 1ns/1ps

// 循环延时线使用的简单双异步读、单同步写分布式 RAM。
// 该写法采用 Vivado 的标准 RAM 推断模板；存储体不清零，
// 上层模块通过有效样本计数屏蔽尚未写入的位置。
module cir_read_ram #(
    parameter integer DATA_W = 19,
    parameter integer DEPTH  = 48,
    parameter integer ADDR_W = 6
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [ADDR_W-1:0]        waddr,
    input  wire signed [DATA_W-1:0] din,
    input  wire [ADDR_W-1:0]        raddr_a,
    input  wire [ADDR_W-1:0]        raddr_b,
    output wire signed [DATA_W-1:0] dout_a,
    output wire signed [DATA_W-1:0] dout_b
);

    (* ram_style = "distributed" *)
    reg signed [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= din;
    end

    assign dout_a = mem[raddr_a];
    assign dout_b = mem[raddr_b];

endmodule
