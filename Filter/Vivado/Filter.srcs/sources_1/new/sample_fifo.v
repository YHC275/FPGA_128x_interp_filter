`timescale 1ns/1ps
// 单时钟、小深度 FIFO
module sample_fifo #(
    parameter integer WIDTH = 24,
    parameter integer DEPTH = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  clr,
    input  wire signed [WIDTH-1:0] din,
    input  wire                  wr_en,
    input  wire                  rd_en,
    output wire signed [WIDTH-1:0] dout,
    output wire                  full,
    output wire                  empty
);
    // 地址位宽。DEPTH 很小时至少保留 1 位，避免出现 0 位向量。
    localparam integer AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);

    // FIFO 存储体与读写指针。
    //
    // 该 FIFO 只有 32×24 bit，属于小容量级间缓冲。
    // 使用分布式 RAM 可以避免把每一个存储位都实现为独立触发器。
    // 存储体只进行同步写，不参与复位；复位时清空 count 即可使全部旧数据失效。
    // 异步读保持原有的首字直通行为，不增加下游取数延迟。
    (* ram_style = "distributed" *)
    reg signed [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW-1:0] wptr;
    reg [AW-1:0] rptr;
    reg [AW:0]   count;

    // 实际执行的读写条件：满时禁止写，空时禁止读。
    wire do_write = wr_en && !full;
    wire do_read  = rd_en && !empty;

    // count 直接反映 FIFO 占用量，因此 full / empty 判断很直观。
    assign empty = (count == 0);
    assign full  = (count == DEPTH);

    // 首字直通 FIFO 的关键点：
    // dout 始终指向当前读指针位置，只要非空，下游无需再额外打一拍就能看到首字。
    assign dout  = mem[rptr];

    // 分布式 RAM 的写入进程必须是纯同步写，不能带异步复位。
    // rst_n 或 clr 有效时禁止写入；存储内容本身无需清零。
    always @(posedge clk) begin
        if (rst_n && !clr && do_write)
            mem[wptr] <= din;
    end

    // 单时钟 FIFO 控制时序：
    //   - 写成功则推进 wptr
    //   - 读成功则推进 rptr
    //   - count 根据“只写/只读/同时读写”三种情况更新
    // 这里只复位指针和计数器，使 Vivado 能将 mem 正确推断为 LUTRAM。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else if (clr) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else begin
            if (do_write) begin
                if (wptr == DEPTH-1)
                    wptr <= {AW{1'b0}};
                else
                    wptr <= wptr + 1'b1;
            end

            if (do_read) begin
                if (rptr == DEPTH-1)
                    rptr <= {AW{1'b0}};
                else
                    rptr <= rptr + 1'b1;
            end

            case ({do_write, do_read})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule
