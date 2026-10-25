`timescale 1ns/1ps
module clock_ctrl #(
    parameter integer RESET_SYNC_STAGES = 4
)(
    input  wire sys_clk,
    input  wire sys_rst_n,
    output wire dbg_clk_20m,
    output wire clk_audio_512fs,
    output wire core_rst_n,
    output wire clk_locked
);

    // Keep the Debug Hub clock independent of the resettable audio MMCM.
    // The Clock Wizard is configured for an already-buffered input, so one
    // IBUF can feed both the MMCM and this dedicated global debug clock.
    (* keep = "true" *) wire sys_clk_ibuf;

    IBUF u_sys_clk_ibuf (
        .I(sys_clk),
        .O(sys_clk_ibuf)
    );

    (* keep = "true", dont_touch = "true" *)
    BUFG u_dbg_clk_bufg (
        .I(sys_clk_ibuf),
        .O(dbg_clk_20m)
    );

    clk_wiz_audio u_clk_wiz_audio (
        .clk_in1 (sys_clk_ibuf),
        .reset   (~sys_rst_n),
        .clk_out1(clk_audio_512fs),
        .locked  (clk_locked)
    );

    localparam integer RESET_SYNC_STAGES_SAFE = (RESET_SYNC_STAGES < 2) ? 2 : RESET_SYNC_STAGES;

    // 音频时钟域内的复位同步器
    reg [RESET_SYNC_STAGES_SAFE-1:0] rst_sync_ff;

    always @(posedge clk_audio_512fs or negedge sys_rst_n) begin
        if (!sys_rst_n || !clk_locked) begin
            rst_sync_ff <= {RESET_SYNC_STAGES_SAFE{1'b0}};
        end else begin
            rst_sync_ff <= {rst_sync_ff[RESET_SYNC_STAGES_SAFE-2:0], 1'b1};
        end
    end

    assign core_rst_n = rst_sync_ff[RESET_SYNC_STAGES_SAFE-1];

endmodule
