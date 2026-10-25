`timescale 1ns/1ps
`include "update_mem_depth.vh"
`include "mixed_wordlength_params.vh"
//     test_audio_input.mem -> XPM BRAM ROM -> 128x 插值 -> 8-bit 高速 DA 输出

module interp_filter_top #(
    parameter integer IN_W               = 24,
    parameter integer OUT_W              = 24,
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
    parameter integer OUTPUT_ALIGN_DELAY = 192,
    parameter integer MEM_DEPTH = `MEM_DEPTH_AUTO
)(
    input  wire                    sys_clk,
    input  wire                    sys_rst_n,

    output wire [7:0]              DA_D,
    output wire                    DA_CLK,

    output wire                    clk_locked
);

    wire clk_audio_512fs;
    (* keep = "true", dont_touch = "true" *) wire dbg_clk_20m;
    wire core_rst_n;

    wire signed [IN_W-1:0]  mem_pcm_in;
    wire                   mem_pcm_valid;
    wire                   mem_pcm_ready;

    wire signed [OUT_W-1:0] pcm_out;
    wire                   pcm_out_valid;
    wire [3:0]             core_fault_flags;

    (* mark_debug = "true", keep = "true" *) reg signed [OUT_W-1:0] ila_pcm_out_q;
    (* mark_debug = "true", keep = "true" *) reg signed [OUT_W-1:0] ila_pcm_out_prev_q;
    (* mark_debug = "true", keep = "true" *) reg [31:0] board_output_count;
    (* mark_debug = "true", keep = "true" *) reg ila_sample_toggle;
    (* mark_debug = "true", keep = "true" *) reg ila_pcm_changed;
    (* mark_debug = "true", keep = "true", dont_touch = "true" *) wire ila_pcm_out_valid;
    (* mark_debug = "true", keep = "true", dont_touch = "true" *) wire ila_core_rst_n;
    (* mark_debug = "true", keep = "true", dont_touch = "true" *) reg [7:0] ila_status_q;

    assign ila_pcm_out_valid = pcm_out_valid;
    assign ila_core_rst_n = core_rst_n;
    wire ila_pcm_changed_now = pcm_out_valid && (pcm_out != ila_pcm_out_q);

    // {reset_ok, clk_locked, mem_valid, mem_ready, out_valid, da_clk, sample_changed, sample_nonzero}
    always @(posedge clk_audio_512fs or negedge core_rst_n) begin
        if (!core_rst_n) begin
            board_output_count  <= 32'd0;
            ila_pcm_out_q       <= {OUT_W{1'b0}};
            ila_pcm_out_prev_q  <= {OUT_W{1'b0}};
            ila_sample_toggle   <= 1'b0;
            ila_pcm_changed     <= 1'b0;
            ila_status_q        <= 8'd0;
        end else begin
            ila_pcm_changed <= 1'b0;
            ila_status_q <= {
                1'b1,
                clk_locked,
                mem_pcm_valid,
                mem_pcm_ready,
                pcm_out_valid,
                DA_CLK,
                ila_pcm_changed_now,
                |ila_pcm_out_q
            };

            if (pcm_out_valid) begin
                board_output_count  <= board_output_count + 32'd1;
                ila_pcm_out_prev_q  <= ila_pcm_out_q;
                ila_pcm_out_q       <= pcm_out;
                ila_sample_toggle   <= ~ila_sample_toggle;
                ila_pcm_changed     <= ila_pcm_changed_now;
            end
        end
    end

    clock_ctrl u_clock_ctrl (
        .sys_clk          (sys_clk),
        .sys_rst_n        (sys_rst_n),
        .dbg_clk_20m      (dbg_clk_20m),
        .clk_audio_512fs  (clk_audio_512fs),
        .core_rst_n       (core_rst_n),
        .clk_locked       (clk_locked)
    );

    test_audio #(
        .DATA_W   (IN_W),
        .MEM_DEPTH(MEM_DEPTH),
        .MEM_FILE ("test_audio_input.mem")
    ) u_test_audio (
        .clk       (clk_audio_512fs),
        .rst_n     (core_rst_n),
        .pcm_out   (mem_pcm_in),
        .pcm_valid (mem_pcm_valid),
        .pcm_ready (mem_pcm_ready)
    );

    audio_interp_core #(
        .IN_W              (IN_W),
        .OUT_W             (OUT_W),
        .STAGE0_COEF_W     (STAGE0_COEF_W),
        .STAGE0_COEF_FRAC  (STAGE0_COEF_FRAC),
        .STAGE0_ACC_W      (STAGE0_ACC_W),
        .HALFBAND_COEF_W   (HALFBAND_COEF_W),
        .HALFBAND_COEF_FRAC(HALFBAND_COEF_FRAC),
        .HALFBAND_ACC_W    (HALFBAND_ACC_W),
        .CIC_ACC_W         (CIC_ACC_W),
        .CIC_COMP_COEF_W   (CIC_COMP_COEF_W),
        .CIC_COMP_COEF_FRAC(CIC_COMP_COEF_FRAC),
        .CIC_COMP_ACC_W    (CIC_COMP_ACC_W),
        .FIFO_DEPTH        (FIFO_DEPTH),
        .OUTPUT_ALIGN_DELAY(OUTPUT_ALIGN_DELAY)
    ) u_audio_interp_core (
        .clk_audio_512fs(clk_audio_512fs),
        .rst_n          (core_rst_n),
        .pcm_in         (mem_pcm_in),
        .pcm_in_valid   (mem_pcm_valid),
        .pcm_in_ready   (mem_pcm_ready),
        .pcm_out        (pcm_out),
        .pcm_out_valid  (pcm_out_valid),
        .core_fault_flags(core_fault_flags)
    );

    data_output #(
        .OUT_W(OUT_W)
    ) u_data_output (
        .clk_audio_512fs(clk_audio_512fs),
        .rst_n           (core_rst_n),
        .pcm_out         (pcm_out),
        .pcm_out_valid   (pcm_out_valid),
        .da_d            (DA_D),
        .da_clk          (DA_CLK)
    );

    ila_pcm_capture u_ila_pcm_capture (
        .clk    (clk_audio_512fs),
        .probe0 (ila_pcm_out_q),
        .probe1 (ila_pcm_out_valid),
        .probe2 (board_output_count),
        .probe3 (ila_status_q),
        .probe4 (ila_core_rst_n)
    );

endmodule
