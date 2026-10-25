`timescale 1ns/1ps
`include "update_mem_depth.vh"

module tb_interp_filter_top;

    // 仿真参数
    parameter integer IN_W               = 24;
    parameter integer OUT_W              = 24;
    parameter integer FIFO_DEPTH         = 32;
    parameter integer OUTPUT_ALIGN_DELAY = 192;

    parameter MEM_FILE    = "test_audio_input.mem";
    parameter GOLDEN_FILE = "golden_out_128x.mem";

    parameter integer GOLDEN_ALIGN_OFFSET = 0;

    parameter integer MAX_INPUT_SAMPLES    = `MEM_DEPTH_AUTO + 16;
    parameter integer MAX_GOLDEN_SAMPLES   = `GOLDEN_OUT_DEPTH_AUTO + 4096;
    parameter integer OUTPUT_EXTRA_SAMPLES = 1024;
    parameter integer MAX_SIM_CYCLES       = 80000000;
    parameter integer DUMP_VCD             = 0;
    parameter integer WRITE_CSV            = 1;

    // 22.5792 MHz音频主时钟周期，对应512Fs时钟
    real CLK_PERIOD_NS = 44.28854875;

    // 时钟与复位
    reg clk_audio_512fs;
    reg rst_n;

    initial begin
        clk_audio_512fs = 1'b0;
        forever #(CLK_PERIOD_NS / 2.0) clk_audio_512fs = ~clk_audio_512fs;
    end

    initial begin
        rst_n = 1'b0;
        #(CLK_PERIOD_NS * 20);
        rst_n = 1'b1;
    end

    // 输入测试数据与Golden参考数据存储
    reg signed [IN_W-1:0]  input_mem  [0:MAX_INPUT_SAMPLES-1];
    reg signed [OUT_W-1:0] golden_mem [0:MAX_GOLDEN_SAMPLES-1];

    integer input_count;
    integer golden_count;

    integer mem_fd;
    integer golden_fd;
    integer scan_ret;
    integer golden_scan_ret;

    integer input_init_i;
    integer golden_init_i;

    reg [IN_W-1:0]  temp_word;
    reg [OUT_W-1:0] golden_temp_word;

    // 读取输入mem文件：要求每个有效数据行只包含一个十六进制样本
    initial begin
        for (input_init_i = 0; input_init_i < MAX_INPUT_SAMPLES; input_init_i = input_init_i + 1) begin
            input_mem[input_init_i] = {IN_W{1'b0}};
        end

        input_count = 0;

        mem_fd = $fopen(MEM_FILE, "r");
        if (mem_fd == 0) begin
            $display("ERROR: Cannot open input mem file: %s", MEM_FILE);
            $finish;
        end

        while (!$feof(mem_fd)) begin
            scan_ret = $fscanf(mem_fd, "%h\n", temp_word);

            if (scan_ret == 1) begin
                if (input_count < MAX_INPUT_SAMPLES) begin
                    input_mem[input_count] = temp_word;
                    input_count = input_count + 1;
                end else begin
                    $display("ERROR: input mem exceeds MAX_INPUT_SAMPLES = %0d", MAX_INPUT_SAMPLES);
                    $fclose(mem_fd);
                    $finish;
                end
            end else if (!$feof(mem_fd)) begin
                $display("ERROR: Invalid hexadecimal data line in input mem file: %s", MEM_FILE);
                $fclose(mem_fd);
                $finish;
            end
        end

        $fclose(mem_fd);

        if (input_count == 0) begin
            $display("ERROR: no valid samples in %s", MEM_FILE);
            $finish;
        end

        $display("INFO: Loaded %0d input samples from %s", input_count, MEM_FILE);
        $display("INFO: Expected main 128x output samples about %0d", input_count * 128);
    end

    // 读取Golden mem文件：要求每个有效数据行只包含一个十六进制样本
    initial begin
        for (golden_init_i = 0; golden_init_i < MAX_GOLDEN_SAMPLES; golden_init_i = golden_init_i + 1) begin
            golden_mem[golden_init_i] = {OUT_W{1'b0}};
        end

        golden_count = 0;

        golden_fd = $fopen(GOLDEN_FILE, "r");
        if (golden_fd == 0) begin
            $display("ERROR: Cannot open golden mem file: %s", GOLDEN_FILE);
            $finish;
        end

        while (!$feof(golden_fd)) begin
            golden_scan_ret = $fscanf(golden_fd, "%h\n", golden_temp_word);

            if (golden_scan_ret == 1) begin
                if (golden_count < MAX_GOLDEN_SAMPLES) begin
                    golden_mem[golden_count] = golden_temp_word;
                    golden_count = golden_count + 1;
                end else begin
                    $display("ERROR: golden mem exceeds MAX_GOLDEN_SAMPLES = %0d", MAX_GOLDEN_SAMPLES);
                    $fclose(golden_fd);
                    $finish;
                end
            end else if (!$feof(golden_fd)) begin
                $display("ERROR: Invalid hexadecimal data line in golden mem file: %s", GOLDEN_FILE);
                $fclose(golden_fd);
                $finish;
            end
        end

        $fclose(golden_fd);

        if (golden_count == 0) begin
            $display("ERROR: no valid samples in %s", GOLDEN_FILE);
            $finish;
        end

        $display("INFO: Loaded %0d golden output samples from %s", golden_count, GOLDEN_FILE);
    end

    // 被测插值滤波器及DA输出模块
    reg  signed [IN_W-1:0]  pcm_in;
    reg                     pcm_in_valid;
    wire                    pcm_in_ready;

    wire signed [OUT_W-1:0] pcm_out;
    wire                    pcm_out_valid;
    wire [3:0]              core_fault_flags;

    wire [7:0]              da_d;
    wire                    da_clk;

    audio_interp_core #(
        .IN_W              (IN_W),
        .OUT_W             (OUT_W),
        .FIFO_DEPTH        (FIFO_DEPTH),
        .OUTPUT_ALIGN_DELAY(OUTPUT_ALIGN_DELAY)
    ) dut_core (
        .clk_audio_512fs(clk_audio_512fs),
        .rst_n          (rst_n),
        .pcm_in         (pcm_in),
        .pcm_in_valid   (pcm_in_valid),
        .pcm_in_ready   (pcm_in_ready),
        .pcm_out        (pcm_out),
        .pcm_out_valid  (pcm_out_valid),
        .core_fault_flags(core_fault_flags)
    );

    data_output #(
        .OUT_W(OUT_W)
    ) dut_da (
        .clk_audio_512fs(clk_audio_512fs),
        .rst_n           (rst_n),
        .pcm_out         (pcm_out),
        .pcm_out_valid   (pcm_out_valid),
        .da_d            (da_d),
        .da_clk          (da_clk)
    );

    integer output_count;
    integer golden_addr;
    integer mismatch_count;

    // 主要波形观测信号
    //   wave_input_data          实际被DUT接收的24 bit输入数据
    //   wave_input_valid         输入数据接收有效脉冲
    //   wave_output_data         RTL插值滤波器24 bit输出数据
    //   wave_output_valid        RTL输出有效脉冲
    //   wave_golden_data         与RTL输出对应的Golden参考数据
    //   wave_golden_valid        Golden参考数据有效标志
    //   wave_da_data             实际送到DA接口的8 bit数据
    //   wave_da_converted_data   DA数据换算回24 bit有符号格式后的观测值
    //   da_clk                   DA输出时钟
    reg signed [IN_W-1:0]   wave_input_data;
    reg                      wave_input_valid;

    reg signed [OUT_W-1:0]  wave_output_data;
    reg                      wave_output_valid;

    reg signed [OUT_W-1:0]  wave_golden_data;
    reg                      wave_golden_valid;

    wire [7:0]               wave_da_data;
    wire signed [OUT_W-1:0] wave_da_converted_data;

    // DA接口使用8 bit偏移二进制；异或最高位后还原为8 bit有符号数，再左移16位，便于与24 bit输入、RTL输出和Golden数据同屏比较。
    wire signed [7:0] da_signed8_internal;

    assign wave_da_data = da_d;
    assign da_signed8_internal = wave_da_data ^ 8'h80;
    assign wave_da_converted_data = {da_signed8_internal, 16'd0};

    // 仿真结果文件输出
    integer input_csv_fd;
    integer output_csv_fd;
    integer da_csv_fd;

    initial begin
        input_csv_fd  = 0;
        output_csv_fd = 0;
        da_csv_fd     = 0;

        if (WRITE_CSV != 0) begin
            input_csv_fd  = $fopen("rtl_input_accepted.mem", "w");
            output_csv_fd = $fopen("rtl_output_pcm.mem", "w");
            da_csv_fd     = $fopen("rtl_da_output.mem", "w");

            if (input_csv_fd == 0) begin
                $display("ERROR: Cannot create rtl_input_accepted.mem");
                $finish;
            end

            if (output_csv_fd == 0) begin
                $display("ERROR: Cannot create rtl_output_pcm.mem");
                $finish;
            end

            if (da_csv_fd == 0) begin
                $display("ERROR: Cannot create rtl_da_output.mem");
                $finish;
            end

        end
    end

    initial begin
        if (DUMP_VCD != 0) begin
            $dumpfile("rtl_audio_interp_wave.vcd");
            $dumpvars(0, tb_interp_filter_top);
        end
    end

    // 输入数据握手驱动：仅在pcm_in_valid && pcm_in_ready时记为已接收
    integer in_idx;
    integer accepted_count;
    reg     all_inputs_done;

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            in_idx            <= 0;
            accepted_count    <= 0;
            all_inputs_done   <= 1'b0;
            pcm_in            <= {IN_W{1'b0}};
            pcm_in_valid      <= 1'b0;

            wave_input_data   <= {IN_W{1'b0}};
            wave_input_valid  <= 1'b0;
        end else begin
            wave_input_valid <= 1'b0;

            if ((pcm_in_valid == 1'b0) && (all_inputs_done == 1'b0) && (in_idx == 0)) begin
                pcm_in       <= input_mem[0];
                pcm_in_valid <= 1'b1;
            end

            if (pcm_in_valid && pcm_in_ready) begin
                wave_input_data  <= pcm_in;
                wave_input_valid <= 1'b1;

                if (WRITE_CSV != 0) begin
                    $fwrite(input_csv_fd, "%06h\n", pcm_in[IN_W-1:0]);
                end

                accepted_count <= accepted_count + 1;

                if ((in_idx + 1) < input_count) begin
                    in_idx       <= in_idx + 1;
                    pcm_in       <= input_mem[in_idx + 1];
                    pcm_in_valid <= 1'b1;
                end else begin
                    in_idx          <= in_idx + 1;
                    pcm_in          <= {IN_W{1'b0}};
                    pcm_in_valid    <= 1'b0;
                    all_inputs_done <= 1'b1;

                    $display("INFO: All %0d input samples have been accepted by DUT.", input_count);
                end
            end
        end
    end

    // 输出采集与Golden参考数据同步显示
    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            output_count       <= 0;

            wave_output_data   <= {OUT_W{1'b0}};
            wave_golden_data   <= {OUT_W{1'b0}};
            wave_output_valid  <= 1'b0;
            wave_golden_valid  <= 1'b0;
            mismatch_count     <= 0;
        end else begin
            wave_output_valid <= 1'b0;
            wave_golden_valid <= 1'b0;

            if (pcm_out_valid) begin
                golden_addr = output_count + GOLDEN_ALIGN_OFFSET;

                wave_output_data  <= pcm_out;
                wave_output_valid <= 1'b1;

                if ((golden_addr >= 0) && (golden_addr < golden_count)) begin
                    wave_golden_data  <= golden_mem[golden_addr];
                    wave_golden_valid <= 1'b1;

                    if ($signed(pcm_out) !== $signed(golden_mem[golden_addr])) begin
                        mismatch_count <= mismatch_count + 1;
                        if (mismatch_count < 10)
                            $display("ERROR: output[%0d]=%0d, golden[%0d]=%0d",
                                output_count, $signed(pcm_out),
                                golden_addr, $signed(golden_mem[golden_addr]));
                    end
                end else begin
                    wave_golden_data  <= {OUT_W{1'b0}};
                    wave_golden_valid <= 1'b0;
                end

                if (WRITE_CSV != 0) begin
                    $fwrite(output_csv_fd, "%06h\n", pcm_out[OUT_W-1:0]);

                    $fwrite(da_csv_fd, "%02h\n", wave_da_data);
                end

                output_count <= output_count + 1;
            end
        end
    end

    // 仿真结束条件与超时保护
    integer cycle_count;
    integer target_output_count;
    integer target_compare_count;

    always @(posedge clk_audio_512fs or negedge rst_n) begin
        if (!rst_n) begin
            cycle_count          <= 0;
            target_output_count  <= 0;
            target_compare_count <= 0;
        end else begin
            cycle_count <= cycle_count + 1;

            if ((all_inputs_done == 1'b1) && (target_output_count == 0)) begin
                target_compare_count = input_count * 128 + OUTPUT_EXTRA_SAMPLES;

                if (golden_count < target_compare_count) begin
                    target_output_count <= golden_count;
                end else begin
                    target_output_count <= target_compare_count;
                end

                $display("INFO: Golden samples = %0d", golden_count);
                $display("INFO: Expected output samples = %0d", input_count * 128);

                if (golden_count < target_compare_count) begin
                    $display("INFO: Target output samples = %0d", golden_count);
                end else begin
                    $display("INFO: Target output samples = %0d", target_compare_count);
                end
            end

            if ((target_output_count != 0) && (output_count >= target_output_count)) begin
                $display("INFO: Simulation finished normally.");
                $display("INFO: Accepted input samples = %0d", accepted_count);
                $display("INFO: Captured output samples = %0d", output_count);
                $display("INFO: Golden mismatches = %0d", mismatch_count);

                if (WRITE_CSV != 0) begin
                    $fclose(input_csv_fd);
                    $fclose(output_csv_fd);
                    $fclose(da_csv_fd);
                end

                if ((mismatch_count == 0) && (core_fault_flags == 4'b0000)) begin
                    $display("PASS: RTL output matches the mixed-wordlength golden vector.");
                    $finish;
                end else begin
                    $fatal(1, "FAIL: RTL/golden mismatch count = %0d, fault_flags = 0x%0h",
                        mismatch_count, core_fault_flags);
                end
            end

            if (cycle_count >= MAX_SIM_CYCLES) begin
                $display("WARNING: Simulation stopped because MAX_SIM_CYCLES was reached.");
                $display("INFO: Accepted input samples = %0d", accepted_count);
                $display("INFO: Captured output samples = %0d", output_count);

                if (WRITE_CSV != 0) begin
                    $fclose(input_csv_fd);
                    $fclose(output_csv_fd);
                    $fclose(da_csv_fd);
                end

                $finish;
            end
        end
    end

endmodule
