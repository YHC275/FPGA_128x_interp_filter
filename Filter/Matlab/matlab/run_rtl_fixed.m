function [y_int24, trace] = run_rtl_fixed(x_int24, q_stage0, q_halfband, q_cic_comp, cfg, n_out)

    if nargin < 6 || isempty(n_out)
        n_out = numel(x_int24) * cfg.interp_factor;
    end

    x_int24    = int64(x_int24(:));
    q_stage0   = int64(q_stage0(:));
    q_halfband = int64(q_halfband(:));
    q_cic_comp = int64(q_cic_comp(:));

    q_stage0_even = q_stage0(1:2:end);
    q_stage0_odd  = q_stage0(2:2:end);
    q_hb_even     = q_halfband(1:2:end);
    q_hb_odd      = q_halfband(2:2:end);

    % 初始化各 RTL 子模块状态
    top = struct();
    top.slot_cnt      = uint8(0);
    top.rate_cnt      = uint8(0);
    top.phase_ref     = false(1, 3);
    top.out_delay     = zeros(max(1, cfg.output_align_delay), 1, 'int64');
    top.pcm_out       = int64(0);
    top.pcm_out_valid = false;
    top.comp_prefill_count = uint8(0);
    top.comp_rate_started  = false;

    stage0 = init_interp2_state(cfg.stage0_ph0_taps);
    stage1 = init_interp2_state(cfg.hb_ph0_taps);
    stage2 = init_interp2_state(cfg.hb_ph0_taps);
    cic    = init_cic_state(cfg);
    comp   = init_comp_state(cfg.cic_comp_taps);

    fifo01 = init_fifo_state(cfg.fifo_depth);
    fifo12 = init_fifo_state(cfg.fifo_depth);
    comp_fifo = init_fifo_state(8);

    y_int24 = zeros(n_out, 1, 'int64');
    out_ptr = 1;
    in_ptr  = 1;

    max_cycles = max(4096, n_out * cfg.slot_div_512_to_128 + cfg.output_align_delay * 16 + 20000);
    cycle = 0;

    % 可选跟踪信息，便于调试节拍
    trace = struct();
    trace.input_accepted = 0;
    trace.output_valid_cycles = zeros(min(n_out, 4096), 1);
    trace.first_output_cycle = [];

    while out_ptr <= n_out
        cycle = cycle + 1;
        if cycle > max_cycles
            error('run_rtl_fixed: 超过最大仿真周期，可能输入 flush 不足或模型状态异常');
        end

        %  顶层组合信号：使用“当前拍边沿之前”的寄存器状态
        slot_ce = (top.slot_cnt == 0);

        ce0 = slot_ce && (bitand(top.rate_cnt, uint8(63)) == 0);
        ce1 = slot_ce && (bitand(top.rate_cnt, uint8(31)) == 0);
        ce2 = slot_ce && (bitand(top.rate_cnt, uint8(15)) == 0);

        stage0_in_slot = ce0 && ~top.phase_ref(1);
        stage1_in_slot = ce1 && ~top.phase_ref(2);
        stage2_in_slot = ce2 && ~top.phase_ref(3);

        pcm_in_valid = stage0_in_slot && (in_ptr <= numel(x_int24));
        if pcm_in_valid
            pcm_in = x_int24(in_ptr);
        else
            pcm_in = int64(0);
        end

        fifo01_empty = (fifo01.count == 0);
        fifo12_empty = (fifo12.count == 0);
        fifo01_dout = fifo_front(fifo01);
        fifo12_dout = fifo_front(fifo12);
        comp_fifo_dout = fifo_front(comp_fifo);

        %  所有子模块在同一个 clk 上升沿更新
        %  FIFO / CIC / 顶层输出看到的是“上一拍已经寄存”的 valid
        stage0_next = interp2_rtl_step(stage0, ce0, pcm_in, pcm_in_valid && stage0_in_slot, ...
            q_stage0_even, q_stage0_odd, ...
            cfg.stage0_ph0_taps, cfg.stage0_ph1_taps, ...
            cfg.stage0_coef_frac, cfg.in_width, ...
            cfg.stage0_data_width, cfg.stage0_acc_width, cfg.stage0_mac_lanes);

        stage1_next = interp2_rtl_step(stage1, ce1, fifo01_dout, stage1_in_slot && ~fifo01_empty, ...
            q_hb_even, q_hb_odd, ...
            cfg.hb_ph0_taps, cfg.hb_ph1_taps, ...
            cfg.halfband_coef_frac, cfg.stage0_data_width, ...
            cfg.halfband1_data_width, cfg.halfband_acc_width, 1);

        stage2_next = interp2_rtl_step(stage2, ce2, fifo12_dout, stage2_in_slot && ~fifo12_empty, ...
            q_hb_even, q_hb_odd, ...
            cfg.hb_ph0_taps, cfg.hb_ph1_taps, ...
            cfg.halfband_coef_frac, cfg.halfband1_data_width, ...
            cfg.halfband2_data_width, cfg.halfband_acc_width, 1);

        fifo01_next = fifo_rtl_step(fifo01, stage0.dout, stage0.dout_valid, stage1_in_slot && ~fifo01_empty);
        fifo12_next = fifo_rtl_step(fifo12, stage1.dout, stage1.dout_valid, stage2_in_slot && ~fifo12_empty);

        % 新结构：补偿 FIR 在 8Fs 低速侧先运行，再送入 16x CIC
        comp_next = comp_rtl_step(comp, stage2.dout, stage2.dout_valid, q_cic_comp, cfg);
        % 共享 MAC 的完成脉冲存在小幅时刻摆动。补偿输出先写入弹性 FIFO，
        % 再由固定 ce2 节拍读出，保证 CIC 的输入间隔严格为 16 个输出槽位
        comp_fifo_empty = (comp_fifo.count == 0);
        comp_fifo_full  = (comp_fifo.count == comp_fifo.depth);
        comp_fifo_read = ce2 && top.comp_rate_started && ~comp_fifo_empty;
        comp_fifo_next = fifo_rtl_step( ...
            comp_fifo, comp.dout, comp.dout_valid, comp_fifo_read);
        cic_next = cic_rtl_step(cic, slot_ce, comp_fifo_dout, comp_fifo_read, cfg);

        [top_next, produced_valid, produced_sample] = top_rtl_step( ...
            top, cic.dout, cic.dout_valid, slot_ce, ce0, ce1, ce2, cfg);

        if ~top.comp_rate_started && comp.dout_valid && ~comp_fifo_full
            if top.comp_prefill_count == uint8(1)
                top_next.comp_prefill_count = uint8(2);
                top_next.comp_rate_started = true;
            else
                top_next.comp_prefill_count = top.comp_prefill_count + uint8(1);
            end
        end

        % 上游握手：如果本拍 ready 且 valid，则在边沿后视为样本已被 RTL 接收
        if pcm_in_valid
            in_ptr = in_ptr + 1;
            trace.input_accepted = trace.input_accepted + 1;
        end

        % 外部 testbench 通常在时钟边沿后采样 pcm_out / pcm_out_valid
        if produced_valid
            y_int24(out_ptr) = produced_sample;
            if out_ptr <= numel(trace.output_valid_cycles)
                trace.output_valid_cycles(out_ptr) = cycle;
            end
            if isempty(trace.first_output_cycle)
                trace.first_output_cycle = cycle;
            end
            out_ptr = out_ptr + 1;
        end

        % 提交所有 next 状态
        top    = top_next;
        stage0 = stage0_next;
        stage1 = stage1_next;
        stage2 = stage2_next;
        fifo01 = fifo01_next;
        fifo12 = fifo12_next;
        comp_fifo = comp_fifo_next;
        cic    = cic_next;
        comp   = comp_next;
    end

    trace.total_cycles = cycle;
    trace.generated_outputs = n_out;
end

function y = wrap_signed_local(x, width)
% 与 RTL 寄存器写入时的二进制补码截断行为保持一致
    modulus = 2 ^ width;
    half = 2 ^ (width - 1);
    u = mod(double(x), modulus);
    if u >= half
        u = u - modulus;
    end
    y = int64(u);
end

% 子模块：2x 多相 FIR 的 RTL 等效状态机
function s = init_interp2_state(delay_len)
    s = struct();
    s.phase      = false;
    s.busy       = false;
    s.calc_phase = false;
    s.op_idx     = uint16(0);
    s.ops_total  = uint16(0);
    s.acc_reg    = int64(0);
    s.delay      = zeros(delay_len, 1, 'int64');
    s.calc_delay = zeros(delay_len, 1, 'int64');
    s.dout       = int64(0);
    s.dout_valid = false;
end

function s_next = interp2_rtl_step(s, ce_out, din, din_valid, q_even, q_odd, ph0_taps, ph1_taps, frac, in_width, out_width, acc_width, ops_per_cycle)
    s_next = s;
    s_next.dout_valid = false;

    ph0_ops = floor(ph0_taps / 2) + mod(ph0_taps, 2);
    ph1_ops = floor(ph1_taps / 2) + mod(ph1_taps, 2);

    if ~s.busy
        if ce_out
            work_delay = s.delay;
            if ~s.phase && din_valid
                work_delay = shift_in(work_delay, din);
            end

            s_next.calc_delay = work_delay;
            if ~s.phase && din_valid
                s_next.delay = work_delay;
            end

            if ~s.phase
                ops = ph0_ops;
                start_term = interp2_mac_group(work_delay, q_even, ph0_taps, 0, ops_per_cycle);
            else
                ops = ph1_ops;
                start_term = interp2_mac_group(work_delay, q_odd, ph1_taps, 0, ops_per_cycle);
            end

            groups = ceil(ops / ops_per_cycle);

            s_next.calc_phase = s.phase;
            s_next.phase      = ~s.phase;
            start_term = wrap_signed_local(start_term, acc_width);
            s_next.acc_reg    = start_term;
            s_next.ops_total  = uint16(groups);

            if groups == 1
                s_next.busy       = false;
                s_next.op_idx     = uint16(0);
                total_shift = frac + max(in_width - out_width, 0);
                s_next.dout       = saturate_signed(round_shift_signed(start_term, total_shift), out_width);
                s_next.dout_valid = true;
            else
                s_next.busy   = true;
                s_next.op_idx = uint16(1);
            end
        end
    else
        op = double(s.op_idx);
        if ~s.calc_phase
            term = interp2_mac_group(s.calc_delay, q_even, ph0_taps, op, ops_per_cycle);
        else
            term = interp2_mac_group(s.calc_delay, q_odd, ph1_taps, op, ops_per_cycle);
        end
        acc_next = wrap_signed_local(s.acc_reg + term, acc_width);

        if double(s.op_idx) == double(s.ops_total) - 1
            s_next.busy       = false;
            s_next.op_idx     = uint16(0);
            s_next.acc_reg    = acc_next;
            total_shift = frac + max(in_width - out_width, 0);
            s_next.dout       = saturate_signed(round_shift_signed(acc_next, total_shift), out_width);
            s_next.dout_valid = true;
        else
            s_next.acc_reg = acc_next;
            s_next.op_idx  = s.op_idx + uint16(1);
        end
    end
end

function term = interp2_mac_group(delay, coef, taps, group_idx, ops_per_cycle)
    pair_ops = floor(taps / 2);
    term = int64(0);
    first_op = double(group_idx) * double(ops_per_cycle);

    for lane = 0:(double(ops_per_cycle) - 1)
        op_idx = first_op + lane;
        if op_idx < pair_ops
            left  = 1 + op_idx;
            right = taps - op_idx;
            term = term + ...
                (int64(delay(left)) + int64(delay(right))) * int64(coef(left));
        elseif mod(taps, 2) ~= 0 && op_idx == pair_ops
            center = 1 + pair_ops;
            term = term + int64(delay(center)) * int64(coef(center));
        end
    end
end

function y = shift_in(x, v)
    y = x;
    if isempty(y)
        return;
    end
    if numel(y) > 1
        y(2:end) = x(1:end-1);
    end
    y(1) = int64(v);
end

% 子模块：FWFT FIFO
function f = init_fifo_state(depth)
    f = struct();
    f.depth = depth;
    f.data  = zeros(depth, 1, 'int64');
    f.wptr  = 1;
    f.rptr  = 1;
    f.count = 0;
end

function dout = fifo_front(f)
    if f.count == 0
        dout = int64(0);
    else
        dout = f.data(f.rptr);
    end
end

function f_next = fifo_rtl_step(f, din, wr_en, rd_en)
    f_next = f;
    full  = (f.count == f.depth);
    empty = (f.count == 0);

    do_write = wr_en && ~full;
    do_read  = rd_en && ~empty;

    if do_write
        f_next.data(f.wptr) = int64(din);
        if f.wptr == f.depth
            f_next.wptr = 1;
        else
            f_next.wptr = f.wptr + 1;
        end
    end

    if do_read
        if f.rptr == f.depth
            f_next.rptr = 1;
        else
            f_next.rptr = f.rptr + 1;
        end
    end

    if do_write && ~do_read
        f_next.count = f.count + 1;
    elseif ~do_write && do_read
        f_next.count = f.count - 1;
    else
        f_next.count = f.count;
    end
end

% 子模块：CIC 插值器
function s = init_cic_state(cfg)
    s = struct();
    s.comb_delay   = zeros(cfg.cic_stages, 1, 'int64');
    s.integ_reg    = zeros(cfg.cic_stages, 1, 'int64');
    s.burst_remain = uint16(0);
    s.burst_active = false;
    s.dout         = int64(0);
    s.dout_valid   = false;
end

function s_next = cic_rtl_step(s, ce_out, din, din_valid, cfg)
    s_next = s;
    s_next.dout_valid = false;

    if ~ce_out
        return;
    end

    din_ext = int64(din);
    comb_work = zeros(cfg.cic_stages, 1, 'int64');
    comb_work(1) = wrap_signed_local( ...
        din_ext - s.comb_delay(1), cfg.cic_acc_width);
    for k = 2:cfg.cic_stages
        comb_work(k) = wrap_signed_local( ...
            comb_work(k-1) - s.comb_delay(k), cfg.cic_acc_width);
    end

    if din_valid
        cic_drive = comb_work(cfg.cic_stages);
    else
        cic_drive = int64(0);
    end

    integ_work = zeros(cfg.cic_stages, 1, 'int64');
    integ_work(1) = wrap_signed_local( ...
        s.integ_reg(1) + cic_drive, cfg.cic_acc_width);
    for k = 2:cfg.cic_stages
        integ_work(k) = wrap_signed_local( ...
            s.integ_reg(k) + integ_work(k-1), cfg.cic_acc_width);
    end

    if din_valid
        s_next.comb_delay(1) = din_ext;
        for k = 2:cfg.cic_stages
            s_next.comb_delay(k) = comb_work(k-1);
        end
        s_next.integ_reg = integ_work;
        total_shift = cfg.cic_gain_shift + max( ...
            cfg.cic_comp_data_width - cfg.cic_data_width, 0);
        s_next.dout = saturate_signed(round_shift_signed( ...
            integ_work(cfg.cic_stages), total_shift), cfg.cic_data_width);
        s_next.dout_valid = true;

        if cfg.cic_rate > 1
            s_next.burst_remain = uint16(cfg.cic_rate - 1);
            s_next.burst_active = true;
        else
            s_next.burst_remain = uint16(0);
            s_next.burst_active = false;
        end
    elseif s.burst_active
        s_next.integ_reg = integ_work;
        total_shift = cfg.cic_gain_shift + max( ...
            cfg.cic_comp_data_width - cfg.cic_data_width, 0);
        s_next.dout = saturate_signed(round_shift_signed( ...
            integ_work(cfg.cic_stages), total_shift), cfg.cic_data_width);
        s_next.dout_valid = true;

        if s.burst_remain == uint16(1)
            s_next.burst_remain = uint16(0);
            s_next.burst_active = false;
        else
            s_next.burst_remain = s.burst_remain - uint16(1);
            s_next.burst_active = true;
        end
    end
end

% 子模块：CIC 补偿 FIR
function s = init_comp_state(taps)
    s = struct();
    s.busy         = false;
    s.group_idx    = uint8(0);
    s.groups_total = uint8(0);
    s.acc_reg      = int64(0);
    s.delay        = zeros(taps, 1, 'int64');
    s.calc_delay   = zeros(taps, 1, 'int64');
    s.dout         = int64(0);
    s.dout_valid   = false;
end

function s_next = comp_rtl_step(s, din, din_valid, q_coef, cfg)
    s_next = s;
    s_next.dout_valid = false;

    taps = cfg.cic_comp_taps;
    start_idx = 1;
    total_ops = floor(taps / 2) + mod(taps, 2);
    % 低速前置补偿器在每个 8Fs 输入之间有 64 个时钟，使用单 MAC
    ops_per_cycle = 1;
    groups = ceil(total_ops / ops_per_cycle);

    if ~s.busy
        if din_valid
            work_delay = shift_in(s.delay, din);
            s_next.calc_delay = work_delay;
            s_next.delay      = work_delay;

            start_sum = comp_group_sum(work_delay, q_coef, start_idx, taps, 0, ops_per_cycle);
            s_next.groups_total = uint8(groups);
            start_sum = wrap_signed_local(start_sum, cfg.cic_comp_acc_width);
            s_next.acc_reg = start_sum;

            if groups == 1
                s_next.busy       = false;
                s_next.group_idx  = uint8(0);
                total_shift = cfg.cic_comp_coef_frac + max( ...
                    cfg.halfband2_data_width - cfg.cic_comp_data_width, 0);
                s_next.dout       = saturate_signed(round_shift_signed( ...
                    start_sum, total_shift), cfg.cic_comp_data_width);
                s_next.dout_valid = true;
            else
                s_next.busy      = true;
                s_next.group_idx = uint8(1);
            end
        end
    else
        group_idx = double(s.group_idx);
        gsum = comp_group_sum(s.calc_delay, q_coef, start_idx, taps, group_idx, ops_per_cycle);
        acc_next = wrap_signed_local( ...
            s.acc_reg + gsum, cfg.cic_comp_acc_width);

        if double(s.group_idx) == double(s.groups_total) - 1
            s_next.busy       = false;
            s_next.group_idx  = uint8(0);
            s_next.acc_reg    = acc_next;
            total_shift = cfg.cic_comp_coef_frac + max( ...
                cfg.halfband2_data_width - cfg.cic_comp_data_width, 0);
            s_next.dout       = saturate_signed(round_shift_signed( ...
                acc_next, total_shift), cfg.cic_comp_data_width);
            s_next.dout_valid = true;
        else
            s_next.acc_reg   = acc_next;
            s_next.group_idx = s.group_idx + uint8(1);
        end
    end
end

function gsum = comp_group_sum(delay, coef, start_idx, taps, group_idx, ops_per_cycle)
    pair_ops = floor(taps / 2);
    gsum = int64(0);
    base_op = double(group_idx) * ops_per_cycle;

    for m = 0:(ops_per_cycle - 1)
        op = base_op + m;
        if op < pair_ops
            left  = start_idx + op;
            right = start_idx + taps - 1 - op;
            gsum = gsum + (int64(delay(left)) + int64(delay(right))) * int64(coef(left));
        elseif mod(taps, 2) ~= 0 && op == pair_ops
            center = start_idx + pair_ops;
            gsum = gsum + int64(delay(center)) * int64(coef(center));
        end
    end
end

% 顶层输出延时和节拍寄存器
function [top_next, produced_valid, produced_sample] = top_rtl_step( ...
    top, cic_dout, cic_dout_valid, slot_ce, ce0, ce1, ce2, cfg)
    top_next = top;
    produced_valid = false;
    produced_sample = int64(0);

    % 对应 audio_interp_core 的输出有效样本延时链
    top_next.pcm_out_valid = cic_dout_valid;
    if cic_dout_valid
        if cfg.output_align_delay == 0
            top_next.pcm_out = restore_output_scale(cic_dout, cfg);
        else
            top_next.pcm_out = restore_output_scale(top.out_delay(end), cfg);
            if numel(top.out_delay) > 1
                top_next.out_delay(2:end) = top.out_delay(1:end-1);
            end
            top_next.out_delay(1) = int64(cic_dout);
        end
    end

    if top_next.pcm_out_valid
        produced_valid = true;
        produced_sample = top_next.pcm_out;
    end

    % slot_cnt：512Fs -> 128Fs
    if top.slot_cnt == 0
        top_next.slot_cnt = uint8(3);
    else
        top_next.slot_cnt = top.slot_cnt - uint8(1);
    end

    % rate_cnt：只在 slot_ce 有效时推进
    if slot_ce
        if top.rate_cnt == uint8(127)
            top_next.rate_cnt = uint8(0);
        else
            top_next.rate_cnt = top.rate_cnt + uint8(1);
        end
    end

    % phase_ref：与 RTL 一样，使用 ce0/ce1/ce2 在同一上升沿翻转
    if ce0
        top_next.phase_ref(1) = ~top.phase_ref(1);
    end
    if ce1
        top_next.phase_ref(2) = ~top.phase_ref(2);
    end
    if ce2
        top_next.phase_ref(3) = ~top.phase_ref(3);
    end
end

function y = restore_output_scale(x, cfg)
% 将缩短后的内部整数样本恢复到 24 bit 外部接口的标度
    restore_shift = max(cfg.out_width - cfg.cic_data_width, 0);
    y = saturate_signed(int64(x) * int64(2^restore_shift), cfg.out_width);
end
