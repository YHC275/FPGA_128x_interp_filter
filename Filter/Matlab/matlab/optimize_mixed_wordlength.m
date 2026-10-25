function [q_stage0, q_halfband, q_cic_comp, cfg_opt, result] = ...
    optimize_mixed_wordlength(h_stage0_ref, h_halfband_ref, h_cic_comp_ref, cfg, out_dir)
% 搜索最小系数字长和最小数据字长
% 第一阶段系数字长搜索：
%    对 Stage0、Halfband 和 CIC 补偿 FIR 尝试不同的小数位 F，
%    计算所需系数总位宽 W 和安全累加器位宽 A；只有满足整链路通带、阻带指标的组合才进入资源评分
%  第二阶段数据字长搜索：
%    固定第一阶段选出的整数系数，在 RTL 等效定点模型中搜索各级输出样本位宽 D，并根据输出误差和资源评分选出方案
%  字长符号：
%    F系数二进制小数位 W系数有符号总位宽 
%    A乘加或 CIC 内部累加器位宽 D各级输出样本、延时线和 FIFO 的数据位宽

    if nargin < 5 || isempty(out_dir)
        this_dir = fileparts(mfilename('fullpath'));
        out_dir = fullfile(fileparts(this_dir), 'rtl_export');
    end

    h_stage0_ref   = h_stage0_ref(:);
    h_halfband_ref = h_halfband_ref(:);
    h_cic_comp_ref = h_cic_comp_ref(:);

    % frac_values：待测试的系数小数位 F
    % nfft：整链路频率响应搜索所使用的 FFT 点数
    frac_values = cfg.wordlength.frac_candidates(:).';
    nfft = cfg.wordlength.search_nfft;

    % 为三组 FIR（Stage0、Halfband 和 CIC 补偿 FIR）分别建立候选
    stage0_candidates = build_candidates( ...
        h_stage0_ref, 2.0, frac_values, cfg.in_width, 'Stage0');
    halfband_candidates = build_candidates( ...
        h_halfband_ref, 2.0, frac_values, cfg.in_width, 'Halfband');
    comp_candidates = build_candidates( ...
        h_cic_comp_ref, 1.0, frac_values, cfg.in_width, 'CIC compensation');

    f = (0:nfft/2).' / nfft * cfg.output_fs_hz;
    pass_sel = f >= cfg.spec.passband_hz(1) & f <= cfg.spec.passband_hz(2);
    stop_sel = f >= cfg.spec.stopband_start_hz;

    % 构造 CIC 的等效冲激响应并计算频响
    H_cic_full = fft(cic_impulse(cfg.cic_rate, cfg.cic_stages), nfft);
    H_cic = H_cic_full(1:nfft/2 + 1);

    for k = 1:numel(stage0_candidates)
        H = fft(upsample_coeff_local(stage0_candidates(k).h, 64), nfft);
        stage0_candidates(k).H = H(1:nfft/2 + 1);
    end

    for k = 1:numel(halfband_candidates)
        H32 = fft(upsample_coeff_local(halfband_candidates(k).h, 32), nfft);
        H16 = fft(upsample_coeff_local(halfband_candidates(k).h, 16), nfft);
        halfband_candidates(k).H = ...
            H32(1:nfft/2 + 1) .* H16(1:nfft/2 + 1);
    end

    for k = 1:numel(comp_candidates)
        % 补偿 FIR 位于 8Fs 低速侧，映射到最终 128Fs 时间轴时系数间隔为 16
        H = fft(upsample_coeff_local(comp_candidates(k).h, 16), nfft);
        comp_candidates(k).H = H(1:nfft/2 + 1);
    end

    max_rows = numel(stage0_candidates) * ...
        numel(halfband_candidates) * numel(comp_candidates);
    rows(max_rows, 1) = empty_row();
    row_count = 0;
    best_score = inf;
    best_idx = [];

    % 穷举三组 FIR 的 F/W/A 组合。
    % 先计算整链路频响，再按要求筛选，最后比较资源评分
    for i0 = 1:numel(stage0_candidates)
        for ih = 1:numel(halfband_candidates)
            H_front = stage0_candidates(i0).H .* ...
                halfband_candidates(ih).H .* H_cic;

            for ic = 1:numel(comp_candidates)
                H = H_front .* comp_candidates(ic).H;
                metrics = response_metrics(H, pass_sel, stop_sel);

                row_count = row_count + 1;
                score = resource_proxy(stage0_candidates(i0), ...
                    halfband_candidates(ih), comp_candidates(ic), cfg);
                accepted = metrics.pass_pm_db <= cfg.wordlength.passband_guard_db && ...
                    metrics.stop_attn_db >= cfg.wordlength.stopband_guard_db;

                rows(row_count) = make_row(stage0_candidates(i0), ...
                    halfband_candidates(ih), comp_candidates(ic), ...
                    metrics, score, accepted);

                % 保留资源评分最低者
                if accepted && score < best_score
                    best_score = score;
                    best_idx = [i0 ih ic];
                end
            end
        end
    end

    rows = rows(1:row_count);
    if isempty(best_idx)
        error('optimize_mixed_wordlength:noFeasibleCandidate', ...
            ['没有满足 %.3f dB 通带和 %.1f dB 阻带保护限值的混合字长候选方案'], ...
            cfg.wordlength.passband_guard_db, cfg.wordlength.stopband_guard_db);
    end

    % 取出第一阶段选中的三组系数格式及整数系数。
    c0 = stage0_candidates(best_idx(1));
    ch = halfband_candidates(best_idx(2));
    cc = comp_candidates(best_idx(3));

    q_stage0   = c0.q;
    q_halfband = ch.q;
    q_cic_comp = cc.q;

    % 将第一阶段选出的 F/W/A 写入优化配置
    cfg_opt = cfg;
    cfg_opt.stage0_coef_width   = c0.coef_width;
    cfg_opt.stage0_coef_frac    = c0.frac;
    cfg_opt.stage0_acc_width    = c0.acc_width;
    cfg_opt.halfband_coef_width = ch.coef_width;
    cfg_opt.halfband_coef_frac  = ch.frac;
    cfg_opt.halfband_acc_width  = ch.acc_width;
    cfg_opt.cic_comp_coef_width = cc.coef_width;
    cfg_opt.cic_comp_coef_frac  = cc.frac;
    cfg_opt.cic_comp_acc_width  = cc.acc_width;
    % CIC 的理论位增长约为 stages*log2(rate)。
    % 先根据当前补偿输出位宽得到初值；第二阶段会按候选位宽重新计算
    cfg_opt.cic_acc_width = cfg.cic_comp_data_width + ...
        ceil(cfg.cic_stages * log2(cfg.cic_rate)) + ...
        cfg.wordlength.cic_guard_bits;

    % 第二阶段：固定整数系数，在 RTL 等效模型中搜索级间数据位宽
    [cfg_opt, data_result] = search_data_wordlengths( ...
        q_stage0, q_halfband, q_cic_comp, cfg_opt, out_dir);

    selected_row = rows(find_best_row(rows, best_score));
    % 汇总第一、第二阶段结果
    result = struct();
    result.method = 'spec_constrained_joint_data_coefficient_accumulator_wordlength';
    result.score = best_score + data_result.score;
    result.stage0 = rmfield(c0, 'H');
    result.halfband = rmfield(ch, 'H');
    result.cic_comp = rmfield(cc, 'H');
    result.cic_acc_width = cfg_opt.cic_acc_width;
    result.search_count = row_count;
    result.feasible_count = sum([rows.accepted]);
    result.pass_pm_db = selected_row.pass_pm_db;
    result.stop_attn_db = selected_row.stop_attn_db;
    result.passband_guard_db = cfg.wordlength.passband_guard_db;
    result.stopband_guard_db = cfg.wordlength.stopband_guard_db;
    result.data_search = data_result;

    % 导出候选表、文字报告、MATLAB 配置、 Verilog 头文件
    write_search_csv(fullfile(out_dir, 'mixed_wordlength_candidates.csv'), rows);
    write_summary(fullfile(out_dir, 'mixed_wordlength_report.txt'), cfg, cfg_opt, result);
    write_matlab_config(fullfile(fileparts(mfilename('fullpath')), ...
        'wordlength_config.m'), cfg_opt);
    write_rtl_header(cfg_opt);

    fprintf(['混合字长搜寻: %d 个候选, %d 个可行, ' ...
        '选定得分 %.0f.\n'], ...
        result.search_count, result.feasible_count, result.score);
    fprintf(['Stage0 F%d/W%d/A%d/D%d; Halfband F%d/W%d/A%d/D%d->%d; ' ...
        'CIC A%d/D%d; Comp F%d/W%d/A%d/D%d.\n'], ...
        cfg_opt.stage0_coef_frac, cfg_opt.stage0_coef_width, ...
        cfg_opt.stage0_acc_width, cfg_opt.stage0_data_width, ...
        cfg_opt.halfband_coef_frac, cfg_opt.halfband_coef_width, ...
        cfg_opt.halfband_acc_width, cfg_opt.halfband1_data_width, ...
        cfg_opt.halfband2_data_width, cfg_opt.cic_acc_width, ...
        cfg_opt.cic_data_width, ...
        cfg_opt.cic_comp_coef_frac, cfg_opt.cic_comp_coef_width, ...
        cfg_opt.cic_comp_acc_width, cfg_opt.cic_comp_data_width);
end

function [cfg_best, result] = search_data_wordlengths(q0, qh, qc, cfg, out_dir)
%  搜索级间数据安全较小位宽
%  Stage0(w0) -> Halfband1(w1) -> Halfband2(w2)-> CIC补偿FIR(wp) -> CIC(wc)
%  约束 w0 >= w1 >= w2 >= wp >= wc

    widths = unique(cfg.wordlength.data_width_candidates(:).');
    widths = sort(widths, 'descend');

    % 定点参考
    cfg_ref = cfg;
    cfg_ref.stage0_data_width    = cfg.out_width;
    cfg_ref.halfband1_data_width = cfg.out_width;
    cfg_ref.halfband2_data_width = cfg.out_width;
    cfg_ref.cic_data_width       = cfg.out_width;
    cfg_ref.cic_comp_data_width  = cfg.out_width;
    cfg_ref.stage0_acc_width     = max(48, cfg.stage0_acc_width);
    cfg_ref.halfband_acc_width   = max(48, cfg.halfband_acc_width);
    cfg_ref.cic_acc_width        = max(48, cfg.cic_acc_width);
    cfg_ref.cic_comp_acc_width   = max(48, cfg.cic_comp_acc_width);

    % 测试序列32个输入样本：最大正值、最小负值、随机数
    max_v = int64(2^(cfg.in_width-1)-1);
    min_v = int64(-2^(cfg.in_width-1));
    rng(19);
    x_test = [int64(0); max_v; min_v; max_v; min_v; ...
        int64(randi([-2^(cfg.in_width-1), 2^(cfg.in_width-1)-1], 27, 1))];
  
    y_ref = run_rtl_fixed(x_test, q0, qh, qc, cfg_ref);

    rows = repmat(empty_data_row(), numel(widths)^5, 1);
    count = 0;
    best_score = inf;
    best_cfg = [];
    best_metrics = struct();

    % 穷举满足实际级联顺序的单调不增数据位宽组合并写入配置
   for w0 = widths
    for w1 = widths(widths <= w0)
        for w2 = widths(widths <= w1)
            for wp = widths(widths <= w2)
                for wc = widths(widths <= wp)

                    cfg_try = cfg;

                    cfg_try.stage0_data_width     = w0;
                    cfg_try.halfband1_data_width  = w1;
                    cfg_try.halfband2_data_width  = w2;
                    cfg_try.cic_comp_data_width   = wp;
                    cfg_try.cic_data_width        = wc;

                        % 根据当前级输入位宽及整数系数的 L1 上界重新计算各 FIR 所需的最小位宽
                        cfg_try.stage0_acc_width = required_fir_acc_width(q0, cfg.in_width);
                        cfg_try.halfband_acc_width = required_fir_acc_width(qh, w0);
               
                        cfg_try.cic_acc_width = wp + ...
                            ceil(cfg.cic_stages * log2(cfg.cic_rate)) + ...
                            cfg.wordlength.cic_guard_bits;
                        cfg_try.cic_comp_acc_width = required_fir_acc_width(qc, w2);

                        % 使用与 RTL 一致的定点回绕、舍入和饱和规则运行当前候选
                        y_try = run_rtl_fixed(x_test, q0, qh, qc, cfg_try);
                        % 当前候选相对于高精度定点参考的逐样点误差
                        err = double(y_try) - double(y_ref);
                        if all(err == 0)
                            snr_db = inf;
                        else
                            snr_db = 20*log10(max(norm(double(y_ref)), 1) / norm(err));
                        end
                        % 衡量误差
                        err_rms = sqrt(mean(err.^2));
                        if err_rms == 0
                            error_snr_dbfs = inf;
                        else
                            error_snr_dbfs = 20*log10( ...
                                (2^(cfg.out_width-1)-1) / err_rms);
                        end
                      
                        max_abs_error = max(abs(err));
                        accepted = error_snr_dbfs >= ...
                            cfg.wordlength.screen_min_error_snr_dbfs && ...
                            max_abs_error <= ...
                            cfg.wordlength.screen_max_abs_error_lsb;
                        % 对误差合格方案进行资源评分
                        score = data_resource_proxy(cfg_try);

                        count = count + 1;
                        rows(count) = make_data_row(cfg_try, snr_db, ...
                            error_snr_dbfs, ...
                            max_abs_error, score, accepted);
                        if accepted && score < best_score
                            best_score = score;
                            best_cfg = cfg_try;
                            best_metrics.snr_db = snr_db;
                            best_metrics.error_snr_dbfs = error_snr_dbfs;
                            best_metrics.max_abs_error_lsb = max_abs_error;
                        end
                    end
                end
            end
        end
    end

    rows = rows(1:count);
    if isempty(best_cfg)
        error('optimize_mixed_wordlength:noFeasibleDataWidth', ...
            ['没有数据字长候选方案能满足误差信噪比 %.1f dBFS 和最大误差 %.0f LSB 的约束条件'], ...
            cfg.wordlength.screen_min_error_snr_dbfs, ...
            cfg.wordlength.screen_max_abs_error_lsb);
    end

    cfg_best = best_cfg;
    result = best_metrics;
    result.score = best_score;
    result.search_count = count;
    result.feasible_count = sum([rows.accepted]);
    result.min_error_snr_dbfs = ...
        cfg.wordlength.screen_min_error_snr_dbfs;
    result.max_error_limit_lsb = ...
        cfg.wordlength.screen_max_abs_error_lsb;
    result.stage0_data_width = cfg_best.stage0_data_width;
    result.halfband1_data_width = cfg_best.halfband1_data_width;
    result.halfband2_data_width = cfg_best.halfband2_data_width;
    result.cic_data_width = cfg_best.cic_data_width;
    result.cic_comp_data_width = cfg_best.cic_comp_data_width;
    write_data_search_csv(fullfile(out_dir, ...
        'mixed_data_wordlength_candidates.csv'), rows);
end

function score = data_resource_proxy(cfg)
% 第二阶段数据资源评分
% 分数由三类构成：
%   selector_bits   ：共享 MAC 为不同抽头选取样本时涉及的宽数据选择量
%   storage_bits    ：主要级间 FIFO 和输出对齐延时线的数据存储容量
%   arithmetic_bits ：FIR 累加器、CIC 梳状器/积分器及相关宽运算通路
%
% 最终权重 4、1、16 是经验权重，用于在 MATLAB 中快速排序 score = 4*selector + 1*storage + 16*arithmetic。

    %  抽头选择器评分
    % 近似按“抽头数 × 进入共享 MAC 的数据位宽”统计
    % 两级 Halfband 分别使用 Stage0 和 Halfband1 的输出作为输入
    % 补偿 FIR 项保持当前原公式，仅作为候选排序近似
    selector_bits = ...
        (cfg.stage0_ph0_taps + cfg.stage0_ph1_taps) * cfg.in_width + ...
        (cfg.hb_ph0_taps + cfg.hb_ph1_taps) * ...
            (cfg.stage0_data_width + cfg.halfband1_data_width) + ...
        cfg.cic_comp_taps * cfg.cic_data_width;

    % 系统存储评分
    % 前一项：两个主要级间 FIFO 的深度 × 数据宽度
    % 后一项：输出对齐延时深度 × 当前模型采用的延时数据宽度
    % 此处只统计主要数据位，不统计读写指针、满空标志和控制寄存器
    storage_bits = cfg.fifo_depth * ...
        (cfg.stage0_data_width + cfg.halfband1_data_width) + ...
        cfg.output_align_delay * cfg.cic_comp_data_width;

    %  算术通路评分
    %   Stage0 权重 1：一套 Stage0 共享 MAC
    %   Halfband 权重 2：系统中有两级 Halfband
    %   CIC 权重 6：3 级梳状器 + 3 级积分器
    %   补偿 FIR 权重 6：对其共享 MAC、宽加法、舍入等逻辑的经验折算
    arithmetic_bits = cfg.stage0_acc_width + ...
        2*cfg.halfband_acc_width + 6*cfg.cic_acc_width + ...
        6*cfg.cic_comp_acc_width;

    % 算术位通常比普通存储位更可能占用 LUT/进位链/DSP 相关资源，因而赋予更高经验权重
    score = 4*selector_bits + storage_bits + 16*arithmetic_bits;
end

function row = empty_data_row()
% 创建一条空的数据位宽候选记录，便于预分配结构体数组
    row = struct('stage0_data_width', 0, 'halfband1_data_width', 0, ...
        'halfband2_data_width', 0, 'cic_data_width', 0, ...
        'cic_comp_data_width', 0, 'stage0_acc_width', 0, ...
        'halfband_acc_width', 0, 'cic_acc_width', 0, ...
        'cic_comp_acc_width', 0, 'snr_db', 0, ...
        'error_snr_dbfs', 0, ...
        'max_abs_error_lsb', 0, 'score', 0, 'accepted', false);
end

function row = make_data_row(cfg, snr_db, error_snr_dbfs, max_abs_error, score, accepted)
% 将一个数据位宽候选的配置、误差指标和评分整理为结构体记录
    row = empty_data_row();
    row.stage0_data_width = cfg.stage0_data_width;
    row.halfband1_data_width = cfg.halfband1_data_width;
    row.halfband2_data_width = cfg.halfband2_data_width;
    row.cic_data_width = cfg.cic_data_width;
    row.cic_comp_data_width = cfg.cic_comp_data_width;
    row.stage0_acc_width = cfg.stage0_acc_width;
    row.halfband_acc_width = cfg.halfband_acc_width;
    row.cic_acc_width = cfg.cic_acc_width;
    row.cic_comp_acc_width = cfg.cic_comp_acc_width;
    row.snr_db = snr_db;
    row.error_snr_dbfs = error_snr_dbfs;
    row.max_abs_error_lsb = max_abs_error;
    row.score = score;
    row.accepted = accepted;
end

function write_data_search_csv(file_name, rows)
% 导出第二阶段全部数据位宽候选
    ensure_parent(file_name);
    fid = fopen(file_name, 'w');
    if fid < 0
        error('optimize_mixed_wordlength:dataCsvOpen', 'Cannot write %s.', file_name);
    end
    cleaner = onCleanup(@() fclose(fid)); 
    fprintf(fid, ['stage0_data_width,halfband1_data_width,' ...
        'halfband2_data_width,cic_data_width,cic_comp_data_width,' ...
        'stage0_acc_width,halfband_acc_width,cic_acc_width,' ...
        'cic_comp_acc_width,snr_db,error_snr_dbfs,' ...
        'max_abs_error_lsb,score,accepted\n']);
    for k = 1:numel(rows)
        r = rows(k);
        fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%d,%d,%.12g,%.12g,%.0f,%.0f,%d\n', ...
            r.stage0_data_width, r.halfband1_data_width, ...
            r.halfband2_data_width, r.cic_data_width, ...
            r.cic_comp_data_width, r.stage0_acc_width, ...
            r.halfband_acc_width, r.cic_acc_width, ...
            r.cic_comp_acc_width, r.snr_db, r.error_snr_dbfs, ...
            r.max_abs_error_lsb, ...
            r.score, r.accepted);
    end
end

function candidates = build_candidates(h_ref, dc_gain, frac_values, in_width, name)
% 为一组浮点 FIR 系数建立不同小数位 F 的定点候选。
% 每个候选保存：
%   frac       ：系数小数位 F
%   coef_width ：能够容纳全部整数系数 q 的最小有符号位宽 W
%   acc_width  ：按满幅输入和系数 L1 范数估算的安全累加器位宽 A
%   q          ：实际导出给 RTL 的整数系数
%   h          ：q/2^F 对应的量化后浮点系数
    candidates = repmat(struct( ...
        'name', name, 'frac', 0, 'coef_width', 0, 'acc_width', 0, ...
        'q', int64([]), 'h', [], 'H', []), numel(frac_values), 1);

    for k = 1:numel(frac_values)
        frac = frac_values(k);
        q = quantize_symmetric_dc_locked(h_ref, frac, dc_gain);
        candidates(k).frac = frac;
        candidates(k).coef_width = required_signed_width(q);
        candidates(k).acc_width = required_fir_acc_width(q, in_width);
        candidates(k).q = q;
        candidates(k).h = double(q) / (2 ^ frac);
    end
end

function q = quantize_symmetric_dc_locked(h, frac, dc_gain)
% 对浮点系数进行严格对称的定点量化

    scale = 2 ^ frac;
    q = int64(round(h(:) * scale));
    n = numel(q);

    for k = 1:floor(n / 2)
        v = int64(round((double(q(k)) + double(q(n + 1 - k))) / 2));
        q(k) = v;
        q(n + 1 - k) = v;
    end

    if mod(n, 2) == 0
        error('optimize_mixed_wordlength:evenLength', ...
            'DC锁定量化需要奇数个采样点，得到 %d', n);
    end

    target_sum = int64(round(dc_gain * scale));
    center = (n + 1) / 2;
    q(center) = q(center) + (target_sum - sum(q));

    if any(q ~= flipud(q))
        error('optimize_mixed_wordlength:symmetry', ...
            '整数系数对称性未被保留');
    end
end

function width = required_signed_width(values)
% 返回能够容纳全部整数值的最小二进制补码有符号位宽
    values = int64(values(:));
    for width = 2:62
        min_v = -2^(width - 1);
        max_v =  2^(width - 1) - 1;
        if all(double(values) >= min_v) && all(double(values) <= max_v)
            return;
        end
    end
    error('optimize_mixed_wordlength:coefWidth', ...
        '系数超出支持的 int64 搜索范围');
end

function width = required_fir_acc_width(q, in_width)
% 按满幅输入估计 FIR 的安全累加器位宽
% 使用保守的 L1 最坏情况上界：
%   max_abs_sum = 2^(in_width-1) * sum(abs(q))。
% 最终额外保留符号位
    max_abs_input = 2^(in_width - 1);
    max_abs_sum = max_abs_input * sum(abs(double(q)));
    width = 1 + ceil(log2(max_abs_sum + 1));
end

function score = resource_proxy(c0, ch, cc, cfg)
% 第一阶段的系数和累加器资源代理评分
% 分数由两类构成：
%   coef_bits     ：全部 FIR 系数按抽头数和系数位宽估计的存储量
%   datapath_bits ：各 FIR 宽累加数据通路的经验代价

    coef_bits = ...
        (cfg.stage0_ph0_taps + cfg.stage0_ph1_taps) * c0.coef_width + ...
        2 * (cfg.hb_ph0_taps + cfg.hb_ph1_taps) * ch.coef_width + ...
        cfg.cic_comp_taps * cc.coef_width;

    datapath_bits = ...
        6 * c0.acc_width + ...
        12 * ch.acc_width + ...
        14 * cc.acc_width;

    score = coef_bits + 8 * datapath_bits;
end

function metrics = response_metrics(H, pass_sel, stop_sel)
% 对总频响先按直流增益归一化
    dc = abs(H(1));
    Hn = H / max(dc, 1e-15);
    mag = abs(Hn);
    pass_db = 20 * log10(max(mag(pass_sel), 1e-14));
    metrics.pass_pm_db = max(abs(pass_db));
    metrics.stop_attn_db = -20 * log10(max(mag(stop_sel)));
end

function h = cic_impulse(rate, stages)
% 通过重复卷积长度为 rate 的全1序列，构造多级 CIC 等效冲激响应
    h = 1;
    for k = 1:stages
        h = conv(h, ones(rate, 1));
    end
    h = h(:) / (rate ^ (stages - 1));
end

function y = upsample_coeff_local(h, factor)
% 在相邻系数之间插入 factor-1 个零，将低速级映射到最终输出时间轴
    h = h(:);
    y = zeros((numel(h) - 1) * factor + 1, 1);
    y(1:factor:end) = h;
end

function row = empty_row()
% 创建一条空的系数字长候选记录
    row = struct( ...
        'stage0_frac', 0, 'stage0_coef_width', 0, 'stage0_acc_width', 0, ...
        'halfband_frac', 0, 'halfband_coef_width', 0, 'halfband_acc_width', 0, ...
        'comp_frac', 0, 'comp_coef_width', 0, 'comp_acc_width', 0, ...
        'pass_pm_db', 0, 'stop_attn_db', 0, 'score', 0, 'accepted', false);
end

function row = make_row(c0, ch, cc, metrics, score, accepted)
% 将一组系数字长候选及其频响、评分整理为结构体记录
    row = empty_row();
    row.stage0_frac = c0.frac;
    row.stage0_coef_width = c0.coef_width;
    row.stage0_acc_width = c0.acc_width;
    row.halfband_frac = ch.frac;
    row.halfband_coef_width = ch.coef_width;
    row.halfband_acc_width = ch.acc_width;
    row.comp_frac = cc.frac;
    row.comp_coef_width = cc.coef_width;
    row.comp_acc_width = cc.acc_width;
    row.pass_pm_db = metrics.pass_pm_db;
    row.stop_attn_db = metrics.stop_attn_db;
    row.score = score;
    row.accepted = accepted;
end

function idx = find_best_row(rows, score)
% 返回第一个“已通过指标且评分等于最优值”的候选下标
    idx = find([rows.accepted] & [rows.score] == score, 1, 'first');
end

function write_search_csv(file_name, rows)
% 导出第一阶段全部系数字长候选、频响指标、评分及是否合格
    ensure_parent(file_name);
    fid = fopen(file_name, 'w');
    if fid < 0
        error('optimize_mixed_wordlength:csvOpen', ...
            'Cannot write %s.', file_name);
    end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, ['stage0_frac,stage0_coef_width,stage0_acc_width,' ...
        'halfband_frac,halfband_coef_width,halfband_acc_width,' ...
        'comp_frac,comp_coef_width,comp_acc_width,' ...
        'pass_pm_db,stop_attn_db,score,accepted\n']);
    for k = 1:numel(rows)
        r = rows(k);
        fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%d,%d,%.12g,%.12g,%.0f,%d\n', ...
            r.stage0_frac, r.stage0_coef_width, r.stage0_acc_width, ...
            r.halfband_frac, r.halfband_coef_width, r.halfband_acc_width, ...
            r.comp_frac, r.comp_coef_width, r.comp_acc_width, ...
            r.pass_pm_db, r.stop_attn_db, r.score, r.accepted);
    end
end

function write_summary(file_name, cfg_before, cfg_after, result)
% 生成总报告
    ensure_parent(file_name);
    fid = fopen(file_name, 'w');
    if fid < 0
        error('optimize_mixed_wordlength:reportOpen', ...
            '无法写入 %s.', file_name);
    end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, '滤波器系数字长优化\n');
    fprintf(fid, '系数字长搜索组合: %d\n', result.search_count);
    fprintf(fid, '系数字长可行组合: %d\n', result.feasible_count);
    fprintf(fid, '通带纹波: %.9f dB\n', result.pass_pm_db);
    fprintf(fid, '阻带衰减: %.9f dB\n\n', result.stop_attn_db);
    fprintf(fid, '数据位宽优化\n');
    fprintf(fid, '数据位宽搜索组合: %d\n', result.data_search.search_count);
    fprintf(fid, '数据位宽可行组合: %d\n', result.data_search.feasible_count);
    fprintf(fid, '输出的 RMS 误差信噪比: %.9f dBFS\n', ...
        result.data_search.error_snr_dbfs);
    fprintf(fid, '最大误差: %.0f LSB\n\n', ...
        result.data_search.max_abs_error_lsb);

    if isfield(cfg_before, 'wordlength') && ...
            isfield(cfg_before.wordlength, 'baseline')
        cfg_report = cfg_before.wordlength.baseline;
    else
        cfg_report = cfg_before;
    end

    fprintf(fid, '模块，优化前系数位宽，优化前系数小数位宽，优化前累加器位宽，优化后系数位宽，优化后系数小数位宽，优化后累加器位宽\n');
    fprintf(fid, 'Stage0 , %d , %d , %d , %d , %d , %d\n', ...
        cfg_report.stage0_coef_width, cfg_report.stage0_coef_frac, ...
        cfg_report.stage0_acc_width, cfg_after.stage0_coef_width, ...
        cfg_after.stage0_coef_frac, cfg_after.stage0_acc_width);
    fprintf(fid, 'Halfband , %d , %d , %d , %d , %d , %d\n', ...
        cfg_report.halfband_coef_width, cfg_report.halfband_coef_frac, ...
        cfg_report.halfband_acc_width, cfg_after.halfband_coef_width, ...
        cfg_after.halfband_coef_frac, cfg_after.halfband_acc_width);
    fprintf(fid, 'CIC , - , - , %d , - , - , %d\n', ...
        cfg_report.cic_acc_width, cfg_after.cic_acc_width);
    fprintf(fid, 'CIC compensation , %d , %d , %d , %d , %d , %d\n', ...
        cfg_report.cic_comp_coef_width, cfg_report.cic_comp_coef_frac, ...
        cfg_report.cic_comp_acc_width, cfg_after.cic_comp_coef_width, ...
        cfg_after.cic_comp_coef_frac, cfg_after.cic_comp_acc_width);
    fprintf(fid, '\n优化前级间数据位宽，优化后级间数据位宽\n');
    fprintf(fid, 'Stage0 output , %d , %d\n', ...
        cfg_report.stage0_data_width, cfg_after.stage0_data_width);
    fprintf(fid, 'Halfband1 output , %d , %d\n', ...
        cfg_report.halfband1_data_width, cfg_after.halfband1_data_width);
    fprintf(fid, 'Halfband2 output , %d , %d\n', ...
        cfg_report.halfband2_data_width, cfg_after.halfband2_data_width);
    fprintf(fid, 'CIC output , %d , %d\n', ...
        cfg_report.cic_data_width, cfg_after.cic_data_width);
    fprintf(fid, 'CIC compensation output , %d , %d\n', ...
        cfg_report.cic_comp_data_width, cfg_after.cic_comp_data_width);
end

function write_matlab_config(file_name, cfg)
% 输出优化结果
    fid = fopen(file_name, 'w');
    if fid < 0
        error('optimize_mixed_wordlength:matlabConfigOpen', ...
            'Cannot write %s.', file_name);
    end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, 'function wl = wordlength_config()\n');
    fprintf(fid, '%%wordlength_config 自动生成的混合字长配置\n\n');
    fprintf(fid, '    wl.stage0_coef_width   = %d;\n', cfg.stage0_coef_width);
    fprintf(fid, '    wl.stage0_coef_frac    = %d;\n', cfg.stage0_coef_frac);
    fprintf(fid, '    wl.stage0_acc_width    = %d;\n\n', cfg.stage0_acc_width);
    fprintf(fid, '    wl.halfband_coef_width = %d;\n', cfg.halfband_coef_width);
    fprintf(fid, '    wl.halfband_coef_frac  = %d;\n', cfg.halfband_coef_frac);
    fprintf(fid, '    wl.halfband_acc_width  = %d;\n\n', cfg.halfband_acc_width);
    fprintf(fid, '    wl.cic_acc_width       = %d;\n\n', cfg.cic_acc_width);
    fprintf(fid, '    wl.cic_comp_coef_width = %d;\n', cfg.cic_comp_coef_width);
    fprintf(fid, '    wl.cic_comp_coef_frac  = %d;\n', cfg.cic_comp_coef_frac);
    fprintf(fid, '    wl.cic_comp_acc_width  = %d;\n', cfg.cic_comp_acc_width);
    fprintf(fid, '\n    %% 各级输出样本存储位宽；外部接口保持 24 bit\n');
    fprintf(fid, '    wl.stage0_data_width    = %d;\n', cfg.stage0_data_width);
    fprintf(fid, '    wl.halfband1_data_width = %d;\n', cfg.halfband1_data_width);
    fprintf(fid, '    wl.halfband2_data_width = %d;\n', cfg.halfband2_data_width);
    fprintf(fid, '    wl.cic_data_width       = %d;\n', cfg.cic_data_width);
    fprintf(fid, '    wl.cic_comp_data_width  = %d;\n', cfg.cic_comp_data_width);
    fprintf(fid, 'end\n');
end

function write_rtl_header(cfg)
% 将最终字长写入 Verilog 头文件
    matlab_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(fileparts(matlab_dir));
    rtl_dir = fullfile(project_dir, 'Vivado', 'Filter', ...
        'Filter.srcs', 'sources_1', 'new');
    file_name = fullfile(rtl_dir, 'mixed_wordlength_params.vh');

    fid = fopen(file_name, 'w');
    if fid < 0
        error('optimize_mixed_wordlength:rtlHeaderOpen', ...
            'Cannot write %s.', file_name);
    end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, '`ifndef MIXED_WORDLENGTH_PARAMS_VH\n');
    fprintf(fid, '`define MIXED_WORDLENGTH_PARAMS_VH\n');
    fprintf(fid, '`define WL_STAGE0_COEF_W %d\n', cfg.stage0_coef_width);
    fprintf(fid, '`define WL_STAGE0_COEF_FRAC %d\n', cfg.stage0_coef_frac);
    fprintf(fid, '`define WL_STAGE0_ACC_W %d\n', cfg.stage0_acc_width);
    fprintf(fid, '`define WL_HALFBAND_COEF_W %d\n', cfg.halfband_coef_width);
    fprintf(fid, '`define WL_HALFBAND_COEF_FRAC %d\n', cfg.halfband_coef_frac);
    fprintf(fid, '`define WL_HALFBAND_ACC_W %d\n', cfg.halfband_acc_width);
    fprintf(fid, '`define WL_CIC_ACC_W %d\n', cfg.cic_acc_width);
    fprintf(fid, '`define WL_CIC_COMP_COEF_W %d\n', cfg.cic_comp_coef_width);
    fprintf(fid, '`define WL_CIC_COMP_COEF_FRAC %d\n', cfg.cic_comp_coef_frac);
    fprintf(fid, '`define WL_CIC_COMP_ACC_W %d\n', cfg.cic_comp_acc_width);
    fprintf(fid, '`define WL_STAGE0_DATA_W %d\n', cfg.stage0_data_width);
    fprintf(fid, '`define WL_HALFBAND1_DATA_W %d\n', cfg.halfband1_data_width);
    fprintf(fid, '`define WL_HALFBAND2_DATA_W %d\n', cfg.halfband2_data_width);
    fprintf(fid, '`define WL_CIC_DATA_W %d\n', cfg.cic_data_width);
    fprintf(fid, '`define WL_CIC_COMP_DATA_W %d\n', cfg.cic_comp_data_width);
    fprintf(fid, '`endif\n');
end

function ensure_parent(file_name)
    parent = fileparts(file_name);
    if ~exist(parent, 'dir')
        mkdir(parent);
    end
end
