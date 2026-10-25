function [h_stage0, h_halfband, h_cic_comp, q_stage0, q_halfband, q_cic_comp, cfg, design_info] = design_all_filters(out_dir, update_coef_mem, update_load_coeffs_m, optimize_wordlength)
% 重新设计滤波器系数

    if nargin < 1 || isempty(out_dir)
        this_dir = fileparts(mfilename('fullpath'));
        out_dir = fullfile(fileparts(this_dir), 'rtl_export');
    end

    if nargin < 2 || isempty(update_coef_mem)
        update_coef_mem = true;
    end

    if nargin < 3 || isempty(update_load_coeffs_m)
        update_load_coeffs_m = true;
    end

    if nargin < 4 || isempty(optimize_wordlength)
        optimize_wordlength = false;
    end

    cfg = filter_config();

    design_info = struct();
    design_info.update_coef_mem = logical(update_coef_mem);
    design_info.update_load_coeffs_m = logical(update_load_coeffs_m);
    design_info.out_dir = out_dir;

    % 使用当前配置重新设计全部 FIR，任何一级不满足整链路指标时立即停止
    % 禁止回退使用预置系数
    [h_stage0_ref, h_halfband_ref, h_cic_comp_ref] = design_by_firpm(cfg);

    if optimize_wordlength 
    % 执行字长优化
        [q_stage0, q_halfband, q_cic_comp, cfg, wl_result] = ...
            optimize_mixed_wordlength(h_stage0_ref, h_halfband_ref, ...
                h_cic_comp_ref, cfg, out_dir);
        design_info.wordlength = wl_result;
        design_info.method = ...
            'firpm_plus_joint_data_coefficient_accumulator_wordlength_search';
    else
        q_stage0 = quantize_symmetric_q(h_stage0_ref, ...
            cfg.stage0_coef_frac, cfg.stage0_coef_width);
        q_halfband = quantize_symmetric_q(h_halfband_ref, ...
            cfg.halfband_coef_frac, cfg.halfband_coef_width);
        q_cic_comp = quantize_symmetric_q(h_cic_comp_ref, ...
            cfg.cic_comp_coef_frac, cfg.cic_comp_coef_width);
        design_info.method = 'firpm_configured_wordlength';
    end
    [ok, reason, resp_try] = verify_total_response( ...
        q_stage0, q_halfband, q_cic_comp, cfg);
    if ~ok
        error('design_all_filters:redesignCheckFailed', ...
            '重新设计的系数未通过整链路检查：%s', reason);
    end
    design_info.verify_reason = reason;
    design_info.response = resp_try;

    %  强制整数系数严格左右对称，保持严格线性相位
    q_stage0   = force_symmetric_int(q_stage0,   cfg.stage0_coef_width, 'Stage0');
    q_halfband = force_symmetric_int(q_halfband, cfg.halfband_coef_width, 'Halfband');
    q_cic_comp = force_symmetric_int(q_cic_comp, cfg.cic_comp_coef_width, 'CIC compensation');

    check_len(q_stage0,   cfg.stage0_ph0_taps + cfg.stage0_ph1_taps, 'Stage0');
    check_len(q_halfband, cfg.hb_ph0_taps     + cfg.hb_ph1_taps,     'Halfband');
    check_len(q_cic_comp, cfg.cic_comp_taps,                        'CIC compensation');

    h_stage0   = double(q_stage0)   / (2 ^ cfg.stage0_coef_frac);
    h_halfband = double(q_halfband) / (2 ^ cfg.halfband_coef_frac);
    h_cic_comp = double(q_cic_comp) / (2 ^ cfg.cic_comp_coef_frac);

    [ok, reason, resp] = verify_total_response(q_stage0, q_halfband, q_cic_comp, cfg);
    design_info.final_response = resp;
    design_info.final_verify_reason = reason;

    if ~ok
        error('design_all_filters:finalCheckFailed', ...
            '系数未通过检查：%s。请不要导出到 RTL。', reason);
    end

    % 更新 coefficient .mem 文件
    if update_coef_mem
        write_coef_mem_files(out_dir, q_stage0, q_halfband, q_cic_comp, cfg);
    end

    % 把新整数系数写回 load_coeffs.m
    if update_load_coeffs_m
        this_dir = fileparts(mfilename('fullpath'));
        loader_file = fullfile(this_dir, 'load_coeffs.m');
        write_load_coeffs_m(loader_file, q_stage0, q_halfband, q_cic_comp);
    end

    fprintf('等效总冲激响应对称误差 = %.3e\n', resp.symmetry_max_err);
    fprintf('通带最大偏差 = %.6f dB，阻带衰减 = %.6f dB\n', ...
        resp.pass_ripple_pm_db, resp.stop_attn_db);
end

% 子函数
% 重新设计 Stage0 / Halfband / CIC 补偿 FIR，并重新经过严格对称检查
function [h_stage0, h_hb, h_cic_comp] = design_by_firpm(cfg)
    if exist('firpm', 'file') == 0
        error('未检测到 firpm，无法执行等波纹 FIR 设计。');
    end

    % Stage0 是整个 Pro 版本的超窄过渡带主滤波器。它工作在 88.2 kHz
    % 输出域，直接按物理频率设计，在同一级内同时完成输入超声噪声清理
    % 和第一次 2x 插值抗镜像。后续两级仍使用宽过渡带半带型滤波器。
    % 所有 2x FIR 的 DC 增益均为 2，用于补偿插 0 带来的幅度缩放。
    stage0_taps = cfg.stage0_ph0_taps + cfg.stage0_ph1_taps;
    hb_taps     = cfg.hb_ph0_taps     + cfg.hb_ph1_taps;

    stage0_nyquist_hz = cfg.input_fs_hz;
    stage0_pass_edge = cfg.spec.passband_hz(2) / stage0_nyquist_hz;
    stage0_stop_edge = cfg.spec.design_stop_hz / stage0_nyquist_hz;

    if stage0_pass_edge <= 0 || stage0_stop_edge >= 1 || ...
            stage0_pass_edge >= stage0_stop_edge
        error('design_all_filters:invalidStage0Edges', ...
            'Stage0 频率边缘非法：pass=%.9f, stop=%.9f。', ...
            stage0_pass_edge, stage0_stop_edge);
    end

    h_stage0 = design_scaled_halfband_like_fir(stage0_taps, 2.0, ...
        stage0_pass_edge, stage0_stop_edge, cfg.spec.design_stop_weight);
    h_hb     = design_scaled_halfband_like_fir(hb_taps,     2.0, 0.45, 0.55, 15);

    % CIC 补偿 FIR 拟合归一化 CIC 幅频响应的倒数
    h_cic_comp = design_cic_comp_fir( ...
        cfg.cic_comp_taps, cfg.cic_rate, cfg.cic_stages);
end

function h = design_scaled_halfband_like_fir(num_taps, dc_gain, pass_edge, stop_edge, stop_weight)
    order = num_taps - 1;
    f = [0 pass_edge stop_edge 1];
    a = [dc_gain dc_gain 0 0];
    w = [1 stop_weight];

    h = firpm(order, f, a, w);
    h = h(:);

    % 浮点层面强制严格对称
    h = 0.5 * (h + flipud(h));

    % 重新归一化，保证 DC 增益精确接近 dc_gain
    h = h * (dc_gain / sum(h));

    % 再做一次对称化，避免归一化后产生数值微差
    h = 0.5 * (h + flipud(h));
end

function h = design_cic_comp_fir(num_taps, rate, stages)
% 在 CIC 输入低速域设计线性相位反下垂补偿 FIR
% 归一化频率 1 对应 8Fs（352.8 kHz）采样率的 Nyquist 频率

    order = num_taps - 1;
    pass_edge = 0.12;
    stop_edge = 0.50;
    bands = [0 pass_edge stop_edge 1];
    weights = [1 2];

    h = firpm(order, bands, ...
        {@cic_comp_fresp, rate, stages}, weights);
    h = h(:);
    h = 0.5 * (h + flipud(h));
    h = h / sum(h);
    h = 0.5 * (h + flipud(h));
end

function [desired, weight] = cic_comp_fresp(order, bands, grid, band_weights, rate, stages)
% 所需的 CIC 反下垂目标响应函数

    if ischar(order) || (isstring(order) && isscalar(order))
        desired = 'even';
        weight = [];
        return;
    end

    desired = zeros(size(grid));
    weight = zeros(size(grid));

    pass_sel = grid >= bands(1) & grid <= bands(2);
    % grid 是低速侧归一化频率。对应 CIC 高速侧角频率为 omega_high = omega_low / rate
    omega_low = pi * grid(pass_sel);
    cic_mag = ones(size(omega_low));
    nz = abs(omega_low) > eps;
    cic_mag(nz) = abs( ...
        sin(omega_low(nz) / 2) ./ ...
        (rate * sin(omega_low(nz) / (2 * rate)))).^stages;

    desired(pass_sel) = 1 ./ max(cic_mag, 1e-12);
    weight(pass_sel) = band_weights(1);

    stop_sel = grid >= bands(3) & grid <= bands(4);
    desired(stop_sel) = 0;
    weight(stop_sel) = band_weights(2);
end

function q = quantize_symmetric_q(h, frac_bits, coef_width)
    scale = 2 ^ frac_bits;
    q = int64(round(h(:) * scale));
    q = force_symmetric_int(q, coef_width, 'quantized FIR');
end

function q = force_symmetric_int(q, coef_width, name)
    q = int64(q(:));

    max_v = int64(2^(coef_width - 1) - 1);
    min_v = -int64(2^(coef_width - 1));

    if any(q > max_v) || any(q < min_v)
        error('design_all_filters:%s 系数量化后超过 %d bit 有符号范围。', name, coef_width);
    end

    n = numel(q);
    for k = 1:floor(n / 2)
        v = int64(round((double(q(k)) + double(q(n + 1 - k))) / 2));
        q(k) = v;
        q(n + 1 - k) = v;
    end

    if max(abs(q - flipud(q))) ~= 0
        error('design_all_filters:%s 强制对称失败。', name);
    end
end

% 总链路验证：检查每一级对称以及 128x 等效总冲激响应对称
function [ok, reason, resp] = verify_total_response(q_stage0, q_halfband, q_cic_comp, cfg)
    q_stage0   = force_symmetric_int(q_stage0,   cfg.stage0_coef_width, 'Stage0');
    q_halfband = force_symmetric_int(q_halfband, cfg.halfband_coef_width, 'Halfband');
    q_cic_comp = force_symmetric_int(q_cic_comp, cfg.cic_comp_coef_width, 'CIC compensation');

    h_stage0   = double(q_stage0)   / (2 ^ cfg.stage0_coef_frac);
    h_halfband = double(q_halfband) / (2 ^ cfg.halfband_coef_frac);
    h_cic_comp = double(q_cic_comp) / (2 ^ cfg.cic_comp_coef_frac);

    [~, h_eq] = run_float_reference(0, h_stage0, h_halfband, h_cic_comp, cfg);
    resp = analyze_response(h_eq, cfg);

    ok = true;
    reason = 'PASS';

    if ~resp.linear_phase_ok
        ok = false;
        reason = sprintf('等效总冲激响应不严格对称，max symmetry error = %.3e。', resp.symmetry_max_err);
        return;
    end

    if ~resp.passband_ok
        ok = false;
        reason = sprintf('通带偏差 %.6f dB 超过 %.6f dB。', ...
            resp.pass_ripple_pm_db, resp.spec.passband_pm_db);
        return;
    end

    if ~resp.stopband_ok
        ok = false;
        reason = sprintf('阻带衰减 %.6f dB 小于 %.6f dB。', ...
            resp.stop_attn_db, resp.spec.stopband_attn_db);
        return;
    end
end

% 写回原 coefficient .mem 文件名
function write_coef_mem_files(out_dir, q_stage0, q_halfband, q_cic_comp, cfg)
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    write_hex_mem(fullfile(out_dir, 'coef_stage0_even.mem'),   q_stage0(1:2:end),   cfg.stage0_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_stage0_odd.mem'),    q_stage0(2:2:end),   cfg.stage0_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_halfband_even.mem'), q_halfband(1:2:end), cfg.halfband_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_halfband_odd.mem'),  q_halfband(2:2:end), cfg.halfband_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_cic_comp.mem'),      q_cic_comp,          cfg.cic_comp_coef_width);

    sync_coef_mem_to_vivado(out_dir);
    fprintf('design_all_filters: 已覆盖 coefficient .mem 文件到：%s\n', out_dir);
end

function sync_coef_mem_to_vivado(out_dir)
    this_dir = fileparts(mfilename('fullpath'));
    filter_part_dir = fileparts(fileparts(this_dir));
    vivado_dir = fullfile(filter_part_dir, 'Vivado', 'Filter');

    sync_dirs = { ...
        fullfile(vivado_dir, 'Filter.ip_user_files', 'mem_init_files'), ...
        fullfile(vivado_dir, 'Filter.sim', 'sim_1', 'behav', 'xsim') ...
    };

    mem_files = { ...
        'coef_stage0_even.mem', ...
        'coef_stage0_odd.mem', ...
        'coef_halfband_even.mem', ...
        'coef_halfband_odd.mem', ...
        'coef_cic_comp.mem' ...
    };

    for d = 1:numel(sync_dirs)
        if ~exist(sync_dirs{d}, 'dir')
            continue;
        end

        for k = 1:numel(mem_files)
            src = fullfile(out_dir, mem_files{k});
            if exist(src, 'file')
                copyfile(src, fullfile(sync_dirs{d}, mem_files{k}));
            end
        end
    end
end

% 写回 load_coeffs.m
function write_load_coeffs_m(loader_file, q_stage0, q_halfband, q_cic_comp)
    [loader_dir, ~, ~] = fileparts(loader_file);
    if ~exist(loader_dir, 'dir')
        mkdir(loader_dir);
    end

    backup_file = fullfile(loader_dir, 'load_coeffs.m.bak');
    if exist(loader_file, 'file') && ~exist(backup_file, 'file')
        copyfile(loader_file, backup_file);
        fprintf('design_all_filters: 已备份原系数文件到：%s\n', backup_file);
    end

    fid = fopen(loader_file, 'w');
    if fid < 0
        error('design_all_filters: 无法写入系数文件：%s', loader_file);
    end
    cleaner = onCleanup(@() fclose(fid)); 

    fprintf(fid, 'function [h_stage0, h_halfband, h_cic_comp, q_stage0, q_halfband, q_cic_comp, cfg] = load_coeffs()\n');
    fprintf(fid, '%%LOAD_COEFFS 加载 44.1 kHz / 128x 插值滤波器系数。\n');
    fprintf(fid, '%% 此文件由 design_all_filters.m 自动更新。\n');
    fprintf(fid, '%% 系数格式由 filter_config/wordlength_config 分级给出。\n\n');
    fprintf(fid, '    cfg = filter_config();\n\n');

    write_q_vector(fid, 'q_stage0', q_stage0);
    write_q_vector(fid, 'q_halfband', q_halfband);
    write_q_vector(fid, 'q_cic_comp', q_cic_comp);

    fprintf(fid, '    check_len(q_stage0,   cfg.stage0_ph0_taps + cfg.stage0_ph1_taps, ''Stage0'');\n');
    fprintf(fid, '    check_len(q_halfband, cfg.hb_ph0_taps     + cfg.hb_ph1_taps,     ''Halfband'');\n');
    fprintf(fid, '    check_len(q_cic_comp, cfg.cic_comp_taps,                        ''CIC 补偿 FIR'');\n\n');

    fprintf(fid, '    if max(abs(q_stage0 - flipud(q_stage0))) ~= 0\n');
    fprintf(fid, '        error(''load_coeffs: Stage0 系数不是严格对称，不能保证严格线性相位。'');\n');
    fprintf(fid, '    end\n');
    fprintf(fid, '    if max(abs(q_halfband - flipud(q_halfband))) ~= 0\n');
    fprintf(fid, '        error(''load_coeffs: Halfband 系数不是严格对称，不能保证严格线性相位。'');\n');
    fprintf(fid, '    end\n');
    fprintf(fid, '    if max(abs(q_cic_comp - flipud(q_cic_comp))) ~= 0\n');
    fprintf(fid, '        error(''load_coeffs: CIC 补偿 FIR 系数不是严格对称，不能保证严格线性相位。'');\n');
    fprintf(fid, '    end\n\n');

    fprintf(fid, '    h_stage0   = double(q_stage0)   / (2 ^ cfg.stage0_coef_frac);\n');
    fprintf(fid, '    h_halfband = double(q_halfband) / (2 ^ cfg.halfband_coef_frac);\n');
    fprintf(fid, '    h_cic_comp = double(q_cic_comp) / (2 ^ cfg.cic_comp_coef_frac);\n');
    fprintf(fid, 'end\n\n');

    fprintf(fid, 'function check_len(x, expected_len, name)\n');
    fprintf(fid, '    if numel(x) ~= expected_len\n');
    fprintf(fid, '        error(''load_coeffs:%%s 长度错误，期望 %%d，实际 %%d。'', name, expected_len, numel(x));\n');
    fprintf(fid, '    end\n');
    fprintf(fid, 'end\n');

    fprintf('design_all_filters: 已更新 MATLAB 系数加载文件：%s\n', loader_file);
end

function write_q_vector(fid, name, q)
    q = int64(q(:)).';
    per_line = 8;

    fprintf(fid, '    %s = [ ...\n', name);
    for k = 1:numel(q)
        if mod(k - 1, per_line) == 0
            fprintf(fid, '    ');
        end

        fprintf(fid, '%d', q(k));

        if k < numel(q)
            fprintf(fid, '  ');
        end

        if mod(k, per_line) == 0 && k < numel(q)
            fprintf(fid, '...\n');
        end
    end
    fprintf(fid, ' ];\n');
    fprintf(fid, '    %s = int64(%s(:));\n\n', name, name);
end

function check_len(x, expected_len, name)
    if numel(x) ~= expected_len
        error('design_all_filters:%s 长度错误，期望 %d，实际 %d。', name, expected_len, numel(x));
    end
end
