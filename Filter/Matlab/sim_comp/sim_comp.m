% 仿真输出与 RTL 定点输出对比
clear;
clc;
close all;

%路径与参数 
P = get_filter_project_paths();

rtl_output_file = fullfile(P.xsim_dir, 'rtl_output_pcm.mem');

golden_mem_file = fullfile(P.rtl_export_dir, 'golden_out_128x.mem');
if ~exist(golden_mem_file, 'file')
    golden_mem_file = fullfile(P.xsim_dir, 'golden_out_128x.mem');
end

report_root = fullfile(P.matlab_dir, 'sim_comp', 'sim_comp_report');

OUT_W = 24;
OUTPUT_FS_HZ = 5644800;
FULL_SCALE = 2^(OUT_W - 1) - 1;

% offset 搜索范围，单位：输出样本点
SEARCH_OFFSET_RANGE = -4096:4096;

% 前段显示点数
SHOW_LEN = 8000;

% 全局误差抽样显示点数
GLOBAL_PLOT_POINTS = 30000;

% 是否导出逐点误差表
WRITE_POINT_ERROR_TABLE = true;
MAX_POINT_ERROR_ROWS = 300000;

% 创建报告目录
time_tag = datestr(now, 'yyyymmdd_HHMMSS');
report_dir = fullfile(report_root, ['aligned_error_', time_tag]);

if ~exist(report_dir, 'dir')
    mkdir(report_dir);
end

diary(fullfile(report_dir, 'matlab_console_log.txt'));

fprintf('============================================================\n');
fprintf('RTL 输出与 Golden 理想输出对齐误差分析\n');
fprintf('工程根目录：%s\n', P.project_root);
fprintf('============================================================\n\n');

fprintf('RTL 输出文件：%s\n', rtl_output_file);
fprintf('Golden 文件 ：%s\n', golden_mem_file);
fprintf('报告目录    ：%s\n\n', report_dir);

% 文件检查 
if ~exist(rtl_output_file, 'file')
    error('找不到 RTL 输出文件：%s\n请确认仿真文件已经生成 rtl_output_pcm.mem。', rtl_output_file);
end

if ~exist(golden_mem_file, 'file')
    error('找不到 Golden 文件：%s\n请确认 MATLAB 已导出 golden_out_128x.mem。', golden_mem_file);
end

% 按 24bit 有符号补码读取数据 
fprintf('---------------- 读取 24bit 有符号数据 ----------------\n');

rtl_dec = read_signed_hex_mem_valid(rtl_output_file, OUT_W);
golden_dec = read_signed_hex_mem_valid(golden_mem_file, OUT_W);

fprintf('RTL 有效样本数    = %d\n', numel(rtl_dec));
fprintf('Golden 有效样本数 = %d\n\n', numel(golden_dec));

if isempty(rtl_dec)
    error('RTL 输出文件没有读到有效十六进制数据。');
end

if isempty(golden_dec)
    error('Golden 文件没有读到有效十六进制数据。');
end

fprintf('RTL 数值范围    = [%d, %d]\n', min(rtl_dec), max(rtl_dec));
fprintf('Golden 数值范围 = [%d, %d]\n\n', min(golden_dec), max(golden_dec));

% 严格 offset 对齐 
fprintf('---------------- 严格 offset 对齐 ----------------\n');
fprintf('搜索范围：%d 到 %d 个输出样本\n', min(SEARCH_OFFSET_RANGE), max(SEARCH_OFFSET_RANGE));
fprintf('对每一个 offset 使用全部重叠数据计算 RMSE，选择 RMSE 最小者。\n\n');

[best_offset, offset_table] = find_best_offset_strict_rmse( ...
    rtl_dec, golden_dec, SEARCH_OFFSET_RANGE, FULL_SCALE);

offset_table_file = fullfile(report_dir, 'offset_search_table.csv');
writetable(offset_table, offset_table_file);

[rtl_cmp_i64, golden_cmp_i64, cmp_index, rtl_start, golden_start] = align_by_offset( ...
    rtl_dec, golden_dec, best_offset);

n_cmp = numel(rtl_cmp_i64);

if n_cmp <= 0
    error('对齐后没有可比较数据，请检查 offset 搜索范围。');
end

fprintf('最佳 offset          = %d\n', best_offset);
fprintf('RTL 起始位置         = %d\n', rtl_start - 1);
fprintf('Golden 起始位置      = %d\n', golden_start - 1);
fprintf('实际参与比较样本数   = %d\n', n_cmp);
fprintf('offset 搜索表        = %s\n\n', offset_table_file);

rtl_cmp = double(rtl_cmp_i64);
golden_cmp = double(golden_cmp_i64);

% 计算原始误差 
fprintf('---------------- 原始误差 RTL - Golden ----------------\n');

err = rtl_cmp - golden_cmp;
abs_err = abs(err);
err_norm = err / FULL_SCALE;

metrics = compute_error_metrics(rtl_cmp, golden_cmp, FULL_SCALE);
print_metrics(metrics, '原始误差');

%导出 summary.txt 
summary_file = fullfile(report_dir, 'summary.txt');

fid = fopen(summary_file, 'w');
if fid < 0
    error('无法写入 summary.txt');
end

fprintf(fid, 'RTL 有效样本数       = %d\n', numel(rtl_dec));
fprintf(fid, 'Golden 有效样本数    = %d\n', numel(golden_dec));
fprintf(fid, '最佳 offset          = %d\n', best_offset);
fprintf(fid, 'RTL 起始位置         = %d\n', rtl_start - 1);
fprintf(fid, 'Golden 起始位置      = %d\n', golden_start - 1);
fprintf(fid, '实际参与比较样本数   = %d\n\n', n_cmp);

write_metrics(fid, metrics, '原始误差');

fclose(fid);

fprintf('summary 已导出：%s\n\n', summary_file);

% 导出逐点误差表
if WRITE_POINT_ERROR_TABLE
    n_write = min(n_cmp, MAX_POINT_ERROR_ROWS);

    point_file = fullfile(report_dir, 'point_error_table.csv');

    T = table( ...
        double(cmp_index(1:n_write)), ...
        rtl_cmp(1:n_write), ...
        golden_cmp(1:n_write), ...
        err(1:n_write), ...
        abs_err(1:n_write), ...
        err_norm(1:n_write), ...
        'VariableNames', { ...
            'index', ...
            'rtl_dec', ...
            'golden_dec', ...
            'error_lsb', ...
            'abs_error_lsb', ...
            'error_norm'} ...
    );

    writetable(T, point_file);

    fprintf('逐点误差表已导出：%s\n', point_file);
    fprintf('导出行数：%d\n\n', n_write);
end

%  画图 
fprintf('---------------- 生成误差图 ----------------\n');

% 图 1：误差直方图
figure('Name', '误差直方图', 'NumberTitle', 'off');
histogram(err, 200);
grid on;
xlabel('误差 RTL - Golden / LSB');
ylabel('出现次数');
title('误差分布直方图');
saveas(gcf, fullfile(report_dir, '01_error_histogram.png'));

fprintf('图像已保存到：%s\n\n', report_dir);

fprintf('============================================================\n');
fprintf('完成。\n');
fprintf('报告目录：%s\n', report_dir);
fprintf('============================================================\n');

diary off;

%子函数
function P = get_filter_project_paths()
% 获取工程根目录、MATLAB目录、Vivado目录及xsim仿真目录的相对路径
    this_file = mfilename('fullpath');

    if isempty(this_file)
        this_dir = pwd;
    else
        this_dir = fileparts(this_file);
    end

    matlab_dir = fileparts(this_dir);
    project_root = fileparts(matlab_dir);

    P = struct();
    P.project_root = project_root;
    P.matlab_dir = matlab_dir;
    P.sim_comp_dir = this_dir;

    P.rtl_export_dir = fullfile(P.matlab_dir, 'rtl_export');

    P.vivado_dir = fullfile(P.project_root, 'Vivado', 'Filter');

    P.xsim_dir = fullfile(P.vivado_dir, ...
        'Filter.sim', 'sim_1', 'behav', 'xsim');
end

function data_dec = read_signed_hex_mem_valid(file_name, width)
% 读取mem文件中的有效十六进制数据，并转换为指定宽度的有符号十进制数据
    lines = readlines(file_name);
    lines = strip(lines);

    data_dec = zeros(0, 1, 'int64');
    max_hex_digits = ceil(width / 4);

    for i = 1:numel(lines)
        s = char(lines(i));
        s = strtrim(s);

        if isempty(s)
            continue;
        end

        if startsWith(s, '//') || startsWith(s, '%') || startsWith(s, '#')
            continue;
        end

        parts = regexp(s, '\s+', 'split');
        s = parts{1};

        if contains(s, ',')
            continue;
        end

        if length(s) > 2 && strcmpi(s(1:2), '0x')
            s = s(3:end);
        end

        if isempty(regexp(s, '^[0-9a-fA-F]+$', 'once'))
            continue;
        end

        if length(s) > max_hex_digits
            s = s(end-max_hex_digits+1:end);
        end

        u = uint32(hex2dec(s));
        v = int64(u);

        if v >= 2^(width - 1)
            v = v - 2^width;
        end

        data_dec(end+1, 1) = v; 
    end
end

function [best_offset, T] = find_best_offset_strict_rmse(rtl_dec, golden_dec, offset_range, full_scale)
% 在指定偏移范围内搜索 RTL 数据与 Golden 数据 RMSE 最小时的最佳样点偏移
    r = double(rtl_dec(:)) / full_scale;
    g = double(golden_dec(:)) / full_scale;

    M = numel(r);
    N = numel(g);

    pr  = [0; cumsum(r)];
    pg  = [0; cumsum(g)];
    pr2 = [0; cumsum(r.^2)];
    pg2 = [0; cumsum(g.^2)];

    conv_len = M + N - 1;
    fft_len = 2^nextpow2(conv_len);

    cross_all = real(ifft(fft(r, fft_len) .* fft(flipud(g), fft_len)));
    cross_all = cross_all(1:conv_len);

    offsets = offset_range(:);
    valid = false(numel(offsets), 1);
    overlap_n = zeros(numel(offsets), 1);
    rmse_lsb = inf(numel(offsets), 1);
    rmse_norm = inf(numel(offsets), 1);

    for k = 1:numel(offsets)
        d = offsets(k);

        if d >= 0
            rtl_start = 1;
            golden_start = 1 + d;
        else
            rtl_start = 1 - d;
            golden_start = 1;
        end

        n = min(M - rtl_start + 1, N - golden_start + 1);

        if n <= 0
            continue;
        end

        rtl_end = rtl_start + n - 1;
        golden_end = golden_start + n - 1;

        sum_r2 = pr2(rtl_end + 1) - pr2(rtl_start);
        sum_g2 = pg2(golden_end + 1) - pg2(golden_start);

        cross_idx = N - d;

        if cross_idx < 1 || cross_idx > numel(cross_all)
            continue;
        end

        cross = cross_all(cross_idx);
        sse = sum_r2 + sum_g2 - 2 * cross;

        if sse < 0 && abs(sse) < 1e-9
            sse = 0;
        end

        rmse = sqrt(max(sse / n, 0));

        valid(k) = true;
        overlap_n(k) = n;
        rmse_norm(k) = rmse;
        rmse_lsb(k) = rmse * full_scale;
    end

    T = table(offsets, valid, overlap_n, rmse_lsb, rmse_norm, ...
        'VariableNames', {'offset', 'valid', 'overlap_n', 'rmse_lsb', 'rmse_norm'});

    valid_idx = find(valid);

    if isempty(valid_idx)
        error('offset 搜索范围内没有有效重叠。');
    end

    best_i = valid_idx(1);

    for ii = valid_idx(:).'
        if rmse_norm(ii) < rmse_norm(best_i) - 1e-15
            best_i = ii;
        elseif abs(rmse_norm(ii) - rmse_norm(best_i)) <= 1e-15
            if abs(offsets(ii)) < abs(offsets(best_i))
                best_i = ii;
            end
        end
    end

    best_offset = offsets(best_i);
end

function [rtl_cmp, golden_cmp, cmp_index, rtl_start, golden_start] = align_by_offset(rtl_dec, golden_dec, offset)
% 根据给定样点偏移截取 RTL 数据与 Golden 数据的公共重叠部分进行对齐
    rtl_dec = rtl_dec(:);
    golden_dec = golden_dec(:);

    if offset >= 0
        rtl_start = 1;
        golden_start = 1 + offset;
    else
        rtl_start = 1 - offset;
        golden_start = 1;
    end

    n_cmp = min(numel(rtl_dec) - rtl_start + 1, numel(golden_dec) - golden_start + 1);

    if n_cmp <= 0
        rtl_cmp = int64([]);
        golden_cmp = int64([]);
        cmp_index = int64([]);
        return;
    end

    rtl_cmp = rtl_dec(rtl_start : rtl_start + n_cmp - 1);
    golden_cmp = golden_dec(golden_start : golden_start + n_cmp - 1);
    cmp_index = int64((0:n_cmp-1).');
end

function m = compute_error_metrics(rtl, ref, full_scale)
% 计算 RTL 输出相对于参考输出的各项误差、 SNR 及相关系数指标
    rtl = double(rtl(:));
    ref = double(ref(:));

    err = rtl - ref;
    abs_err = abs(err);

    m = struct();
    m.err = err;
    m.n = numel(err);
    m.mean_error_lsb = mean(err);
    m.mae_lsb = mean(abs_err);
    m.rmse_lsb = sqrt(mean(err.^2));
    m.max_abs_error_lsb = max(abs_err);
    m.p95_abs_error_lsb = percentile_local(abs_err, 95);
    m.p99_abs_error_lsb = percentile_local(abs_err, 99);
    m.rmse_norm_fullscale = m.rmse_lsb / full_scale;
    m.nrmse_fullscale_db = 20 * log10(m.rmse_norm_fullscale + eps);

    ref_rms = sqrt(mean(ref.^2));
    err_rms = sqrt(mean(err.^2));

    if ref_rms < eps
        m.nrmse_signal = NaN;
        m.snr_db = NaN;
    else
        m.nrmse_signal = err_rms / ref_rms;
        m.snr_db = 20 * log10(ref_rms / (err_rms + eps));
    end

    if std(rtl) > 0 && std(ref) > 0
        c = corrcoef(rtl, ref);
        m.corr_value = c(1, 2);
    else
        m.corr_value = NaN;
    end
end

function p = percentile_local(x, q)
% 对输入数据排序并通过线性插值计算指定百分位数
    x = sort(x(:));
    n = numel(x);

    if n == 0
        p = NaN;
        return;
    end

    if q <= 0
        p = x(1);
        return;
    end

    if q >= 100
        p = x(end);
        return;
    end

    pos = 1 + (n - 1) * q / 100;
    lo = floor(pos);
    hi = ceil(pos);

    if lo == hi
        p = x(lo);
    else
        p = x(lo) + (x(hi) - x(lo)) * (pos - lo);
    end
end

function print_metrics(m, name)
% 将指定误差指标输出到 MATLAB 命令窗口
    fprintf('%s：\n', name);
    fprintf('  样本数                 = %d\n', m.n);
    fprintf('  平均误差               = %.6f LSB\n', m.mean_error_lsb);
    fprintf('  MAE 平均绝对误差       = %.6f LSB\n', m.mae_lsb);
    fprintf('  RMSE 均方根误差        = %.6f LSB\n', m.rmse_lsb);
    fprintf('  最大绝对误差           = %.0f LSB\n', m.max_abs_error_lsb);
    fprintf('  95%% 绝对误差           = %.6f LSB\n', m.p95_abs_error_lsb);
    fprintf('  99%% 绝对误差           = %.6f LSB\n', m.p99_abs_error_lsb);
    fprintf('  RMSE / 满幅            = %.12e\n', m.rmse_norm_fullscale);
    fprintf('  NRMSE_FS               = %.6f dB\n', m.nrmse_fullscale_db);
    fprintf('  NRMSE_signal           = %.12e\n', m.nrmse_signal);
    fprintf('  SNR                    = %.6f dB\n', m.snr_db);
    fprintf('  相关系数               = %.12f\n\n', m.corr_value);
end

function write_metrics(fid, m, name)
% 将指定误差指标写入已经打开的文本报告文件
    fprintf(fid, '%s：\n', name);
    fprintf(fid, '样本数                 = %d\n', m.n);
    fprintf(fid, '平均误差               = %.6f LSB\n', m.mean_error_lsb);
    fprintf(fid, 'MAE 平均绝对误差       = %.6f LSB\n', m.mae_lsb);
    fprintf(fid, 'RMSE 均方根误差        = %.6f LSB\n', m.rmse_lsb);
    fprintf(fid, '最大绝对误差           = %.0f LSB\n', m.max_abs_error_lsb);
    fprintf(fid, '95%% 绝对误差           = %.6f LSB\n', m.p95_abs_error_lsb);
    fprintf(fid, '99%% 绝对误差           = %.6f LSB\n', m.p99_abs_error_lsb);
    fprintf(fid, 'RMSE / 满幅            = %.12e\n', m.rmse_norm_fullscale);
    fprintf(fid, 'NRMSE_FS               = %.6f dB\n', m.nrmse_fullscale_db);
    fprintf(fid, 'NRMSE_signal           = %.12e\n', m.nrmse_signal);
    fprintf(fid, 'SNR                    = %.6f dB\n', m.snr_db);
    fprintf(fid, '相关系数               = %.12f\n', m.corr_value);
end