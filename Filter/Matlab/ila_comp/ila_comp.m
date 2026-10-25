% ILA 抓取数据与 golden_out 比较
function ila_comp()
clc;
close all;

P = get_filter_project_paths();

OUT_W = 24;
OUTPUT_FS_HZ = 5644800;
FULL_SCALE = 2^(OUT_W - 1) - 1;

USE_VALID_ONLY = true;

% 如果 CSV 中存在 board_output_count，程序会先估计该抓取片段在golden_out_128x.mem 中的循环位置，再在估计位置附近搜索最佳对齐点
COUNT_BASED_SEARCH_RADIUS = -4096:4096;

% 仅当 board_output_count 不存在时，才使用下面的普通偏移搜索范围
SEARCH_OFFSET_RANGE = -4096:4096;

SHOW_LEN = 8000;
GLOBAL_PLOT_POINTS = 30000;
WRITE_POINT_ERROR_TABLE = true;
MAX_POINT_ERROR_ROWS = 300000;

% 只选择修改时间最新的 *.ila，不再回退到可能过期的 ila_capture.csv
[ila_source_file, ila_source_time] = find_latest_ila_archive(P);

golden_mem_file = find_first_existing_file({
    fullfile(P.rtl_export_dir, 'golden_out_128x.mem')
    fullfile(P.xsim_dir, 'golden_out_128x.mem')
});

report_root = fullfile(P.ila_comp_dir, 'ila_comp_report');
time_tag = datestr(now, 'yyyymmdd_HHMMSS');
report_dir = fullfile(report_root, ['ila_vs_golden_', time_tag]);

if ~exist(report_dir, 'dir')
    mkdir(report_dir);
end

diary(fullfile(report_dir, 'matlab_console_log.txt'));
diary_cleanup_guard = onCleanup(@() diary('off')); 

fprintf('============================================================\n');
fprintf('ILA 输出与 Golden 理想输出对比\n');
fprintf('工程根目录：%s\n', relpath_simple(P.project_root, P.project_root));
fprintf('ILA 数据源：%s\n', relpath_simple(ila_source_file, P.project_root));
fprintf('数据源类型：ILA 归档中的 waveform.csv\n');
fprintf('数据源时间：%s\n', ila_source_time);
fprintf('Golden 文件：%s\n', relpath_simple(golden_mem_file, P.project_root));
fprintf('报告目录：%s\n', relpath_simple(report_dir, P.project_root));
fprintf('============================================================\n\n');

if isempty(golden_mem_file)
    error('在 Matlab/rtl_export 和 Vivado xsim 目录中均未找到 golden_out_128x.mem。');
end

fprintf('---------------- 读取最新 ILA 归档 ----------------\n');
ila_lines = read_waveform_csv_from_ila(ila_source_file);
ila = read_ila_pcm_lines(ila_lines, OUT_W, USE_VALID_ONLY);

fprintf('ILA 原始行数          = %d\n', ila.raw_row_count);
fprintf('ILA 有效样本数        = %d\n', numel(ila.pcm_dec));
fprintf('仅使用有效输出行      = %d\n', USE_VALID_ONLY);
fprintf('是否找到有效标志列    = %d\n', ila.has_valid);
fprintf('是否找到板级计数列    = %d\n', ila.has_board_count);
fprintf('PCM 数据进制          = %s\n', ila.pcm_radix);
fprintf('ILA 数值范围          = [%d, %d]\n\n', min(ila.pcm_dec), max(ila.pcm_dec));

if isempty(ila.pcm_dec)
    error('未能从 ILA CSV 中读取到可用的 pcm_out 样本。');
end

fprintf('---------------- 读取 Golden 理想输出 ----------------\n');
golden_dec = read_signed_hex_mem_valid(golden_mem_file, OUT_W);

fprintf('Golden 有效样本数     = %d\n', numel(golden_dec));
fprintf('Golden 数值范围       = [%d, %d]\n\n', min(golden_dec), max(golden_dec));

if isempty(golden_dec)
    error('未能从 golden_out_128x.mem 中读取到有效的 24 bit 十六进制样本。');
end

fprintf('---------------- 数据对齐 ----------------\n');

if ila.has_board_count && ~isempty(ila.board_count)
    first_count = ila.board_count(1);
    golden_n_u64 = uint64(numel(golden_dec));
    estimated_start0 = double(mod(first_count, golden_n_u64));

    fprintf('使用 board_output_count 进行循环位置对齐。\n');
    fprintf('首个 board_output_count  = %s\n', uint64_to_hex_string(first_count));
    fprintf('估计 Golden 起始位置     = %d（从0开始）\n', estimated_start0);
    fprintf('搜索半径                 = [%d, %d] 个样本\n', ...
        min(COUNT_BASED_SEARCH_RADIUS), max(COUNT_BASED_SEARCH_RADIUS));

    [best_start0, best_shift, offset_table] = find_best_cyclic_start_rmse( ...
        ila.pcm_dec, golden_dec, estimated_start0, COUNT_BASED_SEARCH_RADIUS, FULL_SCALE);

    offset_table_file = fullfile(report_dir, 'cyclic_start_search_table.csv');
    writetable(offset_table, offset_table_file);

[ila_cmp_i64, golden_cmp_i64, cmp_index, golden_index0] = align_by_cyclic_start( ...
    ila.pcm_dec, golden_dec, best_start0);

    align_mode = '基于板级计数器的循环对齐';
    best_offset = NaN;
    ila_start = 1;
    golden_start = best_start0 + 1;
    best_at_search_edge = best_shift == min(COUNT_BASED_SEARCH_RADIUS) || ...
        best_shift == max(COUNT_BASED_SEARCH_RADIUS);

    fprintf('相对计数估计的最佳偏移   = %d\n', best_shift);
    fprintf('最佳 Golden 起始位置     = %d（从0开始）\n', best_start0);
    fprintf('搜索结果表               = %s\n\n', relpath_simple(offset_table_file, P.project_root));
    if best_at_search_edge
        warning('最佳偏移位于搜索范围边界，建议增大 COUNT_BASED_SEARCH_RADIUS。');
    end
else
    fprintf('未找到 board_output_count，改用严格整数样点偏移搜索。\n');
    fprintf('搜索范围                 = [%d, %d] 个样本\n', ...
        min(SEARCH_OFFSET_RANGE), max(SEARCH_OFFSET_RANGE));

    [best_offset, offset_table] = find_best_offset_strict_rmse( ...
        ila.pcm_dec, golden_dec, SEARCH_OFFSET_RANGE, FULL_SCALE);

    offset_table_file = fullfile(report_dir, 'offset_search_table.csv');
    writetable(offset_table, offset_table_file);

    [ila_cmp_i64, golden_cmp_i64, cmp_index, ila_start, golden_start] = align_by_offset( ...
        ila.pcm_dec, golden_dec, best_offset);

    align_mode = '严格整数样点偏移对齐';
    best_shift = NaN;
    best_start0 = golden_start - 1;
    golden_index0 = int64((golden_start - 1) : (golden_start + numel(golden_cmp_i64) - 2)).';
    best_at_search_edge = best_offset == min(SEARCH_OFFSET_RANGE) || ...
        best_offset == max(SEARCH_OFFSET_RANGE);

    fprintf('最佳 offset              = %d\n', best_offset);
    fprintf('ILA 起始位置             = %d（从0开始）\n', ila_start - 1);
    fprintf('Golden 起始位置          = %d（从0开始）\n', golden_start - 1);
    fprintf('搜索结果表               = %s\n\n', relpath_simple(offset_table_file, P.project_root));
    if best_at_search_edge
        warning('最佳 offset 位于搜索范围边界，建议增大 SEARCH_OFFSET_RANGE。');
    end
end

n_cmp = numel(ila_cmp_i64);

if n_cmp <= 0
    error('对齐后没有可参与比较的重叠样本。');
end

ila_cmp = double(ila_cmp_i64);
golden_cmp = double(golden_cmp_i64);

fprintf('实际参与比较样本数       = %d\n\n', n_cmp);

fprintf('---------------- 原始误差：ILA - Golden ----------------\n');
err = ila_cmp - golden_cmp;
abs_err = abs(err);
err_norm = err / FULL_SCALE;

metrics = compute_error_metrics(ila_cmp, golden_cmp, FULL_SCALE);
print_metrics(metrics, '原始误差');

summary_file = fullfile(report_dir, 'summary.txt');
fid = fopen(summary_file, 'w');
if fid < 0
    error('无法写入 summary.txt。');
end

fprintf(fid, 'ILA 输出与 Golden 理想输出对比\n');
fprintf(fid, '工程根目录               = %s\n', relpath_simple(P.project_root, P.project_root));
fprintf(fid, 'ILA 数据源               = %s\n', relpath_simple(ila_source_file, P.project_root));
fprintf(fid, 'ILA 数据源类型           = ILA 归档中的 waveform.csv\n');
fprintf(fid, 'ILA 数据源时间           = %s\n', ila_source_time);
fprintf(fid, 'Golden 文件              = %s\n', relpath_simple(golden_mem_file, P.project_root));
fprintf(fid, '对齐方式                 = %s\n', align_mode);
fprintf(fid, 'ILA 原始行数             = %d\n', ila.raw_row_count);
fprintf(fid, 'ILA 有效样本数           = %d\n', numel(ila.pcm_dec));
fprintf(fid, 'Golden 有效样本数        = %d\n', numel(golden_dec));
fprintf(fid, '实际参与比较样本数       = %d\n', n_cmp);
fprintf(fid, '仅使用有效输出行         = %d\n', USE_VALID_ONLY);
fprintf(fid, 'PCM 数据进制             = %s\n', ila.pcm_radix);
fprintf(fid, '最佳 offset              = %.0f\n', best_offset);
fprintf(fid, '相对计数估计的最佳偏移   = %.0f\n', best_shift);
fprintf(fid, '最佳 Golden 起始位置     = %d（从0开始）\n', best_start0);
fprintf(fid, 'ILA 起始位置             = %d（从0开始）\n', ila_start - 1);
fprintf(fid, 'Golden 起始位置          = %d（从0开始）\n\n', golden_start - 1);
fprintf(fid, '最佳结果是否位于搜索边界 = %d\n\n', best_at_search_edge);
write_metrics(fid, metrics, '原始误差');
fclose(fid);

fprintf('summary 已导出            = %s\n\n', relpath_simple(summary_file, P.project_root));

if WRITE_POINT_ERROR_TABLE
    n_write = min(n_cmp, MAX_POINT_ERROR_ROWS);
    point_file = fullfile(report_dir, 'point_error_table.csv');

    sample_in_buffer = nan(n_write, 1);
    board_count_hex = strings(n_write, 1);

    if ~isempty(ila.sample_in_buffer)
        sample_in_buffer = double(ila.sample_in_buffer(1:n_write));
    end

    if ~isempty(ila.board_count)
        for i = 1:n_write
            board_count_hex(i) = string(uint64_to_hex_string(ila.board_count(i)));
        end
    end

    T = table( ...
        double(cmp_index(1:n_write)), ...
        sample_in_buffer, ...
        board_count_hex, ...
        double(golden_index0(1:n_write)), ...
        ila_cmp(1:n_write), ...
        golden_cmp(1:n_write), ...
        err(1:n_write), ...
        abs_err(1:n_write), ...
        err_norm(1:n_write), ...
        'VariableNames', { ...
            'index', ...
            'ila_sample_in_buffer', ...
            'board_output_count_hex', ...
            'golden_index_zero_based', ...
            'ila_dec', ...
            'golden_dec', ...
            'error_lsb', ...
            'abs_error_lsb', ...
            'error_norm'} ...
    );

    writetable(T, point_file);
    fprintf('逐点误差表已导出         = %s\n', relpath_simple(point_file, P.project_root));
    fprintf('逐点误差表导出行数       = %d\n\n', n_write);
end

fprintf('---------------- 生成对比图 ----------------\n');

n_show = min([SHOW_LEN, n_cmp]);
t_ms = (0:n_show-1).' / OUTPUT_FS_HZ * 1000;

figure('Name', 'ILA 与 Golden 波形对比', 'NumberTitle', 'off');
plot(t_ms, ila_cmp(1:n_show) / FULL_SCALE, 'b');
hold on;
plot(t_ms, golden_cmp(1:n_show) / FULL_SCALE, 'r--');
grid on;
xlabel('时间 / ms');
ylabel('归一化幅度');
title('ILA pcm\_out 与 Golden 波形对比');
legend('ILA 输出', 'Golden 理想输出');
saveas(gcf, fullfile(report_dir, '01_ila_golden_overlay.png'));

figure('Name', '误差波形', 'NumberTitle', 'off');
plot(t_ms, err(1:n_show), 'k');
grid on;
xlabel('时间 / ms');
ylabel('误差 / LSB');
title('误差波形：ILA - Golden');
saveas(gcf, fullfile(report_dir, '02_error_waveform.png'));

figure('Name', '误差直方图', 'NumberTitle', 'off');
histogram(err, 200);
grid on;
xlabel('误差 / LSB');
ylabel('出现次数');
title('误差分布直方图：ILA - Golden');
saveas(gcf, fullfile(report_dir, '03_error_histogram.png'));

n_global = min(GLOBAL_PLOT_POINTS, n_cmp);
global_idx = unique(round(linspace(1, n_cmp, n_global))).';

figure('Name', '全局抽样对比', 'NumberTitle', 'off');
plot(global_idx, ila_cmp(global_idx) / FULL_SCALE, 'b');
hold on;
plot(global_idx, golden_cmp(global_idx) / FULL_SCALE, 'r--');
grid on;
xlabel('参与比较的样本序号');
ylabel('归一化幅度');
title('全局抽样波形对比');
legend('ILA 输出', 'Golden 理想输出');
saveas(gcf, fullfile(report_dir, '04_global_comparison.png'));

fprintf('图像已保存到             = %s\n\n', relpath_simple(report_dir, P.project_root));

fprintf('============================================================\n');
fprintf('完成。\n');
fprintf('报告目录：%s\n', relpath_simple(report_dir, P.project_root));
fprintf('============================================================\n');

diary off;

end

function [source_file, source_time] = find_latest_ila_archive(P)
    % 在 Vivado 硬件会话目录中递归寻找所有 *.ila，并选修改时间最新者
    hw_root = fullfile(P.vivado_dir, 'Filter.hw');

    if ~exist(hw_root, 'dir')
        error('未找到 Vivado 硬件会话目录：%s', hw_root);
    end

    ila_files = dir(fullfile(hw_root, '**', '*.ila'));
    if isempty(ila_files)
        error(['Filter.hw 中没有已保存的 *.ila。请先在 Vivado Hardware Manager ' ...
            '中完成抓取并保存 ILA 数据。']);
    end

    [~, newest_idx] = max([ila_files.datenum]);
    newest = ila_files(newest_idx);
    source_file = fullfile(newest.folder, newest.name);
    source_time = datestr(newest.datenum, 'yyyy-mm-dd HH:MM:SS');

    wdb_files = dir(fullfile(hw_root, '**', '*.wdb'));
    if ~isempty(wdb_files)
        newest_wdb_time = max([wdb_files.datenum]);
        one_second = 1 / (24 * 60 * 60);
        if newest_wdb_time > newest.datenum + one_second
            warning(['检测到比最新 *.ila 更新的硬件波形数据库。MATLAB 只能读取' ...
                '已保存的 ILA 归档，请先在 Vivado 中保存最新抓取结果。']);
        end
    end
end

function lines = read_waveform_csv_from_ila(ila_file)
    % 通过 Java ZIP 流只读取其中的waveform.csv，不解压 .wdb、.vcd 等无关文件，也不产生临时文件
    try
        zip_file = javaObject('java.util.zip.ZipFile', char(ila_file));
    catch ME
        error(['无法打开 ILA 归档，文件可能仍在保存或已经损坏：\n%s\n' ...
            'Java 信息：%s'], ila_file, ME.message);
    end
    zip_cleanup = onCleanup(@() close_java_quietly(zip_file));

    entries = zip_file.entries();
    waveform_entry = [];

    while entries.hasMoreElements()
        entry = entries.nextElement();
        entry_name = char(entry.getName());
        if strcmpi(entry_name, 'waveform.csv') || ...
                endsWith(lower(entry_name), '/waveform.csv')
            waveform_entry = entry;
            break;
        end
    end

    if isempty(waveform_entry)
        error('ILA 归档中没有找到 waveform.csv：%s', ila_file);
    end

    input_stream = zip_file.getInputStream(waveform_entry);
    reader = javaObject('java.io.BufferedReader', ...
        javaObject('java.io.InputStreamReader', input_stream, 'UTF-8'));
    reader_cleanup = onCleanup(@() close_java_quietly(reader)); 

    % Vivado ILA 深度通常是 2 的幂，先按 16386 行预分配，空间不足时自动扩展
    lines = strings(16386, 1);
    line_count = 0;

    while true
        java_line = reader.readLine();
        if isempty(java_line)
            break;
        end

        line_count = line_count + 1;
        if line_count > numel(lines)
            lines(end + 16384, 1) = ""; 
        end
        lines(line_count) = string(char(java_line));
    end

    lines = lines(1:line_count);
    if numel(lines) < 3
        error('ILA 归档中的 waveform.csv 内容不完整：%s', ila_file);
    end
end

function close_java_quietly(java_object)
    % 安静关闭 Java 流；清理失败不覆盖主要错误或分析结果
    try
        java_object.close();
    catch
    end
end

function P = get_filter_project_paths()
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
    P.ila_comp_dir = this_dir;
    P.rtl_export_dir = fullfile(matlab_dir, 'rtl_export');
    P.vivado_dir = fullfile(project_root, 'Vivado', 'Filter');
    P.xsim_dir = fullfile(P.vivado_dir, 'Filter.sim', 'sim_1', 'behav', 'xsim');
end

function file_name = find_first_existing_file(candidates)
    file_name = '';
    for i = 1:numel(candidates)
        c = candidates{i};
        if exist(c, 'file')
            file_name = c;
            return;
        end
    end
end

function s = relpath_simple(path_name, root_name)
    path_name = char(path_name);
    root_name = char(root_name);

    if isempty(path_name)
        s = '';
        return;
    end

    path_norm = strrep(path_name, '/', filesep);
    root_norm = strrep(root_name, '/', filesep);

    if strcmpi(path_norm, root_norm)
        s = '.';
        return;
    end

    prefix = [root_norm filesep];
    if strncmpi(path_norm, prefix, length(prefix))
        s = path_norm(length(prefix)+1:end);
    else
        s = path_norm;
    end
end

function ila = read_ila_pcm_lines(lines, width, use_valid_only)
    % 解析 Vivado waveform.csv 的文本行，并依据 Radix 行解释每一列
    lines = strip(lines);

    if numel(lines) < 2
        error('ILA waveform.csv 内容过短。');
    end

    headers = split_csv_line(lines(1));

    idx_sample = find_column(headers, {'Sample in Buffer'});
    idx_pcm = find_column(headers, {'ila_pcm_out_q[23:0]', 'ila_pcm_out_q', ...
        'pcm_out[23:0]', 'pcm_out'});
    idx_valid = find_column(headers, {'ila_pcm_out_valid', 'pcm_out_valid'});
    idx_board = find_column(headers, {'board_output_count[31:0]', 'board_output_count'});

    if idx_pcm <= 0
        error('ILA CSV 中未找到 ila_pcm_out_q[23:0] 或 pcm_out[23:0] 数据列。');
    end

    has_valid = idx_valid > 0;
    has_board = idx_board > 0;

    % 没有 Radix 行时沿用旧 CSV 的默认规则：PCM/valid/count 均按 HEX
    sample_radix = 'UNSIGNED';
    pcm_radix = 'HEX';
    valid_radix = 'HEX';
    board_radix = 'HEX';
    data_start = 2;

    if startsWith(char(lines(2)), 'Radix', 'IgnoreCase', true)
        radix_parts = split_csv_line(lines(2));
        sample_radix = get_column_radix(radix_parts, idx_sample, sample_radix);
        pcm_radix = get_column_radix(radix_parts, idx_pcm, pcm_radix);
        valid_radix = get_column_radix(radix_parts, idx_valid, valid_radix);
        board_radix = get_column_radix(radix_parts, idx_board, board_radix);
        data_start = 3;
    end

    max_rows = max(numel(lines) - data_start + 1, 0);
    pcm_all = zeros(max_rows, 1, 'int64');
    valid_all = false(max_rows, 1);
    board_all = zeros(max_rows, 1, 'uint64');
    sample_all = zeros(max_rows, 1, 'int64');
    keep_line = false(max_rows, 1);

    row = 0;
    for i = data_start:numel(lines)
        line = char(lines(i));
        if isempty(strtrim(line))
            continue;
        end

        parts = split_csv_line(line);
        if numel(parts) < idx_pcm
            continue;
        end

        [pcm_v, ok_pcm] = parse_ila_signed(parts{idx_pcm}, pcm_radix, width);
        if ~ok_pcm
            continue;
        end

        row = row + 1;
        pcm_all(row) = pcm_v;

        if idx_sample > 0 && numel(parts) >= idx_sample
            [sample_u, ok_sample] = parse_ila_unsigned(parts{idx_sample}, sample_radix);
            if ok_sample
                sample_all(row) = int64(sample_u);
            else
                sample_all(row) = int64(row - 1);
            end
        else
            sample_all(row) = int64(row - 1);
        end

        if has_valid && numel(parts) >= idx_valid
            [valid_u, ok_valid] = parse_ila_unsigned(parts{idx_valid}, valid_radix);
            valid_all(row) = ok_valid && valid_u ~= 0;
        else
            valid_all(row) = true;
        end

        if has_board && numel(parts) >= idx_board
            [board_u, ok_board] = parse_ila_unsigned(parts{idx_board}, board_radix);
            if ok_board
                board_all(row) = board_u;
            end
        end

        keep_line(row) = true;
    end

    pcm_all = pcm_all(1:row);
    valid_all = valid_all(1:row);
    board_all = board_all(1:row);
    sample_all = sample_all(1:row);
    keep_line = keep_line(1:row);

    if use_valid_only && has_valid
        keep = keep_line & valid_all;
    else
        keep = keep_line;
    end

    ila = struct();
    ila.raw_row_count = row;
    ila.has_valid = has_valid;
    ila.has_board_count = has_board;
    ila.pcm_radix = pcm_radix;
    ila.valid_radix = valid_radix;
    ila.board_radix = board_radix;
    ila.pcm_dec = pcm_all(keep);
    ila.valid = valid_all(keep);
    ila.sample_in_buffer = sample_all(keep);

    if has_board
        ila.board_count = board_all(keep);
    else
        ila.board_count = zeros(0, 1, 'uint64');
    end
end

function parts = split_csv_line(line)
    parts = regexp(char(line), ',', 'split');
    for i = 1:numel(parts)
        parts{i} = strtrim(parts{i});
    end
end

function idx = find_column(headers, names)
    idx = 0;
    clean_headers = lower(strtrim(headers));

    for n = 1:numel(names)
        target = lower(strtrim(names{n}));
        for i = 1:numel(clean_headers)
            if strcmp(clean_headers{i}, target)
                idx = i;
                return;
            end
        end
    end

    for n = 1:numel(names)
        target = lower(strtrim(names{n}));
        for i = 1:numel(clean_headers)
            if contains(clean_headers{i}, target)
                idx = i;
                return;
            end
        end
    end
end

function radix = get_column_radix(radix_parts, column_index, default_radix)
    % 从 Vivado 的 Radix 行读取指定列进制
    radix = upper(default_radix);
    if column_index <= 0 || column_index > numel(radix_parts)
        return;
    end

    candidate = upper(strtrim(char(radix_parts{column_index})));
    candidate = regexprep(candidate, '(?i)^RADIX\s*-\s*', '');
    if ~isempty(candidate)
        radix = candidate;
    end
end

function [v, ok] = parse_ila_signed(s, radix, width)
    % 按 Vivado 指定 Radix 读取有符号探针值
    % SIGNED 表示带符号十进制；其他无符号表示法按 width 做二补码还原
    radix = upper(strtrim(char(radix)));

    if strcmp(radix, 'SIGNED')
        token = clean_ila_token(s);
        value = str2double(token);
        min_value = -2^(width - 1);
        max_value = 2^(width - 1) - 1;
        ok = isfinite(value) && value == fix(value) && ...
            value >= min_value && value <= max_value;

        if ok
            v = int64(value);
        else
            v = int64(0);
        end
        return;
    end

    [u, ok] = parse_ila_unsigned(s, radix);
    if ~ok
        v = int64(0);
        return;
    end

    u_double = double(u);
    modulus = 2^width;
    if u_double < 0 || u_double >= modulus
        v = int64(0);
        ok = false;
        return;
    end

    if u_double >= 2^(width - 1)
        u_double = u_double - modulus;
    end
    v = int64(u_double);
end

function [u, ok] = parse_ila_unsigned(s, radix)
    % 按 Vivado Radix 读取无符号值，支持十进制、十六进制、二进制和八进制。
    radix = upper(strtrim(char(radix)));
    token = clean_ila_token(s);

    if isempty(token)
        u = uint64(0);
        ok = false;
        return;
    end

    switch radix
        case {'UNSIGNED', 'SIGNED', 'DEC', 'DECIMAL'}
            value = str2double(token);
            ok = isfinite(value) && value == fix(value) && value >= 0 && value <= flintmax;

        case {'HEX', 'HEXADECIMAL'}
            token = remove_radix_prefix(token, 'h', '0x');
            ok = ~isempty(regexp(token, '^[0-9a-fA-F]+$', 'once'));
            if ok
                value = hex2dec(token);
            else
                value = 0;
            end

        case {'BIN', 'BINARY'}
            token = remove_radix_prefix(token, 'b', '0b');
            ok = ~isempty(regexp(token, '^[01]+$', 'once'));
            if ok
                value = bin2dec(token);
            else
                value = 0;
            end

        case {'OCT', 'OCTAL'}
            token = remove_radix_prefix(token, 'o', '0o');
            ok = ~isempty(regexp(token, '^[0-7]+$', 'once'));
            if ok
                value = base2dec(token, 8);
            else
                value = 0;
            end

        otherwise
            error('不支持的 Vivado ILA Radix：%s', radix);
    end

    if ok
        u = uint64(value);
    else
        u = uint64(0);
    end
end

function token = clean_ila_token(s)
    % 删除 Vivado 数字字段中可能出现的引号、空格和下划线
    s = strtrim(char(s));
    s = strrep(s, '"', '');
    s = strrep(s, '_', '');
    token = strtrim(s);
end

function token = remove_radix_prefix(token, tick_letter, c_prefix)
    tick_pattern = ['''' tick_letter];
    tick_pos = regexpi(token, tick_pattern, 'once');
    if ~isempty(tick_pos)
        token = token(tick_pos + 2:end);
    elseif length(token) >= 2 && strcmpi(token(1:2), c_prefix)
        token = token(3:end);
    end
end

function data_dec = read_signed_hex_mem_valid(file_name, width)
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

function [best_start0, best_shift, T] = find_best_cyclic_start_rmse(ila_dec, golden_dec, estimated_start0, shift_range, full_scale)
    ila_norm = double(ila_dec(:)) / full_scale;
    golden_norm = double(golden_dec(:)) / full_scale;

    shifts = shift_range(:);
    golden_n = numel(golden_norm);
    n = numel(ila_norm);

    rmse_lsb = inf(numel(shifts), 1);
    rmse_norm = inf(numel(shifts), 1);
    start_zero_based = zeros(numel(shifts), 1);

    for k = 1:numel(shifts)
        start0 = mod(estimated_start0 + shifts(k), golden_n);
        idx = mod(start0 + (0:n-1).', golden_n) + 1;
        g = golden_norm(idx);
        e = ila_norm - g;
        rmse = sqrt(mean(e.^2));

        start_zero_based(k) = start0;
        rmse_norm(k) = rmse;
        rmse_lsb(k) = rmse * full_scale;
    end

    T = table(shifts, start_zero_based, repmat(n, numel(shifts), 1), rmse_lsb, rmse_norm, ...
        'VariableNames', {'shift_from_count', 'golden_start_zero_based', 'overlap_n', 'rmse_lsb', 'rmse_norm'});

    [~, best_i] = min(rmse_norm);
    best_start0 = start_zero_based(best_i);
    best_shift = shifts(best_i);
end

function [ila_cmp, golden_cmp, cmp_index, golden_index0] = align_by_cyclic_start(ila_dec, golden_dec, start_zero_based)
    ila_cmp = ila_dec(:);
    golden_dec = golden_dec(:);
    n = numel(ila_cmp);
    golden_n = numel(golden_dec);

    golden_index0 = int64(mod(start_zero_based + (0:n-1).', golden_n));
    golden_cmp = golden_dec(double(golden_index0) + 1);
    cmp_index = int64((0:n-1).');
end

function [best_offset, T] = find_best_offset_strict_rmse(rtl_dec, golden_dec, offset_range, full_scale)
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
        error('SEARCH_OFFSET_RANGE 范围内没有有效重叠数据。');
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

function m = compute_error_metrics(x, ref, full_scale)
    x = double(x(:));
    ref = double(ref(:));

    err = x - ref;
    abs_err = abs(err);

    m = struct();
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

    if std(x) > 0 && std(ref) > 0
        c = corrcoef(x, ref);
        m.corr_value = c(1, 2);
    else
        m.corr_value = NaN;
    end
end

function p = percentile_local(x, q)
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
    fprintf('%s：\n', name);
    fprintf('  样本数                 = %d\n', m.n);
    fprintf('  平均误差               = %.6f LSB\n', m.mean_error_lsb);
    fprintf('  MAE 平均绝对误差       = %.6f LSB\n', m.mae_lsb);
    fprintf('  RMSE 均方根误差        = %.6f LSB\n', m.rmse_lsb);
    fprintf('  最大绝对误差           = %.0f LSB\n', m.max_abs_error_lsb);
    fprintf('  95%% 绝对误差           = %.6f LSB\n', m.p95_abs_error_lsb);
    fprintf('  99%% 绝对误差           = %.6f LSB\n', m.p99_abs_error_lsb);
    fprintf('  RMSE / 满幅            = %.12e\n', m.rmse_norm_fullscale);
    fprintf('  NRMSE_FS                = %.6f dB\n', m.nrmse_fullscale_db);
    fprintf('  NRMSE_signal            = %.12e\n', m.nrmse_signal);
    fprintf('  SNR                     = %.6f dB\n', m.snr_db);
    fprintf('  相关系数               = %.12f\n\n', m.corr_value);
end

function write_metrics(fid, m, name)
    fprintf(fid, '%s：\n', name);
    fprintf(fid, '样本数                 = %d\n', m.n);
    fprintf(fid, '平均误差               = %.6f LSB\n', m.mean_error_lsb);
    fprintf(fid, 'MAE 平均绝对误差       = %.6f LSB\n', m.mae_lsb);
    fprintf(fid, 'RMSE 均方根误差        = %.6f LSB\n', m.rmse_lsb);
    fprintf(fid, '最大绝对误差           = %.0f LSB\n', m.max_abs_error_lsb);
    fprintf(fid, '95%% 绝对误差           = %.6f LSB\n', m.p95_abs_error_lsb);
    fprintf(fid, '99%% 绝对误差           = %.6f LSB\n', m.p99_abs_error_lsb);
    fprintf(fid, 'RMSE / 满幅            = %.12e\n', m.rmse_norm_fullscale);
    fprintf(fid, 'NRMSE_FS                = %.6f dB\n', m.nrmse_fullscale_db);
    fprintf(fid, 'NRMSE_signal            = %.12e\n', m.nrmse_signal);
    fprintf(fid, 'SNR                     = %.6f dB\n', m.snr_db);
    fprintf(fid, '相关系数               = %.12f\n', m.corr_value);
end

function s = uint64_to_hex_string(u)
    s = dec2hex(double(u), 8);
end
