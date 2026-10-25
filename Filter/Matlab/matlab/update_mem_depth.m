function update_mem_depth(file_name, mem_depth, golden_depth)
% 自动生成 RTL 参数头文件 rtl_auto_params.vh

    if nargin < 2 || isempty(mem_depth)
        error('write_rtl_auto_params: 必须输入 mem_depth。');
    end

    if nargin < 3 || isempty(golden_depth)
        golden_depth = mem_depth * 128;
    end

    mem_depth = double(mem_depth);
    golden_depth = double(golden_depth);

    if mem_depth <= 0 || mem_depth ~= floor(mem_depth)
        error('write_rtl_auto_params: mem_depth 必须是正整数。');
    end

    if golden_depth <= 0 || golden_depth ~= floor(golden_depth)
        error('write_rtl_auto_params: golden_depth 必须是正整数。');
    end

    out_dir = fileparts(file_name);
    if ~isempty(out_dir) && ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    fid = fopen(file_name, 'w');
    if fid < 0
        error('write_rtl_auto_params: 无法写入文件：%s', file_name);
    end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, '`ifndef RTL_AUTO_PARAMS_VH\n');
    fprintf(fid, '`define RTL_AUTO_PARAMS_VH\n\n');

    fprintf(fid, '// 本文件由 MATLAB 自动生成。\n');
    fprintf(fid, '// 请不要手动修改本文件。\n\n');

    fprintf(fid, '`define MEM_DEPTH_AUTO %d\n', mem_depth);
    fprintf(fid, '`define GOLDEN_OUT_DEPTH_AUTO %d\n\n', golden_depth);

    fprintf(fid, '`endif\n');
end
