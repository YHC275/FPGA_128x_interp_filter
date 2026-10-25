function export_rtl_files(out_dir, q_stage0, q_halfband, q_cic_comp, x_int24, y128_int24, cfg)
% 导出最终固定 RTL 所需的 .mem 文件

    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    q_stage0   = int64(q_stage0(:));
    q_halfband = int64(q_halfband(:));
    q_cic_comp = int64(q_cic_comp(:));
    x_int24    = int64(x_int24(:));
    y128_int24 = int64(y128_int24(:));

    write_hex_mem(fullfile(out_dir, 'coef_stage0_even.mem'),   q_stage0(1:2:end),   cfg.stage0_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_stage0_odd.mem'),    q_stage0(2:2:end),   cfg.stage0_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_halfband_even.mem'), q_halfband(1:2:end), cfg.halfband_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_halfband_odd.mem'),  q_halfband(2:2:end), cfg.halfband_coef_width);
    write_hex_mem(fullfile(out_dir, 'coef_cic_comp.mem'),      q_cic_comp,          cfg.cic_comp_coef_width);

    write_hex_mem(fullfile(out_dir, 'test_audio_input.mem'),  x_int24,    24);
    write_hex_mem(fullfile(out_dir, 'golden_out_128x.mem'), y128_int24, 24);

    sync_to_vivado_mem_dirs(out_dir);
end

function sync_to_vivado_mem_dirs(out_dir)
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
        'coef_cic_comp.mem', ...
        'test_audio_input.mem', ...
        'golden_out_128x.mem' ...
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
