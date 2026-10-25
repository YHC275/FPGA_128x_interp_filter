% 44.1 kHz 输入 / 128x 插值 
% Stage0 2x → Halfband 2x → Halfband 2x →  CIC 补偿 FIR → CIC 16x

clear;
clc;
close all;

% 仿真文件存放路径
this_file = mfilename('fullpath');
this_dir  = fileparts(this_file);
root_dir  = fileparts(this_dir);
project_dir = fileparts(root_dir);
export_dir = fullfile(root_dir, 'rtl_export');

restoredefaultpath;
rehash;
addpath(this_dir);

N_FLUSH = 1200;                % 覆盖 2047-tap Stage0 的完整尾部冲刷

% MP3 音频文件路径
MP3_FILE = fullfile(this_dir, 'test_audio.mp3');

MAX_AUDIO_SAMPLES = [];        % 使用MP3音频个数

EXPORT_MEM = true;             % 是否导出 .mem 文件
MAKE_PLOTS = true;             % 是否画图

COEF_CTRL_DESIGN   = true;     % 是否重新设计滤波器系数
UPDATE_COEF_MEM    = true;     
UPDATE_LOAD_COEFFS_M = true;   % 是否覆盖系数

% Pro 版本使用经硬件映射约束验证的固定 18-bit 系数/24-bit 数据字长。
% 不再运行旧版面向最小资源的激进混合字长搜索。
WORDLENGTH_OPTIMIZE = false;

if COEF_CTRL_DESIGN
    [h_stage0, h_halfband, h_cic_comp, ...
     q_stage0, q_halfband, q_cic_comp, cfg, design_info] = ...
        design_all_filters(export_dir, UPDATE_COEF_MEM, ...
            UPDATE_LOAD_COEFFS_M, WORDLENGTH_OPTIMIZE);

    coef_source = '重新设计滤波器系数';
else
    [h_stage0, h_halfband, h_cic_comp, ...
     q_stage0, q_halfband, q_cic_comp, cfg] = load_coeffs();

    design_info = [];
    coef_source = '使用原有固定系数';
end

fprintf('================ 44.1 kHz / 128x RTL 链路仿真 ================\n');
fprintf('系数来源               = %s\n', coef_source);
if ~isempty(design_info)
    fprintf('系数设计方法           = %s\n', design_info.method);
end
fprintf('输入采样率 Fs_in        = %.1f Hz\n', cfg.input_fs_hz);
fprintf('输出采样率 Fs_out       = %.1f Hz\n', cfg.output_fs_hz);
fprintf('512Fs 主时钟            = %.4f MHz\n', cfg.clk_audio_512fs_hz / 1e6);
fprintf('插值倍率                = %d\n', cfg.interp_factor);
fprintf('OUTPUT_ALIGN_DELAY     = %d 个有效输出样本\n', cfg.output_align_delay);
fprintf('Stage_0 taps             = ph0 %d / ph1 %d\n', cfg.stage0_ph0_taps, cfg.stage0_ph1_taps);
fprintf('Halfband taps     = ph0 %d / ph1 %d\n', cfg.hb_ph0_taps, cfg.hb_ph1_taps);
fprintf('CIC                    = rate %d, stages %d, gain_shift %d\n', cfg.cic_rate, cfg.cic_stages, cfg.cic_gain_shift);
fprintf('低速前置 CIC comp taps = %d\n', cfg.cic_comp_taps);
fprintf('Stage0 目标通带/阻带边缘 = %.3f / %.3f Hz\n', ...
    cfg.spec.passband_hz(2), cfg.spec.stopband_start_hz);
fprintf('Stage0 width          = coef %d/F%d, acc %d\n', ...
    cfg.stage0_coef_width, cfg.stage0_coef_frac, cfg.stage0_acc_width);
fprintf('Halfband width        = coef %d/F%d, acc %d\n', ...
    cfg.halfband_coef_width, cfg.halfband_coef_frac, cfg.halfband_acc_width);
fprintf('CIC width             = acc %d\n', cfg.cic_acc_width);
fprintf('CIC comp width        = coef %d/F%d, acc %d\n', ...
    cfg.cic_comp_coef_width, cfg.cic_comp_coef_frac, cfg.cic_comp_acc_width);
fprintf('级间样本位宽         = Stage0 %d, HB1 %d, HB2 %d, CIC %d, Comp %d\n', ...
    cfg.stage0_data_width, cfg.halfband1_data_width, ...
    cfg.halfband2_data_width, cfg.cic_data_width, ...
    cfg.cic_comp_data_width);

%  读取 MP3 音频并转换为 44.1 kHz / 24 位有符号 PCM 输入测试信号
[x_int24, x_float, audio_info] = make_demo_signal(MP3_FILE, N_FLUSH, MAX_AUDIO_SAMPLES);
int24_full_scale = 2^23 - 1;

fprintf('输入音频文件           = %s\n', audio_info.file_name);
fprintf('原始音频采样率         = %.1f Hz\n', audio_info.original_fs_hz);
fprintf('目标输入采样率         = %.1f Hz\n', audio_info.target_fs_hz);
fprintf('有效音频样本数         = %d\n', audio_info.valid_audio_samples);
fprintf('flush 0 样本数         = %d\n', audio_info.flush_samples);
fprintf('输入样本总数           = %d\n', audio_info.total_samples);
fprintf('有效音频时长           = %.6f s\n', audio_info.duration_sec);

%  运行与 RTL 定点链路
n_out = numel(x_int24) * cfg.interp_factor;
fprintf('开始 RTL 定点仿真，目标输出样本数 = %d ...\n', n_out);
[y_rtl_int24, trace] = run_rtl_fixed(x_int24, q_stage0, q_halfband, q_cic_comp, cfg, n_out);

fprintf('RTL 仿真完成。\n');
fprintf('RTL 接收输入样本数       = %d\n', trace.input_accepted);
fprintf('RTL 产生输出样本数       = %d\n', trace.generated_outputs);
fprintf('总 512Fs 周期数          = %d\n', trace.total_cycles);
if ~isempty(trace.first_output_cycle)
    fprintf('第一个 pcm_out_valid 周期 = %d\n', trace.first_output_cycle);
end

%  运行浮点参考链路
[y_float_ref, h_eq] = run_float_reference(x_float, h_stage0, h_halfband, h_cic_comp, cfg);

% 为了与 RTL 的 OUTPUT_ALIGN_DELAY 对齐，浮点参考也补同样的有效样本延时
y_float_ref_aligned = [zeros(cfg.output_align_delay, 1); y_float_ref(:)];
y_float_ref_aligned = y_float_ref_aligned(1:min(numel(y_float_ref_aligned), numel(y_rtl_int24)));

y_rtl_float = double(y_rtl_int24(1:numel(y_float_ref_aligned))) / double(int24_full_scale);
err = y_float_ref_aligned(:) - y_rtl_float(:);
rmse = sqrt(mean(err.^2));
max_abs_err = max(abs(err));

resp = analyze_response(h_eq, cfg);

fprintf('---------------- 128x 频响指标 ----------------\n');
fprintf('10 Hz 增益             = %+0.6f dB\n', resp.gain_10hz_db);
fprintf('20 kHz 增益            = %+0.6f dB\n', resp.gain_20khz_db);
fprintf('%.3f kHz 增益          = %+0.6f dB\n', ...
    resp.spec.stopband_start_hz / 1e3, resp.gain_stop_edge_db);
fprintf('22.05 kHz 增益          = %+0.6f dB\n', resp.gain_22050hz_db);
fprintf('24.10 kHz 增益          = %+0.6f dB\n', resp.gain_24100hz_db);
fprintf('通带最大值            = %+0.6f dB\n', resp.pass_max_db);
fprintf('通带最小值            = %+0.6f dB\n', resp.pass_min_db);
fprintf('通带峰峰值纹波        = %0.6f dB\n', resp.pass_ripple_pp_db);
fprintf('通带相对 0 dB 最大偏差 = %0.6f dB，要求 <= 0.05 dB，判定 %s\n', resp.pass_ripple_pm_db, pass_fail_str(resp.passband_ok));
fprintf('阻带衰减              = %0.6f dB，要求 >= 70 dB，判定 %s\n', resp.stop_attn_db, pass_fail_str(resp.stopband_ok));
fprintf('线性相位对称误差      = %.3e，判定 %s\n', resp.symmetry_max_err, pass_fail_str(resp.linear_phase_ok));
fprintf('总体指标判定          = %s\n', pass_fail_str(resp.overall_ok));

fprintf('---------------- RTL 定点对比 ----------------\n');
fprintf('浮点参考 / RTL 定点 RMSE     = %.10f\n', rmse);
fprintf('浮点参考 / RTL 定点最大误差  = %.10f\n', max_abs_err);

% 导出 RTL 文件
if EXPORT_MEM
    export_rtl_files(export_dir, q_stage0, q_halfband, q_cic_comp, ...
        x_int24, y_rtl_int24, cfg);
    fprintf('已导出 RTL .mem 文件到：%s\n', export_dir);
    % 自动生成 RTL 参数头文件 rtl_auto_params.vh
    rtl_dir = fullfile(project_dir, 'Vivado', 'Filter', ...
        'Filter.srcs', 'sources_1', 'new');

    auto_param_file = fullfile(rtl_dir, 'update_mem_depth.vh');

   update_mem_depth( ...
        auto_param_file, ...
        numel(x_int24), ...
        numel(y_rtl_int24));

    fprintf('已生成 RTL 自动参数头文件：%s\n', auto_param_file);
    fprintf('MEM_DEPTH_AUTO        = %d\n', numel(x_int24));
    fprintf('GOLDEN_OUT_DEPTH_AUTO = %d\n', numel(y_rtl_int24));
end

%  画图
if MAKE_PLOTS

    % 基本显示长度设置
    in_len_all  = numel(x_float);          % 全部输入样点
    out_len_all = numel(y_rtl_float);      % 全部 RTL 输出样点

    show_cmp_len = min(4000, ...
        min(numel(y_float_ref_aligned), numel(y_rtl_float)));

    show_err_len = min(show_cmp_len, numel(err));

    show_trace_len = min(4000, numel(trace.output_valid_cycles));

    % 图 1：全部 MP3 输入样点波形
    figure('Name', 'MP3 音频输入全部样点', 'NumberTitle', 'off');

    t_in = (0:in_len_all-1).' / cfg.input_fs_hz;

    plot(t_in, x_float(:), 'LineWidth', 0.8);
    grid on;
    xlabel('时间 / s');
    ylabel('幅度');
    title(sprintf('MP3 音频输入全部 %d 个样点，采样率 %.1f kHz', ...
        in_len_all, cfg.input_fs_hz / 1e3));

    if ~isempty(t_in)
        xlim([t_in(1), t_in(end)]);
    end

    % 图 2：全部 RTL 输出样点波形
    figure('Name', 'RTL 定点链路全部输出样点', 'NumberTitle', 'off');

    t_out = (0:out_len_all-1).' / cfg.output_fs_hz;

    plot(t_out, y_rtl_float(:), 'LineWidth', 0.8);
    grid on;
    xlabel('时间 / s');
    ylabel('幅度');
    title(sprintf('RTL 定点链路全部输出 %d 个样点，输出采样率 %.4f MHz', ...
        out_len_all, cfg.output_fs_hz / 1e6));

    if ~isempty(t_out)
        xlim([t_out(1), t_out(end)]);
    end

       % 图 3：全部浮点参考链路输出样点波形
    figure('Name', '浮点参考链路全部输出样点', 'NumberTitle', 'off');

    float_out_len_all = numel(y_float_ref);
    t_float_out = (0:float_out_len_all-1).' / cfg.output_fs_hz;

    plot(t_float_out, y_float_ref(:), 'LineWidth', 0.8);
    grid on;
    xlabel('时间 / s');
    ylabel('幅度');
    title(sprintf('浮点参考链路全部输出 %d 个样点，输出采样率 %.4f MHz', ...
        float_out_len_all, cfg.output_fs_hz / 1e6));

    if ~isempty(t_float_out)
        xlim([t_float_out(1), t_float_out(end)]);
    end

    % 图 4：等效总频率响应
    figure('Name', '固定 44.1kHz / 128x：等效总频率响应', 'NumberTitle', 'off');

    subplot(2,1,1);

    plot(resp.f_hz, resp.mag_db, 'LineWidth', 1.2);
    grid on;
    xlabel('频率 / Hz');
    ylabel('幅度 / dB');
    title('128x 等效总频率响应');
    xlim([0, min(cfg.output_fs_hz/2, 120000)]);
    ylim([-140, 5]);
    xline(resp.spec.passband_hz(2), '--');
    xline(resp.spec.stopband_start_hz, '--');

    subplot(2,1,2);

    plot(resp.f_hz, resp.mag_db, 'LineWidth', 1.2);
    hold on;
    yline(+resp.spec.passband_pm_db, '--');
    yline(-resp.spec.passband_pm_db, '--');
    xline(10, '--');
    xline(resp.spec.passband_hz(2), '--');
    xline(resp.spec.stopband_start_hz, '--');

    grid on;
    xlabel('频率 / Hz');
    ylabel('幅度 / dB');
    title('10 Hz ~ 20 kHz 通带与超窄过渡带细节');
    xlim([19500, 20300]);
    ylim([-120, 2]);

    % 图 5：严格线性相位验证
    figure('Name', '严格线性相位验证', 'NumberTitle', 'off');

    subplot(2,1,1);

    plot(h_eq, 'LineWidth', 1.1);
    hold on;
    plot(flipud(h_eq), '--', 'LineWidth', 1.0);

    grid on;
    xlabel('样本序号');
    ylabel('幅度');
    title('等效总冲激响应及其翻转对比');
    legend('h\_eq', 'flipud(h\_eq)');

    subplot(2,1,2);

    sym_err = h_eq - flipud(h_eq);

    plot(sym_err, 'LineWidth', 1.0);
    grid on;
    xlabel('样本序号');
    ylabel('对称误差');
    title(sprintf('线性相位对称误差，max = %.3e', max(abs(sym_err))));

end

fprintf('============================== 完成 ==============================\n');
