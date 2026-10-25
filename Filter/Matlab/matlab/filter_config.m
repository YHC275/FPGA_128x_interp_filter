function cfg = filter_config()
% 44.1 kHz / 128x 超窄过渡带 RTL 链路参数。
% 所有 MATLAB、RTL 和验收脚本必须从本文件读取规格，禁止再次硬编码
% 20 kHz、20.175 kHz 或 70 dB 等频响指标。
%   输入采样率       : 44.1 kHz
%   插值倍率         : 128x
%   512Fs 主时钟     : 22.5792 MHz
%   输出采样率       : 5.6448 MHz

    cfg = struct();

    % 采样率与时钟
    cfg.input_fs_hz        = 44100;
    cfg.interp_factor      = 128;
    cfg.output_fs_hz       = cfg.input_fs_hz * cfg.interp_factor;
    cfg.clk_audio_512fs_hz = cfg.input_fs_hz * 512;

    % 全链路频响规格。设计器以 20.175 kHz 为目标阻带边缘；在量化后的
    % 全 128x 频响上，从该频率直至输出 Nyquist 的任意镜像均需满足阻带。
    cfg.spec.passband_hz          = [10 20000];
    cfg.spec.passband_pm_db       = 0.05;
    cfg.spec.design_stop_hz       = 20175;
    cfg.spec.stopband_start_hz    = 20175;
    cfg.spec.stopband_attn_db     = 70;
    cfg.spec.design_stop_weight   = 30;
    cfg.spec.analysis_nfft        = 2^20;
    cfg.spec.linear_phase_tol     = 1e-10;

    % RTL 定点位宽
    cfg.in_width  = 24;
    cfg.out_width = 24;

    wl = wordlength_config();
    cfg.stage0_coef_width   = wl.stage0_coef_width;
    cfg.stage0_coef_frac    = wl.stage0_coef_frac;
    cfg.stage0_acc_width    = wl.stage0_acc_width;
    cfg.halfband_coef_width = wl.halfband_coef_width;
    cfg.halfband_coef_frac  = wl.halfband_coef_frac;
    cfg.halfband_acc_width  = wl.halfband_acc_width;
    cfg.cic_acc_width       = wl.cic_acc_width;
    cfg.cic_comp_coef_width = wl.cic_comp_coef_width;
    cfg.cic_comp_coef_frac  = wl.cic_comp_coef_frac;
    cfg.cic_comp_acc_width  = wl.cic_comp_acc_width;
    cfg.stage0_data_width   = wl.stage0_data_width;
    cfg.halfband1_data_width = wl.halfband1_data_width;
    cfg.halfband2_data_width = wl.halfband2_data_width;
    cfg.cic_data_width       = wl.cic_data_width;
    cfg.cic_comp_data_width  = wl.cic_comp_data_width;

    % 满足通带、阻带和线性相位指标约束的混合字长搜索设置
    cfg.wordlength.frac_candidates   = 10:16;
    cfg.wordlength.search_nfft       = 2^18;
    cfg.wordlength.passband_guard_db = 0.045;
    % 低速前置补偿无法在 8Fs 域区分 20 kHz 基带与 352.8 kHz 附近镜像，
    % 该镜像主要由三阶 CIC 抑制；量化设计值约 72 dB，仍高于 70 dB 核心指标
    cfg.wordlength.stopband_guard_db = 74;
    cfg.wordlength.cic_guard_bits    = 0;
    % 级间样本只允许逐级保持或缩短，外部输入/输出仍保持 24 bit
    cfg.wordlength.data_width_candidates = 24;
    % 搜索阶段使用相对 24 bit 满量程的误差噪声和峰值误差快速筛选
    cfg.wordlength.screen_min_error_snr_dbfs = 115;
    cfg.wordlength.screen_max_abs_error_lsb  = 96;
    % 最终压力测试采用更长的满幅、交替和随机序列复核。
    cfg.wordlength.min_output_snr_db      = 105;
    cfg.wordlength.max_abs_error_lsb      = 128;
    cfg.wordlength.baseline.stage0_coef_width   = 18;
    cfg.wordlength.baseline.stage0_coef_frac    = 17;
    cfg.wordlength.baseline.stage0_acc_width    = 48;
    cfg.wordlength.baseline.halfband_coef_width = 18;
    cfg.wordlength.baseline.halfband_coef_frac  = 17;
    cfg.wordlength.baseline.halfband_acc_width  = 48;
    cfg.wordlength.baseline.cic_acc_width       = 40;
    cfg.wordlength.baseline.cic_comp_coef_width = 18;
    cfg.wordlength.baseline.cic_comp_coef_frac  = 17;
    cfg.wordlength.baseline.cic_comp_acc_width  = 48;
    cfg.wordlength.baseline.stage0_data_width    = 24;
    cfg.wordlength.baseline.halfband1_data_width = 24;
    cfg.wordlength.baseline.halfband2_data_width = 24;
    cfg.wordlength.baseline.cic_data_width       = 24;
    cfg.wordlength.baseline.cic_comp_data_width  = 24;

    % 顶层参数
    cfg.fifo_depth         = 32;
    cfg.output_align_delay = 192;

    % 2x FIR 参数
    cfg.stage0_ph0_taps = 1024;
    cfg.stage0_ph1_taps = 1023;
    cfg.stage0_mac_lanes = 4;
    cfg.hb_ph0_taps     = 48;
    cfg.hb_ph1_taps     = 47;

    % CIC 与低速前置 CIC 补偿 FIR 参数
    cfg.cic_rate       = 16;
    cfg.cic_stages     = 4;
    cfg.cic_gain_shift = 12;   % 16^(4-1) = 4096 = 2^12
    cfg.cic_comp_taps  = 31;

    % 512Fs 时钟域下的固定节拍关系
    cfg.slot_div_512_to_128 = 4;  % 每 4 个 512Fs 时钟产生 1 个 128Fs 槽位
    cfg.rate_slots_per_input = 128;
end
