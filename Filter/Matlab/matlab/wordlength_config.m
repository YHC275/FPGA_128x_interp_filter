function wl = wordlength_config()
%WORDLENGTH_CONFIG 超窄过渡带版本的受保护字长配置。
% 18 bit 系数可直接映射 DSP48E1 的 B 输入；24 bit 样本经对称预加后
% 为 25 bit，可直接映射 A/D 预加通路。48 bit FIR 累加器避免中间溢出。

    wl.stage0_coef_width   = 18;
    wl.stage0_coef_frac    = 17;
    wl.stage0_acc_width    = 48;

    wl.halfband_coef_width = 18;
    wl.halfband_coef_frac  = 17;
    wl.halfband_acc_width  = 48;

    wl.cic_acc_width       = 40;

    wl.cic_comp_coef_width = 18;
    wl.cic_comp_coef_frac  = 17;
    wl.cic_comp_acc_width  = 48;

    % 各级输出样本存储位宽；外部接口保持 24 bit
    wl.stage0_data_width    = 24;
    wl.halfband1_data_width = 24;
    wl.halfband2_data_width = 24;
    wl.cic_data_width       = 24;
    wl.cic_comp_data_width  = 24;
end
