function [y, h_eq] = run_float_reference(x_float, h_stage0, h_halfband, h_cic_comp, cfg)

    x_float = x_float(:);

    y = interp2_stage_float(x_float, h_stage0);       % 44.1 kHz -> 88.2 kHz
    y = interp2_stage_float(y,       h_halfband);     % 88.2 kHz -> 176.4 kHz
    y = interp2_stage_float(y,       h_halfband);     % 176.4 kHz -> 352.8 kHz
    y = fir_comp_stage_float(y, h_cic_comp);          % 低速前置 CIC 补偿 FIR
    y = cic_interp_stage_float(y, cfg.cic_rate, cfg.cic_stages); % 352.8 kHz -> 5.6448 MHz

    % 等效总冲激响应
    if nargout >= 2
        h_eq = build_equivalent_filter(h_stage0, h_halfband, h_cic_comp, cfg);
    end
end

function y = interp2_stage_float(x, h)
% 与 2x 多相 FIR 结构对应的浮点参考模型

    x = x(:);
    h = h(:);

    h_even = h(1:2:end);
    h_odd  = h(2:2:end);

    delay_line = zeros(numel(h_even), 1);
    y = zeros(2 * numel(x), 1);
    p = 1;

    for n = 1:numel(x)
        delay_line = [x(n); delay_line(1:end-1)]; 
        y(p)     = sum(delay_line .* h_even);
        y(p + 1) = sum(delay_line(1:numel(h_odd)) .* h_odd);
        p = p + 2;
    end
end

function y = cic_interp_stage_float(x, rate, stages)
% CIC 插值器浮点参考模型

    if nargin < 2 || isempty(rate),   rate = 16; end
    if nargin < 3 || isempty(stages), stages = 3; end

    x = x(:);
    comb_state = zeros(stages, 1);
    int_state  = zeros(stages, 1);
    y = zeros(numel(x) * rate, 1);
    p = 1;
    gain = rate ^ (stages - 1);

    for n = 1:numel(x)
        v = x(n);

        % 低速 comb 部分
        for k = 1:stages
            d = v - comb_state(k);
            comb_state(k) = v;
            v = d;
        end

        % 高速 integrator 部分，每个输入样本产生 rate 个输出样本
        for r = 1:rate
            if r == 1
                drive = v;
            else
                drive = 0;
            end

            int_state(1) = int_state(1) + drive;
            for k = 2:stages
                int_state(k) = int_state(k) + int_state(k-1);
            end

            y(p) = int_state(stages) / gain;
            p = p + 1;
        end
    end
end

function y = fir_comp_stage_float(x, h)
% CIC 补偿 FIR 的浮点参考模型

    x = x(:);
    h = h(:);
    delay_line = zeros(numel(h), 1);
    y = zeros(numel(x), 1);

    for n = 1:numel(x)
        delay_line = [x(n); delay_line(1:end-1)]; 
        y(n) = sum(delay_line .* h);
    end
end

function h_eq = build_equivalent_filter(h_stage0, h_halfband, h_cic_comp, cfg)
% 等效链路：  Stage0(2x) -> HB(2x) -> HB(2x) -> 低速 CIC 补偿 FIR -> CIC(16x)
    h_stage0   = h_stage0(:);
    h_halfband = h_halfband(:);
    h_cic_comp = h_cic_comp(:);

    h_cic = build_cic_impulse_response(cfg.cic_rate, cfg.cic_stages);

    h_eq = conv(upsample_coeff(h_stage0, 64), ...
                upsample_coeff(h_halfband, 32));    

    h_eq = conv(h_eq, upsample_coeff(h_halfband, 16));
    h_eq = conv(h_eq, upsample_coeff(h_cic_comp, 16));
    h_eq = conv(h_eq, h_cic);
    h_eq = h_eq(:);
end

function h_cic = build_cic_impulse_response(rate, stages)
% 用浮点 CIC 模型构造归一化 CIC 冲激响应

    if nargin < 1 || isempty(rate),   rate = 16; end
    if nargin < 2 || isempty(stages), stages = 3; end

    x_imp = [1; zeros(stages + 8, 1)];
    y_imp = cic_interp_stage_float(x_imp, rate, stages);

    idx = find(abs(y_imp) > 1e-12, 1, 'last');
    if isempty(idx)
        h_cic = 0;
    else
        h_cic = y_imp(1:idx);
    end

    h_cic = h_cic(:);
end

function y = upsample_coeff(h, factor)
% 把低采样率滤波器系数放到最终输出采样率时间轴上

    h = h(:);

    if factor <= 1
        y = h;
        return;
    end

    y = zeros((numel(h) - 1) * factor + 1, 1);
    y(1:factor:end) = h;
end
