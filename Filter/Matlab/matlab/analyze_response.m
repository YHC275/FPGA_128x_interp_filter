function resp = analyze_response(h_eq, cfg)
% 分析等效滤波器指标

    h_eq = h_eq(:);
    fs_out = cfg.output_fs_hz;
    nfft = cfg.spec.analysis_nfft;

    H = fft(h_eq, nfft);
    H = H(1:nfft/2 + 1);
    f = (0:nfft/2).' / nfft * fs_out;

    dc_gain = abs(H(1));
    if dc_gain < 1e-15
        Hn = H;
    else
        Hn = H / dc_gain;
    end

    mag = abs(Hn);
    mag_db = 20 * log10(max(mag, 1e-14));

    spec = cfg.spec;

    pass_idx = f >= spec.passband_hz(1) & f <= spec.passband_hz(2);
    stop_idx = f >= spec.stopband_start_hz;

    pass_db = mag_db(pass_idx);
    stop_mag = mag(stop_idx);

    resp = struct();
    resp.fs_out = fs_out;
    resp.f_hz = f;
    resp.mag_db = mag_db;
    resp.dc_gain = dc_gain;
    resp.spec = spec;

    resp.gain_10hz_db  = interp1(f, mag_db, 10,    'linear', 'extrap');
    resp.gain_20khz_db = interp1(f, mag_db, 20000, 'linear', 'extrap');
    resp.gain_stop_edge_db = interp1(f, mag_db, spec.stopband_start_hz, 'linear', 'extrap');
    resp.gain_22050hz_db = interp1(f, mag_db, cfg.input_fs_hz / 2, 'linear', 'extrap');
    resp.gain_24100hz_db = interp1(f, mag_db, cfg.input_fs_hz - spec.passband_hz(2), 'linear', 'extrap');
    resp.pass_max_db = max(pass_db);
    resp.pass_min_db = min(pass_db);
    resp.pass_ripple_pp_db = resp.pass_max_db - resp.pass_min_db;
    resp.pass_ripple_pm_db = max(abs(pass_db));
    resp.passband_ok = all(pass_db <= spec.passband_pm_db) && all(pass_db >= -spec.passband_pm_db);

    resp.stop_attn_db = -20 * log10(max(stop_mag));
    resp.stopband_ok = resp.stop_attn_db >= spec.stopband_attn_db;

    resp.symmetry_max_err = max(abs(h_eq - flipud(h_eq)));
    resp.linear_phase_tol = spec.linear_phase_tol;
    resp.linear_phase_ok = resp.symmetry_max_err <= resp.linear_phase_tol;

    resp.overall_ok = resp.passband_ok && resp.stopband_ok && resp.linear_phase_ok;
end
