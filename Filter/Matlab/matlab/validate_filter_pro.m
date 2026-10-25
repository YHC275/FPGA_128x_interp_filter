function resp = validate_filter_pro()
%VALIDATE_FILTER_PRO Rebuild the quantized design in memory and check the spec.
% This function does not overwrite coefficient or RTL export files.

    this_dir = fileparts(mfilename('fullpath'));
    export_dir = fullfile(fileparts(this_dir), 'rtl_export');

    [~, ~, ~, q_stage0, q_halfband, q_cic_comp, cfg, info] = ...
        design_all_filters(export_dir, false, false, false);
    resp = info.final_response;

    assert(numel(q_stage0) == 2047, 'Stage0 coefficient count is not 2047.');
    assert(numel(q_halfband) == 95, 'Halfband coefficient count is not 95.');
    assert(numel(q_cic_comp) == 31, 'CIC compensation coefficient count is not 31.');
    assert(resp.passband_ok, 'Passband ripple exceeds +/-0.05 dB.');
    assert(resp.stopband_ok, 'Stopband attenuation is below 70 dB.');
    assert(resp.linear_phase_ok, 'Equivalent impulse response is not symmetric.');
    assert(resp.overall_ok, 'Filter_Part_Pro frequency-response validation failed.');

    fprintf('\nFilter_Part_Pro validation PASS\n');
    fprintf('  Input/output Fs       : %.1f kHz -> %.4f MHz\n', ...
        cfg.input_fs_hz / 1e3, cfg.output_fs_hz / 1e6);
    fprintf('  Passband              : %.0f Hz .. %.0f Hz\n', ...
        cfg.spec.passband_hz(1), cfg.spec.passband_hz(2));
    fprintf('  Stopband starts       : %.0f Hz\n', cfg.spec.stopband_start_hz);
    fprintf('  Passband max deviation: %.9f dB\n', resp.pass_ripple_pm_db);
    fprintf('  Worst stop attenuation: %.9f dB\n', resp.stop_attn_db);
    fprintf('  Gain at stop edge     : %.9f dB\n', resp.gain_stop_edge_db);
    fprintf('  Gain at 22.05 kHz     : %.9f dB\n', resp.gain_22050hz_db);
    fprintf('  Symmetry max error    : %.3e\n', resp.symmetry_max_err);
end
