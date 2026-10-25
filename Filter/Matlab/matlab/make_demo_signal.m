function [x_int24, x_float, audio_info] = make_demo_signal(mp3_file, n_flush, max_audio_samples)
% 读取 MP3 音频并转换为 44.1 kHz / 24 位有符号 PCM 测试输入

    if nargin < 1 || isempty(mp3_file)
        error('make_demo_signal: 必须指定 MP3 音频文件路径。');
    end

    if nargin < 2 || isempty(n_flush)
        n_flush = 256;
    end

    if nargin < 3
        max_audio_samples = [];
    end

    target_fs = 44100;

    if ~exist(mp3_file, 'file')
        error('make_demo_signal: 找不到 MP3 文件：%s', mp3_file);
    end

    %  读取 MP3 音频
    [audio, fs_in] = audioread(mp3_file);

    if isempty(audio)
        error('make_demo_signal: 读取到的音频为空。');
    end

    %  多声道转单声道
    if size(audio, 2) > 1
        audio = mean(audio, 2);
    end

    audio = audio(:);

    %  如果 MP3 不是 44.1 kHz，则重采样到 44.1 kHz
    if fs_in ~= target_fs
        if exist('resample', 'file') ~= 2
            error(['make_demo_signal: 当前音频采样率为 %d Hz，不是 44100 Hz，' ...
                   '但未检测到 resample 函数，无法重采样。'], fs_in);
        end

        audio = resample(audio, target_fs, fs_in);
    end

    %  截取前 max_audio_samples 个有效样本

    if ~isempty(max_audio_samples)
        max_audio_samples = min(max_audio_samples, numel(audio));
        audio = audio(1:max_audio_samples);
    end

    %  防止音频幅度超过 [-1, 1]
    audio(audio >  1) =  1;
    audio(audio < -1) = -1;

    % 为了避免刚好满幅导致 24 位削顶，稍微留一点余量
    peak_val = max(abs(audio));
    if peak_val > 0.999
        audio = audio / peak_val * 0.999;
    end

    %  尾部补 0，用于冲刷 FIR / CIC / CIC 补偿 FIR 状态
    x_float = [audio; zeros(n_flush, 1)];

    %  浮点音频转换成 24 位有符号 PCM
    full_scale_pos = 2^23 - 1;
    full_scale_neg = -2^23;

    x_int24 = int64(round(x_float * full_scale_pos));

    x_int24(x_int24 >  full_scale_pos) =  full_scale_pos;
    x_int24(x_int24 <  full_scale_neg) =  full_scale_neg;

    %  输出音频信息
    audio_info = struct();
    audio_info.file_name = mp3_file;
    audio_info.original_fs_hz = fs_in;
    audio_info.target_fs_hz = target_fs;
    audio_info.valid_audio_samples = numel(audio);
    audio_info.flush_samples = n_flush;
    audio_info.total_samples = numel(x_int24);
    audio_info.duration_sec = numel(audio) / target_fs;
end