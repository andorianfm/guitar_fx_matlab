function guitarFXApp()
%GUITARFXAPP Interactive guitar effects pedalboard.
%   GUITARFXAPP opens a pedal-style interface: twist the knobs and toggle
%   the stompboxes, watch the live spectrogram update, press Play to audition
%   the processed sound (a playhead sweeps the spectrogram in real time), and
%   hit Export to render the current settings to a new .wav file.
%
%   The app starts in demo mode on clean_guitar.wav. Use Load to open your
%   own mono/stereo .wav (stereo is summed to mono for processing).
%
%   All sound is produced by gfxApplyChain.m, so the export is identical to
%   what you hear.
%
%   Copyright (c) 2025. MIT License.

    appDir = fileparts(mfilename('fullpath'));

    fig = uifigure('Name', 'Guitar FX Pedalboard', ...
        'Position', [80 60 1220 840], ...
        'Color', [0.055 0.055 0.07], ...
        'Resize', 'off');

    % --- Spectrogram axes (top) ---
    ax = uiaxes(fig, 'Position', [18 502 1184 322]);
    initAxes(ax);

    % --- HTML pedalboard (bottom) ---
    h = uihtml(fig, 'Position', [12 10 1196 480]);
    h.HTMLSource = fullfile(appDir, 'pedalboard.html');
    h.HTMLEventReceivedFcn = @(src, ev) onEvent(src, ev, fig);

    % --- Shared state ---
    st = struct();
    st.appDir   = appDir;
    st.fs       = 8000;
    st.dry      = [];          % mono input
    st.wet      = [];          % processed output
    st.cfg      = struct('master', 1.0);
    st.file     = '';
    st.uihtml   = h;
    st.ax       = ax;
    st.playhead = gobjects(1);
    st.player   = [];
    st.playTimer = [];
    st.warmed   = false;
    fig.UserData = st;

    fig.CloseRequestFcn = @(s, ~) onClose(s);

    % Preload the demo clip so the spectrogram is populated immediately.
    loadSignal(fig, locateDemo(appDir), 'clean_guitar.wav (demo)');
end

function p = locateDemo(appDir)
%LOCATEDEMO Find clean_guitar.wav in the app folder or the repo root above it.
    cands = {fullfile(appDir, 'clean_guitar.wav'), ...
             fullfile(appDir, '..', 'clean_guitar.wav')};
    p = cands{1};
    for k = 1:numel(cands)
        if isfile(cands{k}), p = cands{k}; return; end
    end
end

% ======================================================================
%  Event dispatch (JavaScript -> MATLAB)
% ======================================================================

function onEvent(src, event, fig)
    name = event.HTMLEventName;
    data = event.HTMLEventData;
    try
        switch name
            case 'ParamsChanged'
                fig.UserData.cfg = data;
                process(fig);
                sendStatus(fig);
                sendEventToHTMLSource(src, 'Processed', '');

            case 'Play'
                startPlayback(fig);

            case 'Stop'
                stopPlayback(fig);

            case 'LoadDemo'
                stopPlayback(fig);
                loadSignal(fig, locateDemo(fig.UserData.appDir), ...
                    'clean_guitar.wav (demo)');
                notify(src, 'Loaded demo clip');

            case 'LoadFile'
                stopPlayback(fig);
                [f, p] = uigetfile({'*.wav;*.flac;*.ogg;*.mp3', ...
                    'Audio Files (*.wav, *.flac, *.ogg, *.mp3)'}, 'Load an audio clip');
                if isequal(f, 0)
                    notify(src, 'Load cancelled');
                    return;
                end
                loadSignal(fig, fullfile(p, f), f);
                notify(src, ['Loaded ' f]);

            case 'Export'
                exportFile(src, fig);
        end
    catch ME
        fprintf(2, 'guitarFXApp error (%s): %s\n', name, ME.message);
        notify(src, ['Error: ' ME.message]);
    end
end

% ======================================================================
%  Signal loading + processing
% ======================================================================

function loadSignal(fig, fullpath, label)
    st = fig.UserData;
    [x, fs] = audioread(fullpath);
    x = mean(double(x), 2);                 % stereo -> mono
    m = max(abs(x));
    if m > 0, x = x ./ m * 0.98; end        % normalise input headroom
    st.dry  = x;
    st.fs   = fs;
    st.file = label;
    fig.UserData = st;

    warmup(fig);                            % pay the JIT cost once, off the clock
    process(fig);
    sendStatus(fig);
end

function warmup(fig)
    % First call to gfxApplyChain JIT-compiles every effect; do it once on a
    % short slice so the first knob twist isn't sluggish.
    st = fig.UserData;
    if st.warmed || isempty(st.dry), return; end
    n = min(numel(st.dry), 2000);
    c = struct('master', 1, ...
        'wah', struct('on', true), 'flanger', struct('on', true), ...
        'phaser', struct('on', true), 'chorus', struct('on', true), ...
        'octave', struct('on', true), 'delay', struct('on', true), ...
        'reverb', struct('on', true), 'distortion', struct('on', true), ...
        'lpf', struct('on', true), 'hpf', struct('on', true));
    try, gfxApplyChain(st.dry(1:n), st.fs, c); catch, end
    st.warmed = true;
    fig.UserData = st;
end

function process(fig)
    st = fig.UserData;
    if isempty(st.dry), return; end
    st.wet = gfxApplyChain(st.dry, st.fs, st.cfg);
    fig.UserData = st;
    drawSpectrogram(fig);
    % Audio that is already playing keeps its current buffer, so the scan line
    % stays continuous and there is no audible glitch. The new settings take
    % effect the next time Play is pressed.
end

% ======================================================================
%  Spectrogram + playhead
% ======================================================================

function initAxes(ax)
    ax.Color = [0.04 0.04 0.06];
    ax.XColor = [0.65 0.65 0.72];
    ax.YColor = [0.65 0.65 0.72];
    ax.GridColor = [0.3 0.3 0.36];
    ax.Title.Color = [0.9 0.9 0.95];
    ax.Box = 'on';
    xlabel(ax, 'Time (s)');
    ylabel(ax, 'Frequency (Hz)');
    title(ax, 'Spectrogram');
end

function drawSpectrogram(fig)
    st = fig.UserData;
    ax = st.ax;
    y  = st.wet;
    fs = st.fs;
    if isempty(y), return; end

    win = 2^nextpow2(round(0.025 * fs));        % ~25 ms window
    win = min(win, 2^floor(log2(max(8, numel(y)/4))));
    nov = round(win * 0.75);
    nfft = max(1024, win * 2);

    [s, f, t] = spectrogram(y, hann(win), nov, nfft, fs);
    S = 20*log10(abs(s) + eps);
    smax = max(S(:));

    % Keep the playhead where the audio currently is, so a redraw triggered by
    % a knob tweak during playback doesn't make the scan line jump or vanish.
    curPos = 0;
    if ~isempty(st.player) && isvalid(st.player) && isplaying(st.player)
        curPos = st.player.CurrentSample / fs;
    end

    cla(ax);
    imagesc(ax, t, f, S);
    ax.YDir = 'normal';
    ax.YLim = [0 fs/2];
    ax.XLim = [0 max(t)];
    colormap(ax, turbo);
    clim(ax, [smax-70, smax]);
    title(ax, sprintf('Spectrogram  —  %s', st.file), 'Interpreter', 'none');

    st.playhead = xline(ax, curPos, 'Color', [1 1 1], 'LineWidth', 1.5, 'Alpha', 0.9);
    fig.UserData = st;
end

% ======================================================================
%  Transport (audition playback with a moving playhead)
% ======================================================================

function startPlayback(fig)
    st = fig.UserData;
    if isempty(st.wet), return; end
    stopPlayback(fig);

    st.player = audioplayer(st.wet, st.fs);
    st.player.StopFcn = @(~, ~) onPlayerStopped(fig);

    st.playTimer = timer('ExecutionMode', 'fixedSpacing', ...
        'Period', 0.04, 'BusyMode', 'drop', ...
        'TimerFcn', @(~, ~) playTick(fig));
    fig.UserData = st;

    play(st.player);
    start(st.playTimer);
    sendEventToHTMLSource(st.uihtml, 'Playing', '');
end

function playTick(fig)
    if ~isvalid(fig), return; end
    st = fig.UserData;
    if isempty(st.player) || ~isvalid(st.player) || ~isplaying(st.player), return; end
    if isgraphics(st.playhead)
        st.playhead.Value = st.player.CurrentSample / st.fs;
        drawnow limitrate;
    end
end

function onPlayerStopped(fig)
    if ~isvalid(fig), return; end
    stopPlayTimer(fig);
    st = fig.UserData;
    if isgraphics(st.playhead), st.playhead.Value = 0; end
    sendEventToHTMLSource(st.uihtml, 'Stopped', '');
end

function stopPlayback(fig)
    st = fig.UserData;
    if ~isempty(st.player) && isvalid(st.player)
        st.player.StopFcn = '';             % avoid re-entrant Stopped event
        try, stop(st.player); catch, end
    end
    st.player = [];
    fig.UserData = st;
    stopPlayTimer(fig);
    if isgraphics(st.playhead), st.playhead.Value = 0; end
    sendEventToHTMLSource(st.uihtml, 'Stopped', '');
end

function stopPlayTimer(fig)
    st = fig.UserData;
    if ~isempty(st.playTimer) && isvalid(st.playTimer)
        try, stop(st.playTimer); catch, end
        try, delete(st.playTimer); catch, end
    end
    st.playTimer = [];
    fig.UserData = st;
end

% ======================================================================
%  Export
% ======================================================================

function exportFile(src, fig)
    st = fig.UserData;
    if isempty(st.wet)
        notify(src, 'Nothing to export yet');
        return;
    end
    [base, ~] = strtok(st.file, ' (.');
    if isempty(base), base = 'guitar_fx'; end
    [f, p] = uiputfile({'*.wav', 'WAV file'}, 'Export processed audio', ...
        [base '_fx.wav']);
    if isequal(f, 0)
        notify(src, 'Export cancelled');
        return;
    end
    audiowrite(fullfile(p, f), st.wet, st.fs);
    notify(src, ['Exported ' f]);
end

% ======================================================================
%  Messaging helpers (MATLAB -> JavaScript)
% ======================================================================

function sendStatus(fig)
    st = fig.UserData;
    if isempty(st.wet), pk = 0; else, pk = max(abs(st.wet)); end
    d = struct('file', st.file, ...
        'dur', numel(st.dry) / st.fs, ...
        'fs', st.fs, ...
        'peak', pk);
    sendEventToHTMLSource(st.uihtml, 'Status', d);
end

function notify(src, msg)
    sendEventToHTMLSource(src, 'Notify', msg);
end

% ======================================================================
%  Cleanup
% ======================================================================

function onClose(fig)
    try, stopPlayback(fig); catch, end
    delete(fig);
end
