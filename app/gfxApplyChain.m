function y = gfxApplyChain(x, fs, cfg)
%GFXAPPLYCHAIN Apply a guitar-pedal effect chain to a mono signal.
%   y = GFXAPPLYCHAIN(x, fs, cfg) runs the column vector X (sampled at FS)
%   through a fixed pedalboard signal chain. CFG is a struct with one field
%   per pedal; each pedal field is itself a struct with an .on flag plus its
%   knob values. Missing fields fall back to sensible defaults, so a partial
%   CFG is fine.
%
%   Chain order (classic pedalboard routing):
%       wah -> distortion -> chorus -> flanger -> phaser -> octave
%           -> lpf -> hpf -> delay -> reverb -> master level
%
%   This function is the single source of truth for the sound: both the
%   interactive app and the file export call it, so what you hear is what
%   you get on disk.
%
%   Copyright (c) 2025. MIT License.

    arguments
        x  (:,1) double
        fs (1,1) double
        cfg struct = struct()
    end

    y = x;

    % --- WAH (auto-wah: LFO-swept resonant band-pass) ---
    p = pedal(cfg, 'wah');
    if p.on
        y = fx_wah(y, fs, p.rate, p.center, p.q);
    end

    % --- DISTORTION (tanh soft-clip) ---
    p = pedal(cfg, 'distortion');
    if p.on
        y = fx_distortion(y, p.drive, p.level);
    end

    % --- CHORUS (detuned doubling) ---
    p = pedal(cfg, 'chorus');
    if p.on
        y = fx_chorus(y, p.depth, p.mix);
    end

    % --- FLANGER (modulated short delay + feedback) ---
    p = pedal(cfg, 'flanger');
    if p.on
        y = fx_flanger(y, fs, p.rate, p.depth, p.feedback);
    end

    % --- PHASER (cascaded all-pass stages swept by an LFO) ---
    p = pedal(cfg, 'phaser');
    if p.on
        y = fx_phaser(y, fs, p.rate, p.depth, p.stages);
    end

    % --- OCTAVE (+/- 1 octave doubling) ---
    p = pedal(cfg, 'octave');
    if p.on
        y = fx_octave(y, p.mix);
    end

    % --- LOW-PASS FILTER ---
    p = pedal(cfg, 'lpf');
    if p.on
        y = fx_lpf(y, fs, p.cutoff);
    end

    % --- HIGH-PASS FILTER ---
    p = pedal(cfg, 'hpf');
    if p.on
        y = fx_hpf(y, fs, p.cutoff);
    end

    % --- DELAY (feedback echo) ---
    p = pedal(cfg, 'delay');
    if p.on
        y = fx_delay(y, fs, p.time, p.feedback, p.mix);
    end

    % --- REVERB (Schroeder: 4 combs + 2 all-pass) ---
    p = pedal(cfg, 'reverb');
    if p.on
        y = fx_reverb(y, fs, p.amount, p.mix);
    end

    % --- MASTER LEVEL + safety limiter ---
    master = getfield_default(cfg, 'master', 1.0);
    y = y .* master;
    peak = max(abs(y));
    if peak > 1
        y = y ./ peak;          % prevent hard clipping on export/playback
    end
    y(~isfinite(y)) = 0;        % scrub any NaN/Inf from extreme settings
end

% ======================================================================
%  Config helpers
% ======================================================================

function p = pedal(cfg, name)
%PEDAL Return the parameter struct for one pedal, merged with defaults.
    d = defaults(name);
    if isfield(cfg, name) && isstruct(cfg.(name))
        u = cfg.(name);
        fn = fieldnames(d);
        for k = 1:numel(fn)
            if isfield(u, fn{k}) && ~isempty(u.(fn{k}))
                d.(fn{k}) = double(u.(fn{k}));
            end
        end
    end
    % .on may arrive as logical, double, or be derived from defaults
    d.on = logical(d.on);
    p = d;
end

function d = defaults(name)
%DEFAULTS Per-pedal default knob values (also defines the knob set).
    switch name
        case 'wah'
            d = struct('on', false, 'rate', 2,   'center', 1000, 'q', 2);
        case 'distortion'
            d = struct('on', false, 'drive', 12, 'level', 0.8);
        case 'chorus'
            d = struct('on', false, 'depth', 0.3, 'mix', 0.5);
        case 'flanger'
            d = struct('on', false, 'rate', 0.25, 'depth', 0.5, 'feedback', 0.5);
        case 'phaser'
            d = struct('on', false, 'rate', 0.5, 'depth', 0.5, 'stages', 4);
        case 'octave'
            d = struct('on', false, 'mix', 0.6);
        case 'lpf'
            d = struct('on', false, 'cutoff', 1200);
        case 'hpf'
            d = struct('on', false, 'cutoff', 800);
        case 'delay'
            d = struct('on', false, 'time', 0.34, 'feedback', 0.4, 'mix', 0.4);
        case 'reverb'
            d = struct('on', false, 'amount', 0.6, 'mix', 0.4);
        otherwise
            d = struct('on', false);
    end
end

function v = getfield_default(s, name, def)
    if isfield(s, name) && ~isempty(s.(name))
        v = double(s.(name));
    else
        v = def;
    end
end

function v = fitlen(v, n)
%FITLEN Force a column vector to exactly N samples (pad/truncate).
    v = v(:);
    if numel(v) >= n
        v = v(1:n);
    else
        v = [v; zeros(n - numel(v), 1)];
    end
end

% ======================================================================
%  Effect implementations  (parametric versions of the originals)
% ======================================================================

function y = fx_distortion(x, drive, level)
    g = max(1, drive);
    y = tanh(g .* x);
    m = max(abs(y));
    if m > 0, y = y ./ m; end
    y = y .* level;
end

function y = fx_lpf(x, fs, fc)
    % 2nd-order Butterworth low-pass via backward Euler (matches Guitar_FX.m)
    fc = max(20, fc);
    dT = 1/fs;
    G  = 2*pi*fc;
    a0 = 1 + sqrt(2)*G*dT + (G*dT)^2;
    a1 = -2*(1 - (G*dT)^2);
    a2 = 1 - sqrt(2)*G*dT + (G*dT)^2;
    b0 = (G*dT)^2; b1 = 2*b0; b2 = b0;
    y = filter([b0 b1 b2]/a0, [1 a1/a0 a2/a0], x);
end

function y = fx_hpf(x, fs, fc)
    % 2nd-order Butterworth high-pass via backward Euler (matches Guitar_FX.m)
    fc = max(20, fc);
    dT = 1/fs;
    G  = 2*pi*fc;
    a0 = 1 + sqrt(2)*G*dT + (G*dT)^2;
    a1 = 2*((G*dT)^2 - 1);
    a2 = 1 - sqrt(2)*G*dT + (G*dT)^2;
    b0 = 1; b1 = -2; b2 = 1;
    y = filter([b0 b1 b2]/a0, [1 a1/a0 a2/a0], x);
end

function y = fx_wah(x, fs, rate, fcenter, Q)
    % Time-varying 2nd-order band-pass swept by an LFO (backward Euler).
    n   = numel(x);
    t   = (0:n-1).' / fs;
    dT  = 1/fs;
    Q   = max(0.5, Q);
    frange = 0.7 * fcenter;                  % sweep depth tied to centre
    f0  = fcenter + frange .* sin(2*pi*rate.*t);
    f0  = max(100, f0);
    y   = zeros(n, 1);
    for i = 3:n
        G  = 2*pi*f0(i);
        a0 = 1 + (G*dT)/Q + (G*dT)^2;
        a1 = 2*((G*dT)^2 - 1);
        a2 = 1 - (G*dT)/Q + (G*dT)^2;
        b0 = (G*dT)/Q;
        y(i) = (b0*x(i) - b0*x(i-2) - a1*y(i-1) - a2*y(i-2)) / a0;
    end
end

function y = fx_flanger(x, fs, rate, depth, feedback)
    n = numel(x);
    max_delay = 0.0009;                        % 0.9 ms
    md = round(max_delay * fs);
    lfo = sin(2*pi*rate*((0:n-1).'/fs)) * md;
    y = x;
    for i = md+1:n
        ds = round(md * (1 + depth*lfo(i)));
        if i - ds > 0 && ds >= 0
            y(i) = 0.3*x(i) + 0.7*x(i-ds) + feedback*y(i-ds);
        else
            y(i) = x(i);
        end
    end
    m = max(abs(y));
    if m > 1, y = y ./ m; end
end

function y = fx_phaser(x, fs, rate, depth, stages)
    n = numel(x);
    stages = max(1, round(stages));
    a = depth * sin(2*pi*rate*((0:n-1).'/fs));
    xa = x;
    ya = xa;
    for s = 1:stages
        ya = zeros(n, 1);
        for i = 2:n
            ya(i) = -a(i)*xa(i) + xa(i-1) + a(i)*ya(i-1);
        end
        xa = ya;
    end
    y = 0.5*x + 0.5*ya;
end

function y = fx_chorus(x, depth, mix)
    % Detuned doubling. DEPTH is the detune amount in semitones.
    sh = fitlen(shiftPitch(x, depth), numel(x));
    y  = (1-mix).*x + mix.*(x + sh);
    m = max(abs(y));
    if m > 1, y = y ./ m; end
end

function y = fx_octave(x, mix)
    up = fitlen(shiftPitch(x, 12),  numel(x));
    dn = fitlen(shiftPitch(x, -12), numel(x));
    y  = x + mix.*(up + dn);
    m = max(abs(y));
    if m > 1, y = y ./ m; end
end

function y = fx_delay(x, fs, t, fb, mix)
    d = max(1, round(t * fs));
    fb = min(0.95, max(0, fb));               % keep feedback stable
    wet = x;
    for i = d+1:numel(x)
        wet(i) = x(i) + fb*wet(i-d);
    end
    y = (1-mix).*x + mix.*wet;
    m = max(abs(y));
    if m > 1, y = y ./ m; end
end

function y = fx_reverb(x, fs, amount, mix)
    % Schroeder reverb: 4 parallel feedback combs + 2 series all-pass.
    g = 0.70 + 0.28*max(0, min(1, amount));
    combDelays = round([0.0297 0.0371 0.0411 0.0437] * fs);
    out = zeros(size(x));
    for k = 1:numel(combDelays)
        d = combDelays(k);
        out = out + filter(1, [1, zeros(1, d-1), -g], x);
    end
    out = out / numel(combDelays);
    apDelays = round([0.0050 0.0017] * fs);
    ga = 0.7;
    for k = 1:numel(apDelays)
        d = apDelays(k);
        out = filter([ga, zeros(1, d-1), 1], [1, zeros(1, d-1), ga], out);
    end
    m = max(abs(out));
    if m > 0, out = out ./ m; end
    y = (1-mix).*x + mix.*out;
end
