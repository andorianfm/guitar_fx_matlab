% gfx_selftest.m  -- headless validation of the effect chain backend.
clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(here);                                  % ensure gfxApplyChain is visible
wav = fullfile(here, '..', 'clean_guitar.wav'); % demo clip lives in the repo root
[x, fs] = audioread(wav);
x = mean(x, 2);                 % force mono
fprintf('Loaded %d samples (%.2f s) at %d Hz\n', numel(x), numel(x)/fs, fs);

% Build a config that turns EVERY pedal on, to exercise all code paths.
cfg = struct();
cfg.master     = 0.9;
cfg.wah        = struct('on', true, 'rate', 2,    'center', 1100, 'q', 3);
cfg.distortion = struct('on', true, 'drive', 15,  'level', 0.8);
cfg.chorus     = struct('on', true, 'depth', 0.3, 'mix', 0.5);
cfg.flanger    = struct('on', true, 'rate', 0.3,  'depth', 0.5, 'feedback', 0.45);
cfg.phaser     = struct('on', true, 'rate', 0.5,  'depth', 0.6, 'stages', 4);
cfg.octave     = struct('on', true, 'mix', 0.5);
cfg.lpf        = struct('on', true, 'cutoff', 4000);
cfg.hpf        = struct('on', true, 'cutoff', 120);
cfg.delay      = struct('on', true, 'time', 0.34, 'feedback', 0.4, 'mix', 0.35);
cfg.reverb     = struct('on', true, 'amount', 0.6, 'mix', 0.35);

tic;
y = gfxApplyChain(x, fs, cfg);
fprintf('Full chain processed in %.3f s\n', toc);
assert(numel(y) == numel(x), 'length mismatch');
assert(all(isfinite(y)), 'non-finite output');
assert(max(abs(y)) <= 1.0001, 'output exceeds full scale: %.3f', max(abs(y)));
fprintf('Full chain OK. peak=%.3f rms=%.4f\n', max(abs(y)), rms(y));

% Time each pedal individually (so we know per-knob reprocessing latency).
names = {'wah','distortion','chorus','flanger','phaser','octave','lpf','hpf','delay','reverb'};
for i = 1:numel(names)
    c = struct('master', 1.0);
    c.(names{i}) = cfg.(names{i});
    c.(names{i}).on = true;
    t0 = tic; yi = gfxApplyChain(x, fs, c); dt = toc(t0);
    assert(all(isfinite(yi)), '%s produced non-finite', names{i});
    fprintf('  %-11s %.3f s  peak=%.3f\n', names{i}, dt, max(abs(yi)));
end

% Empty config (all off) must be a clean pass-through.
yp = gfxApplyChain(x, fs, struct('master', 1.0));
assert(max(abs(yp - x)) < 1e-9, 'all-off chain is not bit-transparent');
fprintf('All-off pass-through OK\n');

outFile = fullfile(tempdir, 'selftest_output.wav');
audiowrite(outFile, y, fs);
fprintf('Wrote %s\nSELFTEST PASSED\n', outFile);
