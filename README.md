# 🎸 Digital Guitar Effects in MATLAB

This project demonstrates a set of digital guitar effects implemented in pure MATLAB.  
The code processes a clean guitar signal (from a solo by my band *Buluşalım Rüyalarda- Siyah Orkide*) and applies common effects such as:

- Delay
- Reverb
- Chorus / Octaver
- Flanger
- Phaser
- Distortion
- Low-Pass Filter
- High-Pass Filter
- Wah Pedal


Each effect is implemented from scratch using basic signal processing equations — no toolboxes, plugins, or external packages were used.

---

## 🎛️ Interactive Pedalboard App

An interactive app turns these effects into a playable pedalboard. Launch it in MATLAB (R2025a+):

```matlab
addpath('app')
guitarFXApp
```

![Guitar FX Pedalboard](app/screenshot.png)

Features:
- **Load / Demo** — open your own `.wav` (stereo is summed to mono) or start from the bundled `clean_guitar.wav` demo.
- **Stompbox knobs** — drag the rotary knobs (hold **Shift** for fine control, **double-click** to reset) and tap each footswitch LED to engage a pedal. Pedals are chained in classic order: Wah → Distortion → Chorus → Flanger → Phaser → Octave → Low-Pass → High-Pass → Delay → Reverb → Master.
- **Live spectrogram** — the time–frequency view redraws as you turn knobs, with a playhead that sweeps in real time during playback.
- **Play** — audition the processed sound.
- **Export** — render the current settings to a new `.wav`. What you hear is exactly what's written.

All app code lives in [`app/`](app): the DSP in [`gfxApplyChain.m`](app/gfxApplyChain.m) (parametric versions of the `Guitar_FX.m` effects), the UI in [`pedalboard.html`](app/pedalboard.html), and the wiring in [`guitarFXApp.m`](app/guitarFXApp.m). Run [`app/gfx_selftest.m`](app/gfx_selftest.m) to validate the effect chain headlessly. The demo clip `clean_guitar.wav` stays in the repo root (shared with `Guitar_FX.m`).

---

## 🧠 How It Works

All processing is done in a single script: [`Guitar_FX.m`](Guitar_FX.m)

Simply provide a mono `.wav` file (e.g., `guitar_clean.wav`) in the same directory and run the script in MATLAB.

Each effect block:
- Applies the DSP algorithm
- Outputs the result via `sound(...)`
- Generates plots and (optionally) spectrograms

---

## 🔍 Spectrogram Visualizations

Spectrograms showing the time–frequency behavior of each effect are included in `/spectrograms`.

<p float="left">
  <img src="spectrogram_flanger.jpg" width="45%" />
  <img src="spectrogram_distortion.jpg" width="45%" />
</p>

---

## 📄 Report

A technical mini-report is available in the [`report.pdf`](report.pdf) file.  
It includes:
- DSP equations  
- Observations from each spectrogram  
- A summary of methods used

---

## ⚖️ License

This project is licensed under the MIT License — free to use, modify, and cite. Attribution appreciated!

© 2025 Andac Y. Kaya
