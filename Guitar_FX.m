% Guitar_FX.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025 Andac Y. Kaya
% Licensed under the MIT License. See LICENSE file in the project root for full license information.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc

%% PARAMETERS
[guitar, fs]= audioread("clean_guitar.wav");

samp= length(guitar(:,1));
sec= samp/ fs;
time= linspace(0,sec,samp);                                                % Time vector
dT= 1/ fs;                                                                 % Sampling period
f_c_lpf= 200;                                                              % Cutoff freq
f_c_hpf= 4000;                                                             % Cutoff freq
G_lpf= 2* pi* f_c_lpf;                                                     % LPF gain
G_hpf= 2* pi* f_c_hpf;                                                     % HPF gain
a0= 0.5;                                                                   % Attenuation factor
a1= 0.3;                                                                   % Attenuation factor
d= 1500;                                                                   % Delay (in samples)
semi_t= 0.3;                                                               % Semitone shift
semi_t1= 12;                                                               % Semitone shift
semi_t2= -12;                                                              % Semitone shift
max_delay= 0.0009;                                                         % Max. delay (0.9ms)
lfo_freq= 0.25;                                                            % LFO frequency (Hz)
depth= 0.5;                                                                % Modulation depth
feedback= 0.5;                                                             % Feedback ratio
wet_mix= 0.7;                                                              % Wet signal mix ratio
dry_mix= 0.3;                                                              % Dry signal mix ratio
rate= 0.5;
stages= 4;
wah_freq= 2;                                                               % Hz— LFO speed
f_center= 1000;                                                            % Hz— Center frequency
f_range= 800;                                                              % Hz— Sweep depth
Q= 2;                                                                      % Resonance factor
LFO = sin(2*pi*rate*time);

%% CORE ALGORITHM

% FLANGER

max_delay_samples= round(max_delay* fs);
lfo= sin(2* pi* lfo_freq* time)* max_delay_samples;                        % LFO-modulated delay

flanger_guitar= zeros(size(guitar));                                       % Empty vector

for i= max_delay_samples+1:samp
    delay_samples= round(max_delay_samples* (1+ depth* lfo(i)));           % Modulated delay time
    if i- delay_samples > 0
        flanger_guitar(i)= dry_mix* guitar(i)+ wet_mix* guitar(i- delay_samples)+ feedback* flanger_guitar(i- delay_samples);
    else
        flanger_guitar(i)= guitar(i);
    end
end

% PHASER

% All-pass coefficients
a= depth* LFO;
x_ap= guitar;
for stage= 1:stages
    y_ap= zeros(size(x_ap));
    for i= 2:length(guitar)
        y_ap(i)= -a(i)* x_ap(i)+ x_ap(i-1)+ a(i)* y_ap(i-1);
    end
    x_ap= y_ap;                                                            % Chain the next stage
end

% Mix dry and processed
phaser_guitar = 0.5*guitar + 0.5*y_ap;

% DELAY

delay_guitar= zeros(length(guitar(:,1)),1);                                % Empty vector

for i= 1:length(guitar)
    if i-d>0
        delay_guitar(i)= (guitar(i)+a0* guitar(i-d));                      % Apply delay
    else
        delay_guitar(i)= (guitar(i));
    end
end

% REVERB

pre_delay= zeros(length(guitar(:,1)),1);                                   % Empty vector
reverb_guitar= zeros(length(guitar(:,1)),1);                               % Empty vector

for i= 2000:1:length(guitar)
    if i-d>0
        pre_delay(i)= guitar(i)+a0* guitar(i-d);                           % Pre-delay
        reverb_guitar(i)= guitar(i)+a1* pre_delay(i-d);                    % Wet + Dry mix
    else
        reverb_guitar(i)= guitar(i);
    end
end

% DERIVATIVE

diff_guitar= zeros(length(guitar(:,1)),1);                                 % Empty vector

for i= 2:length(guitar)
    diff_guitar(i)= (guitar(i)-guitar(i-1))/ dT;                           % Derivative
end
diff_guitar= diff_guitar/ max(abs(diff_guitar));                           % Normalization

% LPF (2nd order Butterworth using Backward Euler)

% Intermediate variables for coefficients
a0=1+ sqrt(2)*G_lpf*dT+ (G_lpf*dT)^2;
a1=-2* (1- (G_lpf*dT)^2);
a2=1- sqrt(2)*G_lpf*dT+ (G_lpf*dT)^2;

b0=(G_lpf*dT)^2;
b1=2* b0;
b2=b0;

% Normalize
b0=b0/ a0;
b1=b1/ a0;
b2=b2/ a0;
a1=a1/ a0;
a2=a2/ a0;

lpf2_guitar=zeros(size(guitar));

% For the first two samples, we can assign zero or copy:
lpf2_guitar(1)=guitar(1);
lpf2_guitar(2)=guitar(2);

for i=3:length(guitar)
    lpf2_guitar(i)=b0*guitar(i)+ b1*guitar(i-1)+ b2*guitar(i-2)- a1*lpf2_guitar(i-1)- a2*lpf2_guitar(i-2);
end

% HPF (2nd order Butterworth using Backward Euler)

% Intermediate variables for coefficients
a0=1+ sqrt(2)*G_hpf*dT+ (G_hpf*dT)^2;
a1=2* ( (G_hpf*dT)^2- 1 );
a2=1- sqrt(2)*G_hpf*dT+ (G_hpf*dT)^2;

b0=1;
b1=-2;
b2=1;

% Normalization
b0=b0/ a0;
b1=b1/ a0;
b2=b2/ a0;
a1=a1/ a0;
a2=a2/ a0;

hpf2_guitar=zeros(size(guitar));


for i=3:length(guitar)
    hpf2_guitar(i)=b0* guitar(i)+ b1* guitar(i-1)+ b2* guitar(i-2)- a1* hpf2_guitar(i-1)- a2* hpf2_guitar(i-2);
end

% CHORUS

f_modulated= shiftPitch(guitar,semi_t);                                    % Pitch-shifted version
chorus_guitar= f_modulated+guitar;                                         % Mix pitch-shifted and dry signals

% OCTAVE

f_modulated_oct1= shiftPitch(guitar,semi_t1);                              % Pitch shift +12
f_modulated_oct2= shiftPitch(guitar,semi_t2);                              % Pitch shift -12
oct_guitar= f_modulated_oct1+ guitar+ f_modulated_oct2;                    % Mix original + 2 octave shifts

% DISTORTION

dist_guitar= 100* guitar;                                                  % :D :D

% WAH (Time-Varying 2nd-Order BPF via Backward Euler)

wah_guitar = zeros(size(guitar));   % Preallocate output

% Time-varying BPF coefficients
for i = 3:length(guitar)
    % Center frequency modulated by LFO
    f0= f_center+ f_range* sin(2* pi* wah_freq* time(i));
    G_wah= 2* pi* f0;  

    % Coefficients (time-varying)
    a0= 1+ (G_wah* dT)/ Q+ (G_wah* dT)^2;
    a1= 2* ((G_wah* dT)^2- 1);
    a2= 1- (G_wah* dT)/ Q+ (G_wah* dT)^2;
    b0= (G_wah* dT)/ Q;
    b1= 0;
    b2= -b0;

    % Normalize
    b0= b0/ a0;
    b1= b1/ a0;
    b2= b2/ a0;
    a1= a1/ a0;
    a2= a2/ a0;

    % Difference equation
    wah_guitar(i)= b0* guitar(i)+ b1* guitar(i-1)+ b2* guitar(i-2)- a1* wah_guitar(i-1)- a2* wah_guitar(i-2);
end

%% DISTORTION- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, dist_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Distortion Effect');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_distortion.jpg', 'Resolution', 300);

%% REVERB- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, reverb_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Reverb Effect');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_reverb.jpg', 'Resolution', 300);

%% CHORUS- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, chorus_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Chorus Effect');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_chorus.jpg', 'Resolution', 300);

%% FLANGER- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, flanger_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Flanger Effect');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_flanger.jpg', 'Resolution', 300);

%% OCTAVER- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, oct_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Octaver Effect');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_octaver.jpg', 'Resolution', 300);

%% FLANGER- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, flanger_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Flanger Effect');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_flanger.jpg', 'Resolution', 300);

%% PHASER- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, phaser_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Phaser Effect');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_phaser.jpg', 'Resolution', 300);

%% LPF- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, lpf2_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Low-Pass Filter');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_lpf.jpg', 'Resolution', 300);

%% HPF- Plot
figure,
tiledlayout(2,1);

nexttile
plot(time, hpf2_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('High-Pass Filter');
grid on, box on,

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_hpf.jpg', 'Resolution', 300);

%% WAH- Plot
figure,
tiledlayout(2,1)

nexttile
plot(time, guitar, 'r');
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Guitar');
grid on, box on,

nexttile
plot(time, wah_guitar, 'b');
xlabel('Time (s)');
ylabel('Amplitude');
title('After Wah Effect');
grid on, box on,

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'plot_wah.jpg', 'Resolution', 300);

%% REVERB- Spectogram
figure,

win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(reverb_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Reverb');
colormap jet;

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_reverb.jpg', 'Resolution', 300);

%% CHORUS- Spectogram
figure,

win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(chorus_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Chorus');
colormap jet;

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_chorus.jpg', 'Resolution', 300);

%% DELAY- Spectogram
figure,

win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(delay_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Delay');
colormap jet;

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_delay.jpg', 'Resolution', 300);

%% OCTAVER- Spectogram
figure,

win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(oct_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Octaver');
colormap jet;

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_octaver.jpg', 'Resolution', 300);

%% DISTORTION- Spectogram
figure,

win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(dist_guitar, 1024, 900, 4096, fs, 'yaxis');
colormap jet;
title('After Distortion');

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_distortion.jpg', 'Resolution', 300);

%% FLANGER- Spectogram
figure,

win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(flanger_guitar, 1024, 900, 4096, fs, 'yaxis');
colormap jet;
title('After Flanger');

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_flanger.jpg', 'Resolution', 300);

%% PHASER- Spectogram
figure,

win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(phaser_guitar, 1024, 900, 4096, fs, 'yaxis');
colormap jet;
title('After Phaser');

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_phaser.jpg', 'Resolution', 300);

%% LPF- Spectogram
figure,
win= blackman(1024);

subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(lpf2_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Low-Pass Filter');
colormap jet;

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_lpf.jpg', 'Resolution', 300);
%% HPF- Spectogram
figure,
win= blackman(1024);

subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

subplot(2,1,2)
spectrogram(hpf2_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After High-Pass Filter');
colormap jet;

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_hpf.jpg', 'Resolution', 300);
%% WAH- Spectogram
figure,
win= blackman(1024);

subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');
win= blackman(1024);

subplot(2,1,2)
spectrogram(wah_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Wah');
colormap jet;

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);
set(gcf, 'Color', 'w');
exportgraphics(gcf, 'spectrogram_wah.jpg', 'Resolution', 300);

%% EXPORT
audiowrite('output_reverb.wav', reverb_guitar, fs);
audiowrite('output_chorus.wav', chorus_guitar, fs);
audiowrite('output_octaver.wav', oct_guitar, fs);
audiowrite('output_distortion.wav', dist_guitar, fs);
audiowrite('output_flanger.wav', flanger_guitar, fs);
audiowrite('output_delay.wav', delay_guitar, fs);
audiowrite('output_wah.wav', wah_guitar, fs);
audiowrite('output_phaser.wav', phaser_guitar, fs);
audiowrite('output_lpf.wav', lpf2_guitar, fs);
audiowrite('output_hpf.wav', hpf2_guitar, fs);


%% LISTEN
% sound(wah_guitar, fs);
