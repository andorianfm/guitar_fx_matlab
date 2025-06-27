clear all
close all
clc

%% PARAMETERS
[guitar, fs]= audioread("clean_guitar.wav");

samp= length(guitar(:,1));
sec= samp/fs;
time= linspace(0,sec,samp);                                                % Time vector
dT= 1/fs;                                                                  % Sampling period
f_c= 1000;                                                                 % 1000 Hz cutoff
G= 2*pi*f_c;                                                               % LPF gain
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
 
%% FLANGER

max_delay_samples= round(max_delay * fs);
lfo= sin(2 * pi * lfo_freq * time) * max_delay_samples;                    % LFO-modulated delay

flanger_guitar= zeros(size(guitar));                                       % Empty vector

for i= max_delay_samples+1:samp
    delay_samples= round(max_delay_samples * (1 + depth * lfo(i)));        % Modulated delay time
    if i - delay_samples > 0
        flanger_guitar(i)= dry_mix * guitar(i) + wet_mix * guitar(i - delay_samples) + feedback * flanger_guitar(i - delay_samples);
    else
        flanger_guitar(i)= guitar(i);
    end
end

%% DELAY

delay_guitar= zeros(length(guitar(:,1)),1);                                % Empty vector

for i= 1:length(guitar)
    if i-d>0
        delay_guitar(i)= (guitar(i)+a0*guitar(i-d));                       % Apply delay
    else
        delay_guitar(i)= (guitar(i));
    end
end

%% REVERB
    
pre_delay= zeros(length(guitar(:,1)),1);                                   % Empty vector
reverb_guitar= zeros(length(guitar(:,1)),1);                               % Empty vector

for i= 2000:1:length(guitar)
    if i-d>0
        pre_delay(i)= guitar(i)+a0*guitar(i-d);                            % Pre-delay
        reverb_guitar(i)= guitar(i)+a1*pre_delay(i-d);                     % Wet + Dry mix
    else
        reverb_guitar(i)= guitar(i);
    end
end

%% DERIVATIVE

diff_guitar= zeros(length(guitar(:,1)),1);                                 % Empty vector

for i= 2:length(guitar)
    diff_guitar(i)= (guitar(i)-guitar(i-1))/dT;                            % Derivative
end


%% LPF

lp_guitar= zeros(length(guitar(:,1)),1);                                   % Empty vector

for i= 2:length(guitar)
    lp_guitar(i)= (1/((G*dT)+1))*(lp_guitar(i-1)+G*dT-G*diff_guitar(i-1)); % LPF using Backward Euler
end

%% CHORUS

f_modulated= shiftPitch(guitar,semi_t);                                    % Pitch-shifted version
chorus_guitar= f_modulated+guitar;                                         % Mix pitch-shifted and dry signals

%% OCTAVE

f_modulated_oct1= shiftPitch(guitar,semi_t1);                              % Pitch shift +12
f_modulated_oct2= shiftPitch(guitar,semi_t2);                              % Pitch shift -12

oct_guitar= f_modulated_oct1+guitar+f_modulated_oct2;                      % Mix original + 2 octave shifts
%% DISTORTION

dist_guitar= 100*guitar;                                                   % :D :D
%% PLAYER

audiowrite('output_reverb.wav', reverb_guitar, fs);
audiowrite('output_flanger.wav', flanger_guitar, fs);
audiowrite('output_chorus.wav', chorus_guitar, fs);
audiowrite('output_distortion.wav', dist_guitar, fs);
audiowrite('output_octaver.wav', oct_guitar, fs);
audiowrite('output_delay.wav', oct_guitar, fs);


%% DISTORTION
figure,
tiledlayout(2,1);

nexttile
plot(time, dist_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Distortion Effect');
grid on; box on;

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on; box on;


set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');

% Export as 300 DPI image
exportgraphics(gcf, 'plot_distortion.jpg', 'Resolution', 300);

%% REVERB
figure,
tiledlayout(2,1);

nexttile
plot(time, reverb_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Reverb Effect');
grid on; box on;

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on; box on;


set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');

% Export as 300 DPI image
exportgraphics(gcf, 'plot_reverb.jpg', 'Resolution', 300);

%% CHORUS
figure,
tiledlayout(2,1);

nexttile
plot(time, chorus_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Chorus Effect');
grid on; box on;

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on; box on;


set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');

% Export as 300 DPI image
exportgraphics(gcf, 'plot_chorus.jpg', 'Resolution', 300);

%% FLANGER
figure,
tiledlayout(2,1);

nexttile
plot(time, flanger_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Flanger Effect');
grid on; box on;

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on; box on;


set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');

% Export as 300 DPI image
exportgraphics(gcf, 'plot_flanger.jpg', 'Resolution', 300);

%% OCTAVER
figure,
tiledlayout(2,1);

nexttile
plot(time, oct_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Octaver Effect');
grid on; box on;

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on; box on;


set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');

% Export as 300 DPI image
exportgraphics(gcf, 'plot_octaver.jpg', 'Resolution', 300);

%% FLANGER
figure,
tiledlayout(2,1);

nexttile
plot(time, flanger_guitar, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Flanger Effect');
grid on; box on;

nexttile
plot(time, guitar, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
grid on; box on;


set(gcf, 'Units', 'inches', 'Position', [0 0 8 4]);
set(gcf, 'Color', 'w');

% Export as 300 DPI image
exportgraphics(gcf, 'plot_flanger.jpg', 'Resolution', 300);

%% REVERB
figure,
win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

win= blackman(1024);
subplot(2,1,2)
spectrogram(reverb_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Reverb');
colormap jet;
set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);   
set(gcf, 'Color', 'w');  

% Export
exportgraphics(gcf, 'spectrogram_reverb.jpg', 'Resolution', 300);
%% CHORUS
figure,
win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

win= blackman(1024);
subplot(2,1,2)
spectrogram(chorus_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Chorus');
colormap jet;
set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);   
set(gcf, 'Color', 'w');  

% Export
exportgraphics(gcf, 'spectrogram_chorus.jpg', 'Resolution', 300);

%% DELAY
figure,
win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

win= blackman(1024);
subplot(2,1,2)
spectrogram(delay_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Delay');
colormap jet;
set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);   
set(gcf, 'Color', 'w');  

% Export
exportgraphics(gcf, 'spectrogram_delay.jpg', 'Resolution', 300);
%% OCTAVE
figure,
win= blackman(1024);
subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

win= blackman(1024);
subplot(2,1,2)
spectrogram(oct_guitar, 1024, 900, 4096, fs, 'yaxis');
title('After Octaver');
colormap jet;
set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);   
set(gcf, 'Color', 'w');  

% Export
exportgraphics(gcf, 'spectrogram_octaver.jpg', 'Resolution', 300);
%% DISTORTION
figure,
win= blackman(1024);

subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

win= blackman(1024);

subplot(2,1,2)
spectrogram(dist_guitar, 1024, 900, 4096, fs, 'yaxis');
colormap jet;
title('After Distortion');

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);  
set(gcf, 'Color', 'w');  

% Export
exportgraphics(gcf, 'spectrogram_distortion.jpg', 'Resolution', 300);
%% FLANGER
figure,
win= blackman(1024);

subplot(2,1,1)
spectrogram(guitar, 1024, 900, 4096, fs, 'yaxis');
title('Original Guitar');

win= blackman(1024);

subplot(2,1,2)
spectrogram(flanger_guitar, 1024, 900, 4096, fs, 'yaxis');
colormap jet;
title('After Flanger');

set(gcf, 'Units', 'inches', 'Position', [0, 0, 8, 4]);  
set(gcf, 'Color', 'w');  

% Export
exportgraphics(gcf, 'spectrogram_flanger.jpg', 'Resolution', 300);


