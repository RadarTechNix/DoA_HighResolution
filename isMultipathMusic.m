% ===============================================================
%  FILE NAME:      isMultipathMusic.m
%  AUTHOR:         Nikhil Navghade 
%  DATE CREATED:   15-Sep-2024
%  LAST MODIFIED:  15-Sep-2024
%  ORGANIZATION:   Independent Developer
%  VERSION:        1.0
%
% LICENSE: MIT License
% Permission is granted, free of charge, to use, copy, modify, and distribute
% this software for any purpose, with or without attribution. The software is
% provided "as-is" without warranty of any kind, express or implied.
%
%  ===============================================================

function [isMultipath, numDetectedSources] = isMultipathMusic(Rx, M, d, lambda, trueSources, threshold)
% analyzeSignals - Detects whether the multiple signals are due to multipath or separate sources.
%
% Syntax: [isMultipath, numDetectedSources] = analyzeSignals(Rx, M, d, lambda, trueSources, threshold)
%
% Inputs:
%    Rx - Covariance matrix of the received signals (M x M).
%    M - Number of array elements.
%    d - Spacing between elements (in lambda).
%    lambda - Wavelength.
%    trueSources - True number of sources (for comparison).
%    threshold - Threshold for peak detection to consider it significant.
%
% Outputs:
%    isMultipath - Boolean indicating whether signals are due to multipath.
%    numDetectedSources - Number of detected sources.

% Estimate the number of sources using MUSIC
[numDetectedSources, Pmusic] = music_spectrum(Rx, M, d, lambda, trueSources);

% Normalize and find peaks
% Pmusic = 10*log10(Pmusic / max(Pmusic));
peaks = find(Pmusic > threshold);

% Determine if the number of detected sources is close to the true number of sources
isMultipath = abs(numDetectedSources - trueSources) > 1;

% Output results
fprintf('Number of detected sources: %d\n', numDetectedSources);
fprintf('Detected as multipath: %d\n', isMultipath);

% Plot the MUSIC spectrum
figure;
plot(-90:0.1:90, Pmusic, 'LineWidth', 2);
xlabel('Angle (degrees)');
ylabel('MUSIC Spectrum (dB)');
title('MUSIC Spectrum Analysis');
grid on;
end