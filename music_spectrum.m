% ===============================================================
%  FILE NAME:      music_spectrum.m
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
function [numSources, P_music] = music_spectrum(R, M, d, lambda, K)
    % MUSIC_SPECTRUM - Function to calculate the MUSIC spectrum for DOA estimation.
    %
    % INPUTS:
    %   R      : Covariance matrix of the received signals (M x M)
    %   M      : Number of array elements (antennas)
    %   d      : Spacing between elements in multiples of wavelength (lambda)
    %   lambda : Wavelength of the signal (normalized if needed)
    %   K      : Number of signal sources (estimated number of targets)
    %
    % OUTPUTS:
    %   theta_scan : Angles scanned by MUSIC algorithm (from -90° to 90°)
    %   P_music    : MUSIC power spectrum (spatial spectrum values in dB)
    %
    % DESCRIPTION:
    %   This function performs eigenvalue decomposition of the covariance matrix 'R'
    %   to separate the signal and noise subspaces. It then computes the MUSIC
    %   spectrum by evaluating the projection of steering vectors onto the noise
    %   subspace across the scanned angles.

    %{
 [V,D] = eig(A) produces a diagonal matrix D of eigenvalues and 
    a full matrix V whose columns are the corresponding eigenvectors  
    so that A*V = V*D. 
%}
    % Eigenvalue decomposition of covariance matrix R
    [E, D] = eig(R);                     % E is eigenvectors, D is eigenvalues
    [eigenvalues, idx] = sort(diag(D), 'descend');  % Sort eigenvalues in descending order
    E = E(:, idx);                        % Reorder eigenvectors according to eigenvalue sorting
    
    % Noise subspace: Use eigenvectors corresponding to the smallest M-K eigenvalues
    En = E(:, K+1:end);  % Noise subspace (M-K eigenvectors)

    % Define scan angles (theta from -90° to 90° in steps of 0.1°)
    theta_scan = -90:0.1:90;
    P_music = zeros(size(theta_scan));  % Initialize MUSIC spectrum array

    % Loop through scan angles and compute the MUSIC spectrum
    for i = 1:length(theta_scan)
        % Steering vector for angle theta_scan(i)
        a_theta = exp(-1j * 2 * pi * d * (0:M-1).' * sin(theta_scan(i) * pi/180));
        
        % Ensure a_theta is an (M x 1) column vector
        a_theta = a_theta(:);
        
        % MUSIC spectrum estimation (inverse projection onto noise subspace)
        P_music(i) = 1 / (abs(a_theta' * (En * En') * a_theta));
    end

    % Convert the spectrum to dB scale
    P_music = 10 * log10(P_music);

    % Determine number of peaks (sources)
    threshold = -10;
    [~, peakLocs] = findpeaks(P_music, 'MinPeakHeight', threshold);
    numSources = length(peakLocs);
end
