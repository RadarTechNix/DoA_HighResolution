% ===============================================================
%  FILE NAME:      MVDR.m
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
function [theta_scan, P_mvdr] = MVDR(X, M, d, lambda)
    % MVDR_DOA - Perform DOA estimation using MVDR beamforming.
    %
    % INPUTS:
    %   X      : Received signal matrix (M x N) where M is the number of elements, N is the number of snapshots
    %   M      : Number of array elements
    %   d      : Spacing between elements (in lambda units)
    %   lambda : Wavelength of the signal
    %
    % OUTPUTS:
    %   theta_scan : Angles scanned from -90° to 90°
    %   P_mvdr     : MVDR power spectrum for each angle
    
    % Estimate the covariance matrix of the received signals
    R = (X * X') / size(X, 2);  % Covariance matrix (M x M)
    
    % Initialize the steering vector and MVDR power spectrum
    theta_scan = -90:0.1:90;  % Scan angles from -90° to 90°
    P_mvdr = zeros(size(theta_scan));  % Initialize power spectrum
    
    % Compute MVDR power spectrum
    for i = 1:length(theta_scan)
        % Steering vector for the given angle theta_scan(i)
        a_theta = steering_vector(M, d, lambda, theta_scan(i));
        
        % MVDR spectrum calculation (inverse of the projection onto the signal subspace)
        P_mvdr(i) = 1 / (a_theta' * (R \ a_theta));  % \ is the matrix division (i.e., R^(-1) * a_theta)
    end
    
    % Convert to dB scale
    P_mvdr = 10 * log10(abs(P_mvdr));
end

