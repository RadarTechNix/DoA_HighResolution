% ===============================================================
%  FILE NAME:      spatial_smoothing.m
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
% Function for spatial smoothing
function R_smoothed = spatial_smoothing(X, M, L)
    % SPATIAL_SMOOTHING - Applies spatial smoothing to the covariance matrix.
    %
    % INPUTS:
    %   X : Received signal matrix (M x N)
    %   M : Number of array elements
    %   L : Number of subarrays (must be <= M)
    %
    % OUTPUT:
    %   R_smoothed : Smoothed covariance matrix
    
    N = size(X, 2);  % Number of snapshots
    R_smoothed = zeros(L, L);  % Initialize smoothed covariance matrix
    
    % Sliding window over the array to form subarrays
    for i = 1:(M - L + 1)
        X_subarray = X(i:i+L-1, :);  % Extract subarray
        R_smoothed = R_smoothed + (X_subarray * X_subarray') / N;  % Accumulate covariance
    end
    
    % Normalize by the number of subarrays
    R_smoothed = R_smoothed / (M - L + 1);
end

%{ 
Why Multipath Causes Correlation
In the multipath example, although the direct path signal and the ground-reflected signal appear to come from different angles, they are related because they originate from the same source. Since the same signal is reflected along different paths, they maintain a fixed phase difference (coherence), resulting in correlation. This is what makes multipath signals coherent, unlike signals from independent targets, which are generally uncorrelated.

Spatial Smoothing to Handle Coherence
When coherent signals exist, such as in the case of multipath propagation, the MUSIC algorithm may fail because the covariance matrix does not reflect independent signals. The covariance matrix becomes rank-deficient (it doesn't have full rank), and the signal subspace cannot be properly estimated.

Spatial smoothing is a preprocessing technique used to decorrelate coherent signals, effectively allowing MUSIC to perform well even with coherent sources. It involves subdividing the antenna array into overlapping subarrays and averaging their covariance matrices to break the coherence between the signals.

Spatial Smoothing Algorithm
Divide the antenna array into several overlapping subarrays.
Compute the covariance matrix for each subarray.
Average the covariance matrices of the subarrays to obtain a new, smoothed covariance matrix.
Use this smoothed covariance matrix in the MUSIC algorithm to decorrelate the signals.

%}