% ===============================================================
%  FILE NAME:      relax.m
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
function [doa_estimates] = relax(X, M, d, lambda, K, max_iter)
% RELAX_DOA - Perform DOA estimation using the RELAX algorithm.
%
% INPUTS:
%   X      : Received signal matrix (M x N)
%   M      : Number of array elements
%   d      : Spacing between elements (in lambda units)
%   lambda : Wavelength
%   K      : Number of sources
%   max_iter: Maximum number of iterations
%
% OUTPUT:
%   doa_estimates: Estimated directions of arrival (DOAs)

% Initial guess for DOAs (you can use a coarse method like FFT to initialize)
doa_estimates = linspace(-90, 90, K); % Initial guess

for iter = 1:max_iter
    for k = 1:K
        % Estimate the contribution of source k
        v_k = steering_vector(M, d, lambda, doa_estimates(k));

        % Construct the remaining signal by subtracting all other sources
        X_residual = X;
        for j = 1:K
            if j ~= k
                v_j = steering_vector(M, d, lambda, doa_estimates(j));
                X_residual = X_residual - v_j * (v_j' * X);
            end
        end

        % Minimize the least-squares error for source k
        corr_vector = zeros(1, length(doa_estimates));
        for theta = -90:0.1:90
            v_theta = steering_vector(M, d, lambda, theta);
            corr_vector(theta + 91) = norm(v_theta' * X_residual, 2);
        end

        % Update DOA estimate for source k
        [~, max_idx] = max(corr_vector);
        doa_estimates(k) = max_idx - 91; % Update the estimate
    end
end
end

