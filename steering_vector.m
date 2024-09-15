% ===============================================================
%  FILE NAME:      steering_vector.m
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
% Function to generate steering vector for a given angle
function a_theta = steering_vector(M, d, lambda, theta)
    a_theta = exp(-1j * 2 * pi * d * (0:M-1).' * sin(theta * pi / 180) / lambda);
end