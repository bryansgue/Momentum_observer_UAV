function [vref] = dynamic_compensation_UAV(vcp, vc, u, chi_uav, K1, K2, L,ts)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
mu_l = u(1);
mu_m = u(2);
mu_n = u(3);
w = u(4);


%% Gain Matrices
K3 = K1*eye(size(u,1));
K4 = K2*eye(size(u,1));

% INERCIAL MATRIX
M = function_M(chi_uav,L);
C = function_C(chi_uav,u, L);
G = function_G();
%% CENTRIOLIS MATRIX


%% Control error veclocity
ve = vc-u;
control = vcp + K3*tanh(inv(K3)*K4*ve);
vref = M*control+C*vc+G;


end