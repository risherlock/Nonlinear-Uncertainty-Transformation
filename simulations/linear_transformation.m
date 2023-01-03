% Linear transformation of mean and covariance
% Rishav (2023-01-03)

clc
clear
close all

% Initial state vectors as random vectors
mu_0 = [12, 24]';             % Mean
Sigma_0 = [1, 2; 2, 4];       % Covariance matrix
n = 50000;                    % Number of samples
R = mvnrnd(mu_0, Sigma_0, n); % State vector samples

% Linear dynamcal system, x_new = A * x
A = [1, 3; 2, 1]; % State transition matrix

% Allocate memory
R_mc = zeros(size(R));
mu_mc = zeros(size(mu_0));
Sigma_mc = zeros(size(Sigma_0));
mu_as = zeros(size(mu_0));
Sigma_as = zeros(size(Sigma_0));

% Monte-Carlo simulation
R_mc = (A*R')';
mu_mc = mean(R_mc);
Sigma_mc = cov(R_mc);

% Analytical propagation
mu_as = A * mean(R)';
Sigma_as = A * Sigma_0 * A';

% Display result
printf("\t\t~~ Analytical solution ~~\r\n");
printf("Mean:\r\n");
display(mu_as');
printf("\r\nCovariance:\r\n");
display(Sigma_as);

printf("\r\n\t\t~~ Monte-Carlo simulation ~~\r\n");
printf("Mean:\r\n");
display(mu_mc);
printf("\r\nCovariance:\r\n");
display(Sigma_mc);
