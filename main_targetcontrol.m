%% Example of target control and target estimation in a closed-loop system.
%
% This code examplifies how to:
%   (1) test the output controllability and functional observability of a
%       system
%   (2) design a static feedback controller to control the target state
%       z(t) = F*x(t) with arbitrary pole placement
%   (3) design a functional observer to asymptotically estimate the target 
%       state K*x(t).
%
% In this numerical setup, the dynamical system is operating in closed loop.
% The closed-loop system is described by
%       xdot(t) = A*x(t) + B1*u(t) + B2*r
%          y(t) = C*x(t)
% where r is the constant reference signal (to be tracked) and u(t) = K*x(t) 
% is a control  signal that is estimated by a functional observer. The
% The functional observer's input is the measurement signal y(t) and the
% output is the estimated control signal u(t). See Fig. 3 in Ref. [1] for
% details.
% 
% Here, we assume that:
%    (1) matrices (A,B,C) are known
%    (2) triple (A,B;F) is output controllable and (B',A';F) is
%        functionally observable (i.e., strong duality holds)
%    (3) the triple (C,A;K) is functionally observable.
% Thus, target control of F*x  and target estimation of K*x is possible.
% 
% First, we will show how to design a static feedback control law u(t) = 
% K*x(t), that is, how to determine K such that control of the target state 
% z(t) = F*x(t) is possible with arbitrary partial pole placement. 
% 
% Second, we will show how to design a functional observer
%       wdot(t) = N*w(t) + J*y(t) + H*r
%          u(t) = D*w(t) + E*y(t)     
% such that the output u(t) of the functional observer converges to the
% sought feedback control signal K*x(t) as t -> infinity. To this end, 
% we show how to determine matrices (N,D,J,E) of the functional observer.
%
% References:
%
%   [1] A. N. Montanari, C. Duan, A. E. Motter. Duality between 
%       controllability and observability for target control and 
%       estimation in networks. Under review.

% Copyright (C) 2023  Arthur Montanari
% 
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or (at
% your option) any later version.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% General Public License for more details.
% 
% The full text of the GNU General Public License can be found in the 
% file license.txt.

%   Last modified by Arthur Montanari on 25/05/2023

clear all; close all; clc

%% Dynamical system represented in Section IV.B of Ref. [1]
A = [-1  0  0  0 -1;
     -1 -1  0  0  0;
      0  0  -3  0  0;
      0  0  0  -3  0;
      0 -1 -1 -1  -1];      % system matrix
B = [1 0 0 0 0]';           % input matrix
C = [0 0 0 0 1;
    1 0 0 0 0];             % output matrix

% Functional matrix (defining variables sought to be controlled)
F = [0 1 0 0 0];

n = size(A,1);              % number of nodes
p = size(B,2);              % number of inputs
q = size(C,1);              % number of outputs
r = size(F,1);              % number of targets

%% Output controllability
Ctrb = ctrb(A,B);      % controllability matrix

% Test output controllability
if rank(F*Ctrb) == rank(F)  
    disp('System (A,B;F) is output controllable.');
    isTC = 1;
else
    error('System (A,B;F) is not output controllable.');
    isTC = 0;
end

% Test strong duality
ObsvDual = obsv(A',B');
if rank([ObsvDual; F]) == rank(ObsvDual)
    disp('System (A,B;F) is output controllable and (B",A";F) is functionally observavble. Strong duality holds.')
    isSD = 1;
else
    isSD = 0;
end

%% Design static feedback controller for target control.
if isSD == 1
    poles = [-4 -5 -6];     % desired (partial) pole placement
    K = targetcontrol_design(A,B,C,F,poles)
    disp('Design of the static feedback controller is concluded. Target control is enabled with u(t) = -K*x(t).')
else
    error('Strong duality does not hold for (A,B;F). Target control of z = F*x is not possible via static feedback.')
end

% Reference tracking for full-state feedback
if rank([A B; F zeros(r,p)]) == n+p     % test if reference tracking is possible
    disp('Setpoint tracking is possible. Setpoint for z(t) set to 1.')
    setpoint = ones(r,1);
    G = inv([A B; F 0])*[zeros(n,p); 1];
    Mx = G(1:n,:);
    Mu = G(n+1:end,:);
    ref = (Mu + K*Mx)*setpoint;
else
    error('Setpoint tracking is not possible.')
end

%% Functional observability
Obsv = obsv(A,C);      % observability matrix

% Test functional observability
if rank([Obsv; -K]) == rank(Obsv)  
    disp('System (C,A;-K) is functionally observable.');
    isTO = 1;
else
    error('System (C,A;-K) is not functionally observable. Target estimation is not possible.');
    isTO = 0;
end

% Test Darouach's conditions for functional observer design
K0 = [-K]; 
r0 = size(K0,1);

% Darouach's condition 1
Darouach1 = (rank([C; C*A; K0; K0*A]) == rank([C; C*A; K0]));
if Darouach1 == 1
    disp('Darouach condition (1) holds.')
else
    disp('Darouach condition (1) does not hold.')
end 

% Darouach condition 2
[V,lambda,W] = eig(A);
lambda = diag(lambda);
for i = 1:n
    Darouach2(i) = (rank([lambda(i)*(K0) - (K0)*A; C*A; C]) == rank([C*A; C; K0]));
end
if sum(Darouach2) == n
    disp('Darouach condition (2) holds.')
else
    disp('Darouach condition (2) does not hold.')
end

%% Designs functional observer if both Darouach's conditions are met
if Darouach1 == 1 && sum(Darouach2) == n
    disp('Darouach conditions are satisfied for (C,A;-K).')
else
    disp('Darouach conditions were not satisfied. Functional observer design is not possible for (C,A;-K).')
    if isTO == 1
        disp('Since (C,A;-K) is functionally observable, it is possible to determine a matrix K0 (whose first rows are the rows of K) such that functional observer design is possible for (C,A;K0).')
        K0 = find_F0(A,C,-K);
    end
end

% Design of functional observer matrices
fobsv = [];
[fobsv.N,fobsv.J,fobsv.H,fobsv.D,fobsv.E,fobsv.T] = functobsv_design(A,C,K0,B);
fobsv


% The solutions below must be zero (or numerically close to zero) to
% guarantee stable target estimation.
sol17 = fobsv.N*fobsv.T + fobsv.J*C - fobsv.T*A;
sol18 = fobsv.H - fobsv.T*B;
sol19 = K0 - fobsv.D*fobsv.T - fobsv.E*C;

disp('Design of the functional observer matrices (N,J,D,E,H) is concluded. Target estimation is enabled for -K*x(t).')

%% Simulation of target control in a closed-loop system
disp('All design steps concluded. System simulation initiated.')

%% System simulation
x0 = rand(1,n+r0);              % random initial conditions
tspan = [0:0.001:10];           % time span

% Define "null observer" for the simulation scenarios in which no observer
% is considered.
nullobsv.N = zeros(size(fobsv.N)); 
nullobsv.D = zeros(size(fobsv.D)); 
nullobsv.E = zeros(size(fobsv.E)); 
nullobsv.J = zeros(size(fobsv.J)); 
nullobsv.H = zeros(size(fobsv.H)); 

% Scenario 1: open-loop system
[t,X] = ode45(@(t,X)closedloopsys(t,X,A,0,0,C,nullobsv,ref),tspan,x0);
X = X';
x_ol = X(1:n,:);           % system state in open loop

% Scenario 2: Closed-loop system with full-state feedback
[t,X] = ode45(@(t,X)closedloopsys(t,X,A-B*K,0,B,C,nullobsv,ref),tspan,x0);
X = X';
x_ff = X(1:n,:);           % system state in closed loop
u_ff = -K*x_ff;

% Scenario 3: Closed-loop system with functional observer-based feedback
[t,X] = ode45(@(t,X)closedloopsys(t,X,A,B,B,C,fobsv,ref),tspan,x0);
X = X';
x_of = X(1:n,:);                        % system state
y_of = C*x_of;                          % output
w_of = X(n+1:n+r0,:);                   % observer state
u_of = fobsv.D*w_of + fobsv.E*y_of;     % input (observer's output)

%% Plot
figure(1);
step = 300;
plot(t,F*x_ol,'-o','LineWidth',2,'Color',[0.6350 0.0780 0.1840],'MarkerIndices',1:step:length(t))
hold on
plot(t,F*x_of,'-s','LineWidth',2,'Color',[0.9290 0.6940 0.1250],'MarkerIndices',1:step:length(t))
plot(t,F*x_ff,'-x','LineWidth',2,'Color',[0 0.4470 0.7410],'MarkerIndices',1:step:length(t))
hold off
xlim([0 5])

