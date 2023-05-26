function Xdot = closedloopsys(t,X,A,B1,B2,C,fobsv,ref)
% Ordinary differential equations for a closed-loop system with functional 
% observer-based output feedback. In this case, the dynamical system is
% driven by an input signal u(t) = Kx(t). The control signal u(t) is, in 
% turn, the output signal of a functional observer that estimates uhat(t)
% using the measurement signal y(t) of the system. See full description in
% "main_closedloopsystem".
%
% Inputs:
%   t           - time
%   X           - state vector (nx1) 
%   A           - system matrix (nxn)
%   B1          - input matrix of control signal u(t) (nxp)
%   B2          - input matrix of reference signal r (nxp)
%   C           - output matrix (qxn)
%   fobsv       - structure containing the functional observer's matrices
%                 (N,J,D,E,H)
%   ref         - reference signal
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

% System state
x(:,1) = X(1:size(A,1));
w(:,1) = X(size(A,1)+1:end);
p = size(B1,2);

% System equations
y = C*x;                    % measurement equation
u = fobsv.D*w + fobsv.E*y;  % control signal (functional observer's output)

xdot = A*x + B1*u(1:p,1) + B2*ref;             % dynamical system ODE
wdot = fobsv.N*w + fobsv.J*y + fobsv.H*ref;    % functional observer ODE

% System derivatives
Xdot = [xdot; wdot];

end