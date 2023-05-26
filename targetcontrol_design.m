function K = targetcontrol_design(A,B,C,F,poles)
% Designs the feedback control matrix for static feedback controller 
% u(t) = K*x(t) in order to enable target control of z(t)=F*x(t). 
% The design method is based on partial pole placement, as described in
% Ref. 1, Theorem 3.
%
% Inputs:
%   A          - system matrix A (size nxn)
%   B          - input matrix B (size nxp)
%   C          - output matrix C (size qxn)
%   F          - functional matrix F  (size rxn)
%   poles      - desired partial pole placement
%
% Outputs:
%   K          - feedback control matrix
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

% System decomposition
[V,lambda,W] = eig(A);
lambda = diag(lambda);

[Abar,Bbar,Cbar,P,k] = ctrbf(A,B,C);
Fbar = F*P';
n = size(A,1);
nc = sum(k);
nu = n - nc;

[Vbar,lambdabar,Wbar] = eig(Abar);

% Controllable and uncontrollable subsystems
Ac = Abar(nu+1:n,nu+1:n);   Au = Abar(1:nu,1:nu);
Bc = Bbar(nu+1:n,:);        Bu = Bbar(1:nu,:);
Cc = Cbar(:,nu+1:n);        Cu = Cbar(:,1:nu);
Fc = Fbar(:,nu+1:n);        Fu = Fbar(:,1:nu);

Q = F'*inv(F*F')*F;
QW = Q*W;

% Partial pole placement to desired position
Kc = place(Ac,Bc,poles);
Ku = zeros(size(Kc,1),nu);
Kbar = [Ku Kc];
K = Kbar*P;                 % feedback matrix


end