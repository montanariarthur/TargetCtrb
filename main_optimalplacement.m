%% Example of target controllability and target observability
% for a small network system. 
%
% This code examplifies how to:
%   (1) check the target controllability and target observability of a
%       system
%   (2) apply "find_mdp" to determine the minimum set of driver nodes for
%       the target controllability of a system
%   (3) apply "find_msp" to determine the minimum set of driver nodes for
%       the target controllability of a system
%
% References:
%
%   [1] A. N. Montanari, C. Duan, Adilson E. Motter. Target controllability
%       and target observability of networks. Under review (2023).
%   [2] J. Gao, Y.-Y. Liu, R. M. DSouza, A.-L. Barabasi. Target control 
%       of complex networks. Nature Communications, 5:5415 (2014).
%   [3] A. N. Montanari, C. Duan, L. A. Aguirre, A. E. Motter. Functional
%       observability and target state estimation in large-scale networks.
%       Proceedings of the National Academy of Sciences 119:e2113750119 (2022).

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

%% Dynamical system represented in Fig. 1 in Ref. [1]: triple (A,B;F)
clear all; close all; clc
disp('-- Small network example (Fig 1 in Ref. [1]) ----------------------')

A = [0 0 0 0 0 0 0;
    1 0 0 0 0 0 0;
    1 0 0 0 0 0 0;
    0 0 1 1 0 0 1;
    0 0 1 0 0 0 0;
    0 0 0 0 1 0 0;
    0 0 0 0 0 0 0];
B = [1 0 0 0 0 0 0]';
F = [0 1 0 0 0 0 0;
    0 0 0 1 0 0 0;
    0 0 0 0 1 0 0];
n = size(A,1);              % number of nodes
p = size(B,2);              % number of drivers
r = size(F,1);              % number of targets

% Test target controllability and target observability
Atilde = A.*rand(n,n);      % numerical realization of structured matrix A
Ctrb = ctrb(Atilde,B);      % controllability matrix
Obsv = obsv(Atilde',B');    % observability matrix

% Target controllability test
if rank(F*Ctrb) == rank(F)  
    disp(['System (A,B;F) is target controllable.']);
    isTC = 1;
else
    disp(['System (A,B;F) is not target controllable.']);
    isTC = 0;
end

% Target observability test
if rank([Obsv; F]) == rank(Obsv)  
    disp(['System (B",A";F) is target observable.']);
    isTO = 1;
else
    disp(['System (B",A";F) is not target observable.']);
    isTO = 0;
end

% Minimum driver/sensor placement for target controllability/observability
B = [];                           % assume to not know neither the drivers
C = [];                           % nor the sensors
T = [2 4 5];                      % set of target nodes

% MDPt -- Ref. [2, Algorithm 3] applied to system A and functional F
D = find_mdp(A,F)'                % finds minimum driver set
p = length(D);                    % number of driver nodes
B = zeros(n,p);                   % builds input matrix
for k = 1:length(D)
    B(D(k),k) = 1;
end
Ctrb = ctrb(Atilde,B);            % controllability matrix
if rank(F*Ctrb) == rank(F)        % target controllability test
    disp(['MDPt solved for triple (A,B;F) using Algorithm 3 of Ref. 2.']);
end

% MSPt -- Ref. [3, Algorithm 1] applied to system A and functional F
S = sort(find_msp(A,T,1:n))       % finds minimum sensor set
q = length(S);                    % number of sensor nodes
C = zeros(q,n);                   % builds output matrix
for k = 1:length(S)
    C(k,S(k)) = 1;
end
Obsv = obsv(Atilde,C);            % observability matrix
if rank([Obsv; F]) == rank(Obsv)  % target observability test
    disp(['MSPt solved for triple (C,A;F) using Algorithm 1 of Ref. 3.']);
end

% MSPt -- Ref. [1, Algorithm 3] applied to the dual system A' and funct. F
S = find_mdp(A',F)'               % finds minimum sensor set
q = length(S);                    % number of sensor nodes
C = zeros(q,n);                   % builds output matrix
for k = 1:length(S)
    C(k,S(k)) = 1;
end
Obsv = obsv(Atilde,C);            % observability matrix
if rank([Obsv; F]) == rank(Obsv)  % target observability test
    disp(['Due to the strong duality, MSPt can also be solved for triple (C,A;F) using the dual Algorithm 3 of Ref. 2.']);
end

%% Minimum driver/sensor placement for target controllability/observability
clear all; close all;
disp(' ')
disp('-- Example using the C. elegans neuronal network ------------------')

% Adjacency matrix
nettype = 'celegans';
A = load([nettype,'.mat'],nettype);
A = A.(nettype);
n = size(A,1);
A = A';     % transposes so that A(i,j) corresponds to xj -> xi

% Target variables
r = round(0.05*n);                           % number of targets
T = datasample(1:n,r,'Replace',false)';     % selects random target nodes
F = zeros(r,n);                             % builds functional matrix
for i = 1:r
    F(i,T(i)) = 1;
end

% MDPt -- Ref. [2, Algorithm 3] applied to system A and functional F
D = find_mdp(A,F)';               % finds minimum driver set
p = length(D);                    % number of driver nodes
B = zeros(n,p);                   % builds input matrix
for k = 1:length(D)
    B(D(k),k) = 1;
end
Ctrb = ctrb(A,B);            % controllability matrix
if sprank(F*Ctrb) == sprank(F)        % target controllability test
    disp(['MDPt solved for the C. elegans network with ', num2str(p),' driver nodes.']);
end

% Plot
figure(1)
G = digraph(A');
nodesize = 10;
h = plot(G,'Layout','force','Iterations',7);
highlight(h,1:n,'NodeColor',0.5*[87 87 87]/255);    % states
highlight(h,1:n,'MarkerSize',6);
highlight(h,T,'NodeColor',[216 48 39]/255);     % targets
highlight(h,T,'MarkerSize',nodesize);
highlight(h,D,'NodeColor',[78 153 132]/255);    % drivers
highlight(h,D,'MarkerSize',nodesize);
h.EdgeColor = 0.3*[1 1 1];
h.LineWidth = 0.5;
