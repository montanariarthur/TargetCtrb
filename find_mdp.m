function D = find_mdp(A,F)  
% Finds the minimum set of driver nodes required for the system target
% controllability with respect to a set of target nodes. This is a MATLAB
% implementation of Algorithm 3 in Ref. [1], a combination of a greedy
% algorithm and maximum matching.
%
% If you use this code, please cite Refs. [1,2]. 
%
% Inputs:
%   A           - system matrix A of a dynamical system/network (nxn)
%   T           - array with target nodes' index
%   Cand        - array with candidate nodes' index for sensor placement
%   k           - maximum observable distance from sensors (default k = n)
%
% Outputs:
%   S           - array with sensor nodes' index
%
% References:
%
%   [1] J. Gao, Y.-Y. Liu, R. M. D’Souza, A.-L. Barabasi. Target control 
%       of complex networks. Nature Communications, 5:5415 (2014).
%   [2] A. N. Montanari, C. Duan, Adilson E. Motter. Target controllability
%       and target observability of structured network systems.
%       IEEE Control Systems Letters, 7:3060-3065 (2023).

% Copyright (C) 2021  Arthur Montanari
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

%   Last modified by Arthur Montanari on 16/03/2023.

G = digraph(A');                % builds inference graph
n = size(A,2);                  % number of nodes
D = [];                         % set of driver nodes (initially empty)

% Builds bipartite graph (first step, initialization)
[~,target] = find(F);           % identifies target nodes
BipG.R = target;                % right side of bipartite graph B (targets)
BipG.L = 1:n;                   % left side of bipartite graph B

% Greedy algorithm
iter = 0; 
cycles = zeros(100,n);
while ~isempty(BipG.R)
    
    % Constructs connection matrix of bipartite matrix
    BipG.G = zeros(n,length(BipG.R));
    for i = 1:length(BipG.R)
        pred = predecessors(G,BipG.R(i));
        if ~isempty(pred)
            for j = 1:length(pred)
                BipG.G(pred(j),i) = 1;
            end
        end
    end

    % Maximum matching
    nR = length(BipG.R);            % number of nodes on the right side
    Adj = [zeros(n,n) BipG.G; BipG.G' zeros(nR,nR)];   % adjacency matrix
    matches = maxmatch(Adj);        % finds matched edges
    matched_edges = matches(n+1:end)';

    % Adds unmatched nodes to the set of driver nodes
    for i = 1:length(matched_edges)
        if matched_edges(i) == 0
            D = [D; BipG.R(i)];
        end
    end
    p(iter+1) = length(D);
    
    % Set of right-matched nodes
    BipG.R = matched_edges(matched_edges ~= 0);
    
    % Breaks cycles of less than 100 nodes
    loopsize = 20;
    if iter > loopsize && sum(p(iter+1) == p(iter-loopsize:iter)) == loopsize + 1
        aux = mod(iter,100)+1;
        cycles(aux,:) = zeros(1,n);
        cycles(aux,1:length(BipG.R)) = sort(BipG.R);
        for i = 1:100
            if sum(cycles(aux,:) == cycles(i,:)) == n
                D = [D; BipG.R(1)];
                BipG.R = BipG.R(2:length(BipG.R));
            end
        end
    end
    p(iter+1) = length(D);
    iter = iter + 1;

end

% Set of driver nodes
D = unique(D);
