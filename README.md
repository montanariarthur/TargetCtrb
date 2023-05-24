# Target Controllability and Target Observability
Codes for minimum driver/sensor placement and controller/observer design for target control and target estimation in large-scale networks.


# Usage

The following codes are direct implementations of the algorithms for minimum sensor placement and minimum driver placement for target observability and target controllability problems, respectively.

- `find_msp` : Finds the minimum set of sensor nodes required for the system's target observability (a.k.a. functional observability) with respect to a set of target nodes. This is a MATLAB implementation of Algorithm 1 in Ref. [2].

- `find_mdp` : Finds the minimum set of driver nodes required for the system's target controllability (a.k.a. output controllability) with respect to a set of target nodes. This is a MATLAB implementation of Algorithm 3 in Ref. [3]. 



The following codes are design algorithms of functional observers (for target estimation applications) and static feedback controllers (for target control applications).

- `find_F0` : Finds F0 with minimum-order such that Darouach's condition (4) in Ref. [3] is satisfied for a triple (A,C,F0). This is the first step in the design of a functional observer. This is a MATLAB implementation of Algorithm 2 in Ref. [2].

- `functobsv_design` : Designs the functional observer's matrices (N,J,H,D,E) in Eq. (10) of Ref. [2]. The design method is guaranteed to provide a stable functional observer if the triple (A,C,F0) satisfies conditions (4-5) in Ref. [2].




The following examples illustrate numerical results of the algorithms described above for a small numerical examples as well as complex networks like the *C. elegans* neuronal network.

- `example_dynamicalnetwork` : Example of minimum sensor placement and minimum-order functional observer design for a random complex dynamical networks. This code examplifies how to:
    1. apply Algorithm 1 to determine the minimum set of sensor nodes for functional observability of a dynamical network with respect to a given set of target nodes; and
    2. apply Algorithm 2 to design a minimum-order functional observer.

- `example_cyberdetection` : Example of cyber-attack detection in power grids using functional observers and target state estimation.

- `example_epidemicspreading` : Example of sensor placement and nonlinear functional observer design for estimating the prevalence rate of infection in target populations following an epidemic outbreak.




The above codes are dependent on the following codes and datasets:

- `spnull`,`sporth` :  Computes a sparse orthonormal basis for the null space and the range space of a matrix, respectively.
- `maxmatch` : Finds the maximum matching of a bipartite graph.
- `celegans.mat` : Adjacency matrix of the *C. elegans* neuronal network.


# References
1. A. N. Montanari, C. Duan, A. E. Motter. Target controllability and target observability of networks. *Under review* (2023).
2. A. N. Montanari, C. Duan, L. A. Aguirre, A. E. Motter. Functional observability and target state estimation in large-scale networks. *Proceedings of the National Academy of Sciences* 119(1):e2113750119 (**2022**). 
https://doi.org/10.1073/pnas.2113750119
3. J. Gao, YY. Liu , R. D'Souza, AL. Barabási. Target control of complex networks. *Nature Communications* 5, 5415 (**2014**). https://doi.org/10.1038/ncomms6415
