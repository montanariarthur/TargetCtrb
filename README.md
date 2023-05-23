# Target Controllability and Target Observability
Codes for minimum driver/sensor placement and controller/observer design for target control and target estimation in large-scale networks.

# Usage

The


The following codes are direct implementations of the algorithms for minimum sensor placement and minimum driver placement for target observability and target controllability problems, respectively.

- `find_msp` : Finds the minimum set of sensor nodes required for the system's target observability (a.k.a. functional observability) with respect to a set of target nodes. This is a MATLAB implementation of Algorithm 1 in Ref. [3].

- `find_mdp` : Finds the minimum set of driver nodes required for the system's target controllability (a.k.a. output controllability) with respect to a set of target nodes. This is a MATLAB implementation of Algorithm 3 in Ref. [4]. 


The following codes are design algorithms for functional observers in target estimation applications as well as for static feedback controllers in target control applications.

- `find_F0` : Finds F0 with minimum-order such that Darouach's condition (4) in Ref. [3] is satisfied for a triple (A,C,F0). This is the first step in the design of a functional observer. This is a MATLAB implementation of Algorithm 2 in Ref. [3].

- `functobsv_design` : Designs the functional observer's matrices (N,J,H,D,E) in Eq. (10) of Ref. [1]. The design method is guaranteed to provide a stable functional observer if the triple (A,C,F0) satisfies conditions (4-5) in Ref. [1].




The following examples illustrate numerical results of the algorithms described above for small numerical examples as well as complex networks.

- `example_dynamicalnetwork` : Example of minimum sensor placement and minimum-order functional observer design for a random complex dynamical networks. This code examplifies how to:
    1. apply Algorithm 1 to determine the minimum set of sensor nodes for functional observability of a dynamical network with respect to a given set of target nodes; and
    2. apply Algorithm 2 to design a minimum-order functional observer.

- `example_cyberdetection` : Example of cyber-attack detection in power grids using functional observers and target state estimation.

- `example_epidemicspreading` : Example of sensor placement and nonlinear functional observer design for estimating the prevalence rate of infection in target populations following an epidemic outbreak.




The above examples are dependent on the following codes:

- `spnull`,`sporth` :  Computes a sparse orthonormal basis for the null space and the range space of a matrix, respectively.



# References
1. A. N. Montanari, C. Duan, A. E. Motter. Target controllability and target observability of networks. *Under review* (2023).
2. A. N. Montanari, C. Duan, L. A. Aguirre, A. E. Motter. Functional observability and target state estimation in large-scale networks. *Proceedings of the National Academy of Sciences* 119(1):e2113750119 (**2022**). 
https://doi.org/10.1073/pnas.2113750119
