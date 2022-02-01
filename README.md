# JINT_paper_codes

This project has the code for a particular # of vehicles, cluster number (k) and density (rho) out of the simulation results produced in the JINT paper which is under review. This problem is formulated as a Mixed Integer Programming problem and solved using Gurobi. This problem is also solved using Constraint Programming (CP) method and OR-Tools' constraint programming solver is used for solving this problem. These results are specific to 1 UGV, 3 UAVs, 2 clusters, 3 vehicles and rho=3.

The three-tiered optimization code is as follows:

### Kmeans_clustering_2cluster_3clustdist_bounds_one.py 
Using unsupervised Machine Learning (ML) algorithm to cluster the mission points on the area into 2 clusters. Each cluster has its own cluster centroid. This is the first level out of the three-tiered optimization

### UGV_route_2cluster_3clustdist_bounds_one.py
The centroid points obtained from K-means clustering are the nodes for the UGV route, and the optimal UGV route is solved using the Miller-Tucker-Zemlin formulation of the Traveling Salesman Problem. In TSP, there are no constraints for the vehicle and likewise, UGV do not have any constraints and its route is optimized for minimum distance.

### MCsim_3veh_2cluster_3clustdist_bounds_one.py
This program performs the UAV optimization which is a Vehicle Routing Problem with time-windows, recharging and optional node constraints. This problem is solved using Constraint Programming solver. Local search heuristics are used to solve for approximate solutions. 

### UAS_gurobi_formulation_k2.py
This program solves for the UAV optimization formulated by Mixed Integer Linear Programming (MILP) Formulation. This is a Vehicle Routing Problem formulated by MILP formulation and solved using Gurobi solver. This is an exact solution solved by Branch-and-Bound algorithm.
