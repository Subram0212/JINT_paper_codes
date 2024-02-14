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

### 3veh_2clusters_3clustdist_plots
This file contains the output results of the simulation performed. It represents the optimal routes of UAV and UGV for a set of mission points.

### 3veh_2clusters_3clustdist_datapts
This folder contains the input mission points for which the multi-tiered UAV-UGV optimization is needed to be carried out. The PDF file represents the mission points on a graph, whereas the spreadsheet file contains the actual input mission points that are to be fed to the program to optimize.

### requirements.txt
This file contains the necessary Python packages and its versions to be installed to run this repository.

# Steps to run the code

1. Run the code 'MCsim_3veh_2cluster_3clustdist_bounds_one.py' to perform UAV-UGV routing with Constraint Programming solver (OR-Tools).
    * The other Python functions such as K-Means_clustering, random_mission_points_generation, UGV_routing are nested into this code.
    * This code will generate the excel file of the data points, the plots of the optimal route obtained from simulation.
    * Excel files generated are: 3veh_2clusters_3clustdist_1bd_2.xlsx; Additional data for gurobi.xlsx
2. Use the generated excel files data into 'UAS_gurobi_formulation_k2.py' to perform UAV-UGV routing with MILP solver (Gurobi).
