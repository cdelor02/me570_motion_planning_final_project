# Rapidly-exploring Random Trees for UAVs in an Urban Environment

### Robot Motion Planning Final Project

#### Authors: Carter Berlind, Adam Rozman, Charles DeLorey

_You can read our writeup [here](https://github.com/cdelor02/me570_motion_planning_final_project/blob/main/ME570_final_proj_writeup.pdf)_.

GitHub repository for our final project for ME570: Robot Motion Planning. Our novelty consists of an implementation of the Rapidly exploring Random Trees (RRT) algorithm, more specifically the RRT* algorithm, in 2D and in 3D. RRT and RRT* are explained below. 


![2D rrt path planning algorithm run on simple 3-obstacle environment](2D_RRT_figures/RRT_1.png "RRT")


### RRT:
Random points are found in the configuration space, and are connected to their nearest node (avoiding collisions and obstacles). This creates a random growing tree, which eventually will converge to the goal region.


### RRT*:

Based off of RRT, but differs in two key ways.
  
- Connects new nodes to the nearest neighbor with the lowest cost
- If the cost would be reduced by restructuring the tree, the tree is "rewired" to reduce that local cost

These two added features enable RRT* to converge faster on an optimal solution, as well as refine paths that are already found.

![3D rrt* path planning algorithm run on table environment](3D_RRTstar_figures/3DRRTstar_25000itrs_Iso2.png "3D RRT*")


### Required packages/software:

#### Python:
- Matplotlib
- NumPy
- NetworkX

#### MATLAB:
- UAV Toolbox
- Simulink

You should be able to get these easily using `requirements.txt`.


### Running the code

Each file should be executable individually, with a representative example contained in the `main()` for each file. 

The pipeline for generating a path based on an environment and bringing it into MATLAB for simulation is as follows:

1. Import the corresponding path planning functions from the file you are interested in. These files can be found in the `scripts` directory: 
- `me570_2D_RRT_Project.py`
- `me570_2D_RRTStar_Project.py`
- `me570_3D_RRT_Project.py`
- `me570_3D_RRTStar_Project.py`
2. Provide the planning function the necessary parameters `World, Start, End, Obstacles, Resolution, egoSize, nodes, nodePoses, tree, pathFound, itr, itr_max, s_radius`.
3. Running the algorithm yields `tree, nodes, nodePoses, path_found, final_path`.
- You can visualize the path in the environment using Matplotlib if you use the `graph()` function, also included in the algorithm files.
4. The path is saved to a `.txt` file as three columns: x, y, z.


#### To run the simulations:
1. Open the `scripts` directory and run `UAV-RRT-2D.mlx` or `UAV-RRT-3D.mlx`a program based on a live script from the MATLAB UAV Toolbox. 
2. The script will load the path data as the waypoints for the simulation, as well as the obstacle coordinates, and create and run a simulation of a quadcopter moving from the start and end positions along the given path. 




![3D rrt* path planning algorithm run on table environment 2](3D_RRTstar_figures/3drrt_sim_example.png "another 3D RRT*")


