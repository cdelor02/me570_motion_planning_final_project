# RRT* implementation for ME570 Robot Motion Planning final project
# Authors: Carter Berlind, Adam Rozman, Charles DeLorey
# 

import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from random import randrange
import numpy as np
import math
from math import sqrt
from networkx.classes.function import path_weight
import time


def graph(World,Start,End,Obstacles,Resolution,egoSize,extra_itrs):

	# Initialize nodes, variables, and the tree
    #start     = time.time()
    #nodes     = {}
    #nodes[0]  = (Start[0],Start[1])
    #nodePoses = []
    #nodePoses.append(Start)
    #tree      = nx.Graph()
    #pathFound = False
    #itr       = 0

    #nodes,nodePoses,tree,final_node,itr = find_initial_path(World, Start, End, Obstacles, Resolution, egoSize,
       #                                                     nodes, nodePoses, tree, pathFound, itr)
       
    #path = nx.shortest_path(tree,0,final_node,weight='weight')
    #path_x,path_y = path_formater(path,nodePoses)
    #print("Initial length:",edge_sum(tree.subgraph(path)))

	# Plot your results
    #plt.plot(path_x,path_y,'g')
    #end = time.time()
    #print("Time elapsed: ",end-start)
    #for z in range(extra_itrs):
    #     if z in [extra_itrs*.1, extra_itrs*.2, extra_itrs*.3, extra_itrs*.4, extra_itrs*.5, \
    #              extra_itrs*.6, extra_itrs*.7, extra_itrs*.8, extra_itrs*.9]:
    #        path = nx.shortest_path(tree,0,final_node,weight='weight')
    #        path_x,path_y = path_formater(path,nodePoses)
    #        print("itr",z,"length:",edge_sum(tree.subgraph(path)))
	#        #plot your results
    #        plt.plot(path_x,path_y,'b')
    #        end = time.time()
    #        print("Time elapsed: ",end-start)
    #        
    #     newNode = [randrange(0,World[0]),randrange(0,World[1])]
    #     
    #     # use of Charlie's collision detection function
    #     no_obs = detect_collisions(newNode, egoSize, Obstacles)
    #     #no_obs = True
         #for Obstacle in Obstacles:
         #    if newNode[0]+egoSize>Obstacle.xy[0] and newNode[0]-egoSize<Obstacle.get_extents()._points[1][0] \
         #       and newNode[1]+egoSize>Obstacle.xy[1] and newNode[1]-egoSize<Obstacle.get_extents()._points[1][1]:
         #    #if newNode[0]+egoSize>Obstacle[0] and newNode[0]-egoSize<Obstacle[1] \
         #    #   and newNode[1]+egoSize>Obstacle[2] and newNode[1]-egoSize<Obstacle[3]:
         #       no_obs = False
            
     #    if no_obs:
     #       close_nodes = []
     #       for node_idx in range(len(nodePoses)):
     #           if np.hypot(nodePoses[node_idx][0]-newNode[0],nodePoses[node_idx][1]-newNode[1])<3:
     #               close_nodes.append(node_idx)
     #       if close_nodes != []:
     #           itr += 1
     #           nodePoses.append(newNode)
     #           nodes[itr] = (newNode[0],newNode[1])
     #           for node in close_nodes:   
     #               tree.add_edge(node,itr,weight=np.hypot(nodePoses[node][0]-nodePoses[itr][0],
           #                                                nodePoses[node][1]-nodePoses[itr][1]))

	#display options for the tree

    
	#report shortest path to terminal node
    #path = nx.shortest_path(tree,0,final_node,weight='weight')
    #print("itr", extra_itrs, "length",edge_sum(tree.subgraph(path)))
    # nx.draw_networkx(tree, nodes, **options)

	#format path for export
    path_x,path_y = path_formater(path,nodePoses)


	#plot your results
    plt.plot(path_x,path_y,'r')
    endRegion =plt.Circle((End[0],End[1]),End[2],color='r')
    ax = plt.gca()
    plot_obs(ax,Obstacles)
    ax.add_patch(endRegion)
    plt.plot([0,0,World[0],World[0],0],[0,World[1],World[1],0,0])
    plt.axis("on")
    plt.axis("equal")
    end = time.time()
    print("Time Elapsed: ",end-start)
    plt.show()


def detect_collisions(newnode, ego, Obstacles):
        """
        Takes a point, an ego (cushion space), and list of obstacles 
        (which are matplotlib.patches. Rectangle or Circle),
        checks each one for a collision with the given point. If there is
        a collision, it returns True. Otherwise, it returns False.
        """
        no_obs = True
        for Obstacle in Obstacles:
            if type(Obstacle) is Rectangle:
                if newnode[0]+ego>Obstacle.xy[0] and newnode[0]-ego<Obstacle.get_extents()._points[1][0] \
                   and newnode[1]+ego>Obstacle.xy[1] and newnode[1]-ego<Obstacle.get_extents()._points[1][1]:
                    no_obs = False
            elif type(Obstacle) is Circle:
                if math.dist(newnode, Obstacle.center) < Obstacle.radius:
                    no_obs = False
        return no_obs


def plot_obs(ax,Obstacles):
    """
    ax : takes ax 
    obstacles : list of matplotlib.patches (Rectangle and Circle)
    """

    for obs in Obstacles:
        ax.add_patch(obs)


def path_formater(path,nodePoses):
    path_x = []
    path_y = []

    for j in range(len(path)):
        path_x.append(nodePoses[path[j]][0])
        path_y.append(nodePoses[path[j]][1])
    return path_x, path_y


def rrt_star(World,Start,End,Obstacles,Resolution,egoSize,nodes,nodePoses,tree,pathFound,itr,itr_max,s_radius):
    final_nodes=[]
    num_nds=0
    while itr<itr_max:
		#sample a random node
         newNode = [randrange(0,World[0]),randrange(0,World[1])]
         itr+=1
         closest = -1 #stores the position in the tree of the closest node
		#iterate through and find the closest node
         closest_distance=float("inf")
         for i in range(len(nodePoses)):
            newNode_distance = np.hypot(float(nodePoses[i][0]-newNode[0]),float(nodePoses[i][1]-newNode[1]))
            if newNode_distance < closest_distance:
                closest = i
                closest_distance = newNode_distance
		#create a vector from the closest node in the direction of the smpled node
		#with magnitude equal to the resolution
		#This part also resets the loop if the sampled node is on top of an esisting node
         closest_point_x = nodePoses[closest][0]
         closest_point_y = nodePoses[closest][1]
         vector_x = (newNode[0] - closest_point_x)
         vector_y = (newNode[1] - closest_point_y)
         if np.hypot(vector_x,vector_y) != 0:
             unit_x = vector_x/np.hypot(vector_x,vector_y)
             unit_y = vector_y/np.hypot(vector_x,vector_y)
         else:
             closest = -1
             
         if closest != -1:
             newNode[0] = nodePoses[closest][0]+(Resolution*unit_x)
             newNode[1] = nodePoses[closest][1]+(Resolution*unit_y)
		 #check new node for obstacle collision
		 #if one is found set closest = -1
		 #do not check if the sampled node is on top of an existing node
         #no_obs = detect_collisions(newNode, egoSize, Obstacles)
         #if no_obs:
         #   closest = -1
         for obstacle in Obstacles: # **for some reason, the detect_collisions function doesn't work here, so I've left these if statements
            if type(obstacle) is Rectangle:
                if newNode[0]+egoSize>obstacle.xy[0] and newNode[0]-egoSize<obstacle.get_extents()._points[1][0] \
               and newNode[1]+egoSize>obstacle.xy[1] and newNode[1]-egoSize<obstacle.get_extents()._points[1][1]:
         #    #if newNode[0]+egoSize>Obstacle[0] and newNode[0]-egoSize<Obstacle[1] \
         #    #   and newNode[1]+egoSize>Obstacle[2] and newNode[1]-egoSize<Obstacle[3]:
                    closest = -1
            elif type(obstacle) is Circle: 
                if math.dist(newNode, obstacle.center) < obstacle.radius:
                    closest = -1
		 #add new node to my tree if plausible point
         if closest != -1:
            num_nds += 1
            nodePoses.append(newNode)
            nodes[num_nds] = (newNode[0],newNode[1]) #add new node
            # tree.add_edge(closest,num_nds,weight=np.hypot(nodePoses[closest][0]-nodePoses[num_nds][0],
            #                                           nodePoses[closest][1]-nodePoses[num_nds][1]))
            #END OF TRADITIONAL RRT
            
            # # if goal is reached
            if np.hypot(End[0]-newNode[0],End[1]-newNode[1]) <=  End[2]:                
                final_nodes = final_nodes.append(num_nds)
            
            #start RRT* neighbor consideration
            if closest != 0:
                path = nx.shortest_path(tree,0,closest,weight='weight')                   
                cost=edge_sum(tree.subgraph(path)) + closest_distance             
                cheapest_node=closest #wire to nearest node if none closer
                close_nodes = [] #neighborhood
                for node_idx in range(len(nodePoses)): #assemble neighborhood
                    if np.hypot(nodePoses[node_idx][0]-newNode[0],nodePoses[node_idx][1]-newNode[1])<s_radius and node_idx != num_nds:
                        close_nodes.append(node_idx)
                        
                if close_nodes != []: #if neighbors exist, check for cheaper connection
                    for neighbor in close_nodes:
                        # tree.add_edge(neighbor,num_nds,weight=np.hypot(nodePoses[neighbor][0]-nodePoses[num_nds][0],
                        #                                        nodePoses[neighbor][1]-nodePoses[num_nds][1]))
                        path = nx.shortest_path(tree,0,neighbor,weight='weight')                   
                        new_cost=edge_sum(tree.subgraph(path))+np.hypot(nodePoses[neighbor][0]-newNode[0],
                                                                      nodePoses[neighbor][1]-newNode[1])
                        if new_cost<cost:
                            cheapest_node=neighbor
                            cost=new_cost
                        #tree.remove_edge(neighbor, num_nds)
                tree.add_edge(cheapest_node,num_nds,weight=np.hypot(nodePoses[cheapest_node][0]-nodePoses[num_nds][0],
                                                                    nodePoses[cheapest_node][1]-nodePoses[num_nds][1]))
                if close_nodes != []:
                    for neighbor in close_nodes: 
                        if neighbor !=cheapest_node:    
                            path1 = nx.shortest_path(tree,0,num_nds,weight='weight')
                            old_cost=edge_sum(tree.subgraph(path1))+np.hypot(nodePoses[neighbor][0]-newNode[0],
                                                                          nodePoses[neighbor][1]-newNode[1])
                            # tree.add_edge(neighbor,num_nds,weight=np.hypot(nodePoses[neighbor][0]-nodePoses[num_nds][0],
                            #                                        nodePoses[neighbor][1]-nodePoses[num_nds][1]))   
                            path2 = nx.shortest_path(tree,0,neighbor,weight='weight')
                            new_cost=edge_sum(tree.subgraph(path2))
                            if old_cost<new_cost:
                                tree.remove_edge(path2[-2],neighbor)
                                tree.add_edge(neighbor,num_nds,weight=new_cost)
            else:
                tree.add_edge(0,1,weight=np.hypot(nodePoses[0][0]-nodePoses[1][0],
                                                                    nodePoses[0][1]-nodePoses[1][1]))
            
    #dist=float("inf")
    #for test in final_nodes:
        
    return nodes, nodePoses, tree, final_nodes, num_nds 
                        

def edge_sum(G):
    edges = G.edges
    total_weight = 0
    for edge in edges:
        total_weight += G[edge[0]][edge[1]]["weight"]
    return total_weight



def main():
    
    World = [60,60]     # Map size
    Start = [2.5,0]     # Start coordinate
    End   = [50,55,2.5] # End coordinate with radius

    obstacle_coords = [[0, 50, 11, 15],
                       [10, 60, 20, 30],
                       [00, 50, 40, 45]] #[x_1, x_2, y_1, y_2], aka the rectangle's 
                       # lower left corner (x_1, x_2) and upper right corner (x_2, y_2)

    Obstacles = [
    Rectangle((obstacle_coords[0][0], obstacle_coords[0][2]), 
               obstacle_coords[0][1]-obstacle_coords[0][0], obstacle_coords[0][3]-obstacle_coords[0][2]),
    Rectangle((obstacle_coords[1][0], obstacle_coords[1][2]), 
               obstacle_coords[1][1]-obstacle_coords[1][0], obstacle_coords[1][3]-obstacle_coords[1][2]),
    Rectangle((obstacle_coords[2][0], obstacle_coords[2][2]), 
               obstacle_coords[2][1]-obstacle_coords[2][0], obstacle_coords[2][3]-obstacle_coords[2][2]),
    Circle((30, 5), 3)]

    
    
    Resolution = 3    # Resolution of RRT
    egoSize    = 1.5  # Buffer space around obstacles
    itr_max = 100 #steps for RRT*
    s_radius=3 #neighbor search radius
    
    # Initialize nodes, variables, and the tree
    start     = time.time()
    nodes     = {}
    nodes[0]  = (Start[0],Start[1])
    nodePoses = []
    nodePoses.append(Start)
    tree      = nx.Graph()
    pathFound = False
    itr       = 0
    
    nodes,nodePoses,tree,final_node,num_nds = rrt_star(World, Start, End, Obstacles, Resolution, egoSize,
                                                            nodes, nodePoses, tree, pathFound, itr, itr_max, s_radius)
    
    #graph(World,Start,End,Obstacles,Resolution,egoSize,extra_itrs)

if __name__ == '__main__':
	main()