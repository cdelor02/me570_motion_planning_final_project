# RRT* implementation for ME570 Robot Motion Planning final project
# Authors: Carter Berlind, Adam Rozman, Charles DeLorey

from networkx.classes.function import path_weight
from matplotlib.patches import Rectangle, Circle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from random import randrange
from math import sqrt
import networkx as nx
import numpy as np
import math
import time


def graph(optn, tree, nodes, nodePoses, path_found, final_path, World, Start, End, Obstacles, extra_itrs):
        
    if optn == "subf": # Plot 2 subfigures: the tree with no nodes, and the tree in our environment
        plt.figure(1, figsize=(60, 60))

        plt.subplot(211)
        nx.draw_networkx(tree, pos=nodes, with_labels=False, node_size=30)
 
        plt.subplot(212)
        endRegion =plt.Circle((End[0],End[1]),End[2],color='r')
        ax = plt.gca()
        plot_obs(ax,Obstacles)
        ax.add_patch(endRegion)
        plt.plot([0,0,World[0],World[0],0],[0,World[1],World[1],0,0])
        plt.axis("on")
        plt.axis("equal")
        end = time.time()
        #print("Time Elapsed: ",end-start)
        nx.draw_networkx_edges(tree, pos=nodes, ax=ax, node_size=30)
        if path_found==True:
            # best_path=nx.shortest_path(tree,0,final_node,weight='weight')
            path_x,path_y = path_formater(final_path,nodePoses)
            plt.plot(path_x,path_y,'r') 
            best_path_to_txt(final_path, nodePoses, "drone_path.txt")
        plt.show()

    else: # Plot just our environment
        #plt.plot(path_x,path_y,'r') #TODO
        endRegion =plt.Circle((End[0],End[1]),End[2],color='r')
        ax = plt.gca()
        plot_obs(ax,Obstacles)
        ax.add_patch(endRegion)
        plt.plot([0,0,World[0],World[0],0],[0,World[1],World[1],0,0])
        plt.axis("on")
        plt.axis("equal")
        nx.draw_networkx_edges(tree, pos=nodes, ax=ax, node_size=30)
        if path_found==True:
            # best_path=nx.shortest_path(tree,0,final_node,weight='weight')
            path_x,path_y = path_formater(final_path,nodePoses)
            plt.plot(path_x,path_y,'r') 
            best_path_to_txt(final_path, nodePoses, "drone_path.txt")
        #end = time.time()
        #print("Time Elapsed: ",end-start)
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
    ax : matplotlib.axes.ax
    obstacles : list of matplotlib.patches (Rectangle or Circle)
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
    final_nodes = [] #list of all possible
    final_path=None #lowest-cost endpoint
    path_found=False
    num_nds = 0
    start=time.time()
    while itr < itr_max:
         if itr % 1000 ==0:
             end=time.time()
             print("Iteration", itr, "of", itr_max,"time",end-start)  
		 #sample a random node
         newNode = [randrange(0,World[0]),randrange(0,World[1]),randrange(0,World[2])]
         itr += 1
         closest = -1 #stores the position in the tree of the closest node
		 #iterate through and find the closest node
         closest_distance=float("inf")
         for i in range(len(nodePoses)):
            newNode_distance = np.hypot(float(nodePoses[i][0]-newNode[0]),float(nodePoses[i][1]-newNode[1])) #math.dist
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
         if newNode[0]<0 or newNode[0]>World[0] or newNode[1]<0 or newNode[1]>World[1]:
             closest = -1
		 #add new node to my tree if plausible point
         if closest != -1:
            num_nds += 1
            nodePoses.append(newNode)
            nodes[num_nds] = (newNode[0],newNode[1]) #add new node
            #END OF TRADITIONAL RRT
            
            # if goal is reached
            if np.hypot(End[0]-newNode[0],End[1]-newNode[1]) <=  End[2]:                
                final_nodes.append(num_nds)
                path_found=True
            
            #start RRT* neighbor consideration
            if num_nds != 1:
                close_nodes = [] #neighborhood
                for node_idx in range(len(nodePoses)): #assemble neighborhood
                    if node_idx==closest or node_idx==num_nds: #exclude nearest node and new node
                        continue
                    elif np.hypot(nodePoses[node_idx][0]-newNode[0],nodePoses[node_idx][1]-newNode[1])<s_radius:
                            close_nodes.append(node_idx)      
                #start evaluating lowest cost connection
                path = nx.shortest_path(tree,0,closest,weight='weight')                   
                cost=edge_sum(tree.subgraph(path)) + closest_distance #cost initialized to cost thru nearest node             
                cheapest_node=closest #wire to nearest node if none found closer        
                if close_nodes != []: #if neighbors exist, check for cheaper connection to newnode
                    for neighbor in close_nodes:
                        path = nx.shortest_path(tree,0,neighbor,weight='weight')                   
                        new_cost=edge_sum(tree.subgraph(path))+np.hypot(nodePoses[neighbor][0]-newNode[0],
                                                                      nodePoses[neighbor][1]-newNode[1]) #change to math.dist
                        if new_cost<cost:
                            cheapest_node=neighbor
                            cost=new_cost
                tree.add_edge(cheapest_node,num_nds,weight=np.hypot(nodePoses[cheapest_node][0]-newNode[0],
                                                                    nodePoses[cheapest_node][1]-newNode[1]))
                #rewire step
                if close_nodes != []:
                    path1 = nx.shortest_path(tree,0,num_nds,weight='weight')
                    for neighbor in close_nodes: 
                        if neighbor !=cheapest_node:    
                            new_cost=edge_sum(tree.subgraph(path1))+np.hypot(nodePoses[neighbor][0]-newNode[0],
                                                                          nodePoses[neighbor][1]-newNode[1]) #cost of path to neighbor through new node  
                            path2 = nx.shortest_path(tree,0,neighbor,weight='weight') 
                            old_cost=edge_sum(tree.subgraph(path2)) #existing path to neighbor 
                            if new_cost<old_cost: #if cheaper to connect to neighbor thru new node, connect and break neighbor's previous edge
                                if path2[-2]>0: #do not remove connection to start!!!
                                    tree.remove_edge(path2[-2],neighbor)
                                    tree.add_edge(num_nds,neighbor,weight=np.hypot(nodePoses[neighbor][0]-newNode[0],
                                                                                        nodePoses[neighbor][1]-newNode[1]))
            else:
                tree.add_edge(0,1,weight=np.hypot(nodePoses[0][0]-nodePoses[1][0],
                                                                    nodePoses[0][1]-nodePoses[1][1]))
    #find final path
    if path_found == True:
        endcost=float("inf")
        for point in final_nodes: #find cheapest path to goal
            path = nx.shortest_path(tree,0,point,weight='weight')
            if edge_sum(tree.subgraph(path)) < endcost:
                final_path=path
    return tree, nodes, nodePoses, path_found, final_path
                        

def edge_sum(G):
    edges = G.edges
    total_weight = 0
    for edge in edges:
        total_weight += G[edge[0]][edge[1]]["weight"]
    return total_weight

def best_path_to_txt(path, nodePoses, fname):
    xs,ys = path_formater(path,nodePoses)
    zs=None
    for node in path:
        zs.append(0.9)
    np.savetxt(fname, np.c_[xs, ys, zs], fmt='%1.2f')

def main():
    
    World = [60,60,60]     # Map size
    Start = [2.5,1,0]     # Start coordinate
    End   = [50,55,1,2.5] # End coordinate with radius

    Resolution = 3    # Resolution for node addition step
    egoSize    = 1.5  # Buffer space around obstacles
    itr_max    = 4000 # Steps for RRT*
    s_radius   = 4    # Neighbor search radius
    
    # Initialize nodes, variables, and the tree
    start     = time.time()
    nodes     = {}
    nodes[0]  = (Start[0],Start[1])
    nodePoses = []
    nodePoses.append(Start)
    tree      = nx.Graph()
    itr       = 0
    pathFound = False #  
    plt.close('all')
    
    def cuboid_data(o, size=(1,1,1)):
        X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
             [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
             [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
             [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
             [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
             [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
        X = np.array(X).astype(float)
        for i in range(3):
            X[:,:,i] *= size[i]
        X += np.array(o)
        return X

    def plotCubeAt(positions,sizes=None,colors=None, **kwargs):
        if not isinstance(colors,(list,np.ndarray)): colors=["C0"]*len(positions)
        if not isinstance(sizes,(list,np.ndarray)): sizes=[(1,1,1)]*len(positions)
        g = []
        for p,s,c in zip(positions,sizes,colors):
            g.append( cuboid_data(p, size=s) )
        return Poly3DCollection(np.concatenate(g),  
                                facecolors=np.repeat(colors,6), **kwargs)

    Obs_pos = [[0,11,0],[40,11,0],[10,20,0],[55,20,0],[55,45,0],[10,45,0],[10,20,10],[30,30,20],[20,30,0]]
    Obs_size = [[20,4,5],[20,4,7],[5,5,10],[5,5,10],[5,5,10],[5,5,10],[50,30,10],[15,15,15],[25,15,10]] 
    colors = ["crimson","crimson","limegreen","limegreen","limegreen","limegreen","limegreen","limegreen","crimson"]
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('auto')
    
    pc = plotCubeAt(Obs_pos,Obs_size,colors=colors, edgecolor="k")
    ax.add_collection3d(pc)    
    
    ax.set_xlim([0,60])
    ax.set_ylim([0,60])
    ax.set_zlim([0,60])
    ax.plot(Start[0], Start[1], Start[2], markerfacecolor='k', markeredgecolor='k', marker='o', markersize=5, alpha=0.6)
    plt.show()
    
    # tree,nodes,nodePoses,path_found,final_path = rrt_star(World, Start, End, Obstacles, Resolution, egoSize,
    #                                                    nodes, nodePoses, tree, pathFound, itr, itr_max, s_radius)
    
    # "subf" will make it plot a figure with two subfigures: the tree, and then the tree in the environment
    # any other string will make it plot a figure with just the environment and the tree in it
    # graph("subf", tree, nodes, nodePoses, path_found, final_path, World, Start, End, Obstacles, itr_max)



if __name__ == '__main__':
	main()