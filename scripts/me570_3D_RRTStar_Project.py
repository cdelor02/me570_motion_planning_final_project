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

def graph(optn, tree, nodes, nodePoses, path_found, final_path, World, Start, End, Obs_pos, Obs_size,colors, extra_itrs):  
        #functions for plotting 3D polygons using mplot3d Poly3DCollection
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
            return Poly3DCollection(np.concatenate(g), facecolors=np.repeat(colors,6), **kwargs)
        fig = plt.figure()
        ax=plt.axes(projection='3d')
        ax.set_aspect('auto')
        #plot obstacles
        pc = plotCubeAt(Obs_pos,Obs_size,colors=colors, edgecolor="k", alpha=0.2)
        ax.add_collection3d(pc)  
        #plot start and end points
        ax.plot(Start[0], Start[1], Start[2], markerfacecolor='k', markeredgecolor='k', marker='o', markersize=5, alpha=0.6) 
        ax.plot(End[0], End[1], End[2], markerfacecolor='r', markeredgecolor='r', marker='o', markersize=5, alpha=0.6)
        # plot whole tree nodes and edges (toggleable)
        if optn == True:
            for j in range(len(final_path)):
                x=(nodePoses[final_path[j]][0])
                y=(nodePoses[final_path[j]][1])
                z=(nodePoses[final_path[j]][2])
                ax.scatter(x,y,z,marker='o',alpha=0.9)      
            #xi,yi,zi=tree_formatter(nodes,nodePoses)
            #ax.scatter(xi,yi,zi,marker='o',alpha=0.9)     
            for edge in tree.edges:
                edge_xyz=( [ nodes[edge[0]], nodes[edge[1]] ] )
                ax.plot([edge_xyz[0][0], edge_xyz[1][0]],[edge_xyz[0][1], edge_xyz[1][1]], [edge_xyz[0][2], edge_xyz[1][2]], 'k-')
        #plot path if found
        if path_found==True:
            path_x,path_y,path_z = path_formater(final_path,nodePoses)
            plt.plot(path_x,path_y,path_z,'r',linewidth=4) 
            best_path_to_txt(final_path, nodePoses, "3Ddrone_path.txt")
        ax.set_xlim([0,60])
        ax.set_ylim([0,60])
        ax.set_zlim([0,60])
        plt.show()

def detect_collisions(newnode, ego, Obstacles):

        return 
        
def tree_formatter(nodes,nodePoses):           
    xi=[]; yi=[]; zi=[]
    for point in nodes:
        xi.append(nodePoses[point][0])
        yi.append(nodePoses[point][1])
        zi.append(nodePoses[point][2])     
    
    return xi, yi, zi


def path_formater(path,nodePoses):
    path_x = []
    path_y = []
    path_z = []
    for j in range(len(path)):
        path_x.append(nodePoses[path[j]][0])
        path_y.append(nodePoses[path[j]][1])
        path_z.append(nodePoses[path[j]][2])
    return path_x, path_y, path_z

def rrt_star(World,Start,End,Obs_pos,Obs_size,Resolution,egoSize,nodes,nodePoses,tree,pathFound,itr,itr_max,s_radius):
    final_nodes = [] #list of all possible
    final_path=None #lowest-cost endpoint
    path_found=False
    num_nds = 0
    start=time.time()
    while itr < itr_max:
         if itr % 1000 ==0:
             end=time.time()
             print("Iteration", itr, "of", itr_max,"found path:",path_found,"time",end-start)  
		 #sample a random node
         newNode = [randrange(0,World[0]),randrange(0,World[1]),randrange(0,World[2]-20)]
         itr += 1
         closest = -1 #stores the position in the tree of the closest node
		 #iterate through and find the closest node
         closest_distance=float("inf")
         for i in range(len(nodePoses)):
            newNode_distance=math.dist(nodePoses[i], newNode)
            if newNode_distance < closest_distance:
                closest = i
                closest_distance = newNode_distance
		 #create a vector from the closest node in the direction of the smpled node
		 #with magnitude equal to the resolution
		 #This part also resets the loop if the sampled node is on top of an esisting node
         closest_point_x = nodePoses[closest][0]
         closest_point_y = nodePoses[closest][1]
         closest_point_z = nodePoses[closest][2]
         vector_x = (newNode[0] - closest_point_x)
         vector_y = (newNode[1] - closest_point_y)
         vector_z = (newNode[2] - closest_point_z)
         mag=np.linalg.norm([vector_x, vector_y, vector_z])
         if mag != 0:
             unit_x = vector_x/mag
             unit_y = vector_y/mag
             unit_z = vector_z/mag
         else:
             closest = -1
             
         if closest != -1:
             newNode[0] = nodePoses[closest][0]+(Resolution*unit_x)
             newNode[1] = nodePoses[closest][1]+(Resolution*unit_y)
             newNode[2] = nodePoses[closest][2]+(Resolution*unit_z)
		 #check new node for obstacle collision
		 #if one is found set closest = -1
		 #do not check if the sampled node is on top of an existing node
         #no_obs = detect_collisions(newNode, egoSize, Obstacles)

         for obstacle in range(len(Obs_pos)): # **for some reason, the detect_collisions function doesn't work here, so I've left these if statements
           #if type(obstacle) is Rectangle:
            if newNode[0]+egoSize>Obs_pos[obstacle][0] and newNode[0]-egoSize<(Obs_pos[obstacle][0] + Obs_size[obstacle][0]) \
            and newNode[1]+egoSize>Obs_pos[obstacle][1] and newNode[1]-egoSize<(Obs_pos[obstacle][1] + Obs_size[obstacle][1]) \
            and newNode[2]+egoSize>Obs_pos[obstacle][2] and newNode[2]-egoSize<(Obs_pos[obstacle][2] + Obs_size[obstacle][2]):
                closest = -1
           # elif type(obstacle) is Circle: 
           #     if math.dist(newNode, obstacle.center) < obstacle.radius:
           #         closest = -1
         if newNode[0]<0 or newNode[0]>World[0] or newNode[1]<0 or newNode[1]>World[1] or newNode[2]<0 or newNode[2]>World[2]:
            closest = -1
		 #add new node to my tree if plausible point
         if closest != -1:
            num_nds += 1
            nodePoses.append(newNode)
            nodes[num_nds] = (newNode[0],newNode[1],newNode[2]) #add new node            
            # END OF TRADITIONAL RRT STEP
            if math.dist(End[0:3], newNode) <= End[3]: # if goal is reached, add node to final_nodes             
                final_nodes.append(num_nds)
                path_found=True
            #begin RRT* neighbor consideration
            if num_nds > 1:
                close_nodes = [] #neighborhood
                for node_idx in range(len(nodePoses)): #assemble neighborhood
                    if node_idx==closest or node_idx==num_nds: #exclude nearest node and new node
                        continue
                    elif math.dist(nodePoses[node_idx],newNode)<s_radius:
                            close_nodes.append(node_idx)      
                #evaluate lowest cost connection, starting to closest node
                path = nx.shortest_path(tree,0,closest,weight='weight')                   
                cost=edge_sum(tree.subgraph(path)) + closest_distance          
                cheapest_node=closest
                if len(close_nodes)>0: #if neighbors exist, check for cheaper connection to newnode
                    for neighbor in close_nodes:
                        path = nx.shortest_path(tree,0,neighbor,weight='weight')   
                        new_cost=edge_sum(tree.subgraph(path))+math.dist(nodePoses[neighbor],newNode)
                        if new_cost<cost:
                            cheapest_node=neighbor
                            cost=new_cost
                tree.add_edge(cheapest_node,num_nds,weight=math.dist(nodePoses[cheapest_node],newNode))
                #rewire step
                if len(close_nodes)>0:
                    path1 = nx.shortest_path(tree,0,num_nds,weight='weight')
                    for neighbor in close_nodes: 
                        if neighbor !=cheapest_node:    
                            new_cost=edge_sum(tree.subgraph(path1))+math.dist(nodePoses[neighbor],newNode) #cost of path to neighbor through new node   
                            path2 = nx.shortest_path(tree,0,neighbor,weight='weight') 
                            old_cost=edge_sum(tree.subgraph(path2)) #existing path to neighbor 
                            if new_cost<old_cost: #if cheaper to connect to neighbor thru new node, connect and break neighbor's previous edge
                                if path2[-2]>0: #do not remove connection to start!!!
                                    tree.remove_edge(path2[-2],neighbor)
                                    tree.add_edge(num_nds,neighbor,weight=math.dist(nodePoses[neighbor],newNode))
            else:
                tree.add_edge(0,1,weight=math.dist(nodePoses[0],nodePoses[1]))
    #output best path from list
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
    xs,ys,zs = path_formater(path,nodePoses)
    np.savetxt(fname, np.c_[xs, ys, zs], fmt='%1.2f')

def main(): 
    World = [60,60,60]     # Map size
    Start = [2.5,1,0]     # Start coordinate
    End   = [55,55,3,3] # End coordinate with radius

    Resolution = 3    # Resolution for node addition step
    egoSize    = 1.5  # Buffer space around obstacles
    itr_max    = 6000 # Steps for RRT*
    s_radius   = 4    # Neighbor search radius
    
    # Initialize nodes, variables, and the tree
    start     = time.time()
    nodes     = {}
    nodes[0]  = (Start[0],Start[1],Start[2])
    nodePoses = []
    nodePoses.append(Start)
    tree      = nx.Graph()
    itr       = 0
    pathFound = False #  
    plt.close('all')
    

    Obs_pos = [[0,11,0],[30,11,0],[10,20,0],[55,20,0],[55,45,0],[10,45,0],[10,20,10],[30,30,20],[20,30,0]]
    Obs_size = [[20,4,5],[20,4,7],[5,5,10],[5,5,10],[5,5,10],[5,5,10],[50,30,10],[15,15,15],[25,15,10]] 
    colors = ["crimson","crimson","limegreen","limegreen","limegreen","limegreen","limegreen","limegreen","blue"]
    
    tree,nodes,nodePoses,path_found,final_path = rrt_star(World, Start, End, Obs_pos,Obs_size, Resolution, egoSize,
                                                        nodes, nodePoses, tree, pathFound, itr, itr_max, s_radius)
    print("Path Found: ",path_found)
    # "subf" will make it plot a figure with two subfigures: the tree, and then the tree in the environment
    # any other string will make it plot a figure with just the environment and the tree in it
    graph(False, tree, nodes, nodePoses, path_found, final_path, World, Start, End, Obs_pos, Obs_size,colors, itr_max)

if __name__ == '__main__':
	main()