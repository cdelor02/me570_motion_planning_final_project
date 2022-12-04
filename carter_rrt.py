import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from random import randrange
import numpy as np
import math
from math import sqrt
from networkx.classes.function import path_weight


def rrt(World,Start,End,Obstacles,Resolution,egoSize):
	#Initialize nodes, variables, and the tree
	nodes = {}
	nodes[0] = (Start[0],Start[1])
	nodePoses = []
	nodePoses.append(Start)
	tree = nx.Graph()
	pathFound = False
	itr = 0



	while pathFound == False:
		#sample a random node
         newNode = [randrange(0,World[0]),randrange(0,World[1])]

         closest = -1 #stores the position in the tree of the closest node

		#iterate through and find the closest node
		#assign the position in the nodePoses list to the closest cariable
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
             newNode[0] =  nodePoses[closest][0]+(Resolution*unit_x)
             newNode[1] = nodePoses[closest][1]+(Resolution*unit_y)

		#check new node for obstacle colision
		#if one is found set closest = -1
		#do not check if the smapled node is on top of an existing node
         for Obstacle in Obstacles:
             if newNode[0]>Obstacle[0] and newNode[0]<Obstacle[1] and newNode[1]>Obstacle[2] and newNode[1]<Obstacle[3]:
                closest = -1

		#add new node to my tree and report if path is found
         if closest != -1:
            itr += 1
            nodePoses.append(newNode)
            nodes[itr] = (newNode[0],newNode[1])
            tree.add_edge(closest,itr,weight=np.hypot(nodePoses[closest][0]-nodePoses[itr][0],nodePoses[closest][1]-nodePoses[itr][1]))
            if np.hypot(End[0]-newNode[0],End[1]-newNode[1]) <=  End[2]:
                pathFound=True

            
		    

	
	#display options for the tree
	options = {
		"font_size": 1,
		"node_size": 3,
		"node_color": "white",
		"edgecolors": "green",
		"linewidths": 5,
		"width": 5,
	}

	#report shortest path to terminal node
	path = nx.shortest_path(tree,0,itr,weight='weight')
	print(path)
	nx.draw_networkx(tree, nodes, **options)

	#format path for export
	path_x = []
	path_y = []

	for j in range(len(path)):
		path_x.append(nodePoses[path[j]][0])
		path_y.append(nodePoses[path[j]][1])

	#plot your results
	plt.plot(path_x,path_y,'r')
	endRegion =plt.Circle((End[0],End[1]),End[2],color='r')
	ax = plt.gca()
	ax.add_patch(Rectangle((Obstacles[0][0],Obstacles[0][2]),Obstacles[0][1]-Obstacles[0][0],Obstacles[0][3]-Obstacles[0][2]))
	ax.add_patch(endRegion)
	plt.plot([0,0,World[0],World[0],0],[0,World[1],World[1],0,0])
	plt.axis("on")
	plt.axis("equal")
	plt.show()




def main():
	World = [10,20] #Map size
	Start = [2.5,0] #Start coordinate
	End = [2.5,15,2.5] #End Coordinate with radius
	Obstacles = [[0,7,11,15]] #[x_1, x_2, y_1, y_2]
	Resolution = 1 #Resolution of rrt
	egoSize = .15
	rrt(World,Start,End,Obstacles,Resolution,egoSize)

if __name__ == '__main__':
	main()