#!/usr/bin/env python
import numpy as np
import math
import rospy
from turtlebot_srv.srv import Planner,PlannerResponse,PlannerRequest
from std_msgs.msg import Float32MultiArray


class XYNode:
	g_score = 0
	f_score = 1000
	parent = '00000'
	key = '00000'
	x = 0
	y = 0

class XYPlanner:

	def NodeInitialize(self,node,x_init,y_init):
		node.x = x_init
		node.y = y_init
		node.key = str(node.x)+str(node.y)
		return node

	def g_score(self,current,neighbor):
		g_score = math.sqrt((current.x-neighbor.x)**2+(current.y-neighbor.y)**2)
		return g_score

	def heuristic_cost(self,neighbor,goal):
		heuristic_cost = 100*(math.sqrt((neighbor.x-goal.x)**2+(neighbor.y-goal.y)**2))
		return heuristic_cost

	def goalReached(self,current,goal):
		if abs(goal.x-current.x) < 0.5 and abs(goal.y-current.y) < 0.5:
			return 'true'
		else:
			return 'false'

	def GetNeighbors(self,current):
		x_val = current.x
		y_val = current.y
		df = 0.1
		max_radius = 0.5
		neighbor_list = []

		for i in np.arange(current.x - max_radius,current.x + max_radius+df,df):
			for j in np.arange(current.y - max_radius,current.y + max_radius+df,df):
				if math.sqrt(abs(current.x-i)**2 + abs(current.y-j)**2) > 1e-8:

					neighbor_node = XYNode()
					neighbor_node.g_score = 0
					neighbor_node.x = i
					neighbor_node.y = j
					neighbor_node.key = str(neighbor_node.x)+str(neighbor_node.y)
					neighbor_node.parent = current.key
					neighbor_list.append(neighbor_node)

		return neighbor_list

	def CompareClosedSet(self,ns_node,ClosedSet):
		for cs_node in ClosedSet:
			if abs(ns_node.x-cs_node.x) < 1e-5 and abs(ns_node.y-cs_node.y) < 1e-5:
				return 'true'


	def CompareExploredSet(self,ns_node,ExploredSet):
		for es_node in ExploredSet:
			if abs(ns_node.x-es_node.x) < 1e-5 and abs(ns_node.y-es_node.y) < 1e-5:
				return es_node


	def RemoveFromExploredSet(self,current_node,ExploredSet):
		for es_node in ExploredSet:
			if abs(current_node.x-es_node.x) < 1e-5 and abs(current_node.y-es_node.y) < 1e-5:
				ExploredSet.remove(es_node)

	def CompareClosedSetKey(self,current_node,ClosedSet):
		for cs_node in ClosedSet:
			if current_node.parent == cs_node.key:
				return cs_node

	def ReproducePath(self,current_node,OpenSet):
		planner = XYPlanner()
		optimal_path = []
		op_xcoords = []
		op_ycoords = []
		optimal_path.append(current_node)
		while current_node.parent != '00000':
			opt_node = planner.CompareClosedSetKey(current_node,OpenSet)
			optimal_path.append(opt_node)
			current_node = opt_node

		for opt_node in optimal_path:
			op_xcoords.append(opt_node.x)
			op_ycoords.append(opt_node.y)

		return optimal_path

	def Obstacles(self):
		obstacle_list = [[0.5,0.5],[2,2],[3,3],[5,5]]
		obstacles_x = [0.5,2,3,5]
		obstacles_y = [0.5,2,3,5]
		return obstacle_list,obstacles_x,obstacles_y

	def CompareObstacleSet(self,obstacle_list,neighbor):
		for ob in obstacle_list:
			ob_x = ob[0]
			ob_y = ob[1]
			if math.sqrt((ob_x-neighbor.x)**2+(ob_y-neighbor.y)**2) < 1:
				neighbor.f_score = 1000
		return neighbor

	def PathtoMessage(self,optimal_path):
		path_coords = []
		for node in optimal_path:
			# temp_coord = []
			# temp_coord.append(node.x)
			# temp_coord.append(node.y)
			# path_coords.append(temp_coord)
			path_coords.append(node.x)
			path_coords.append(node.y)
		return path_coords

	def test(self,req):
		message = req.coords_msg
		print('function successful!!!')
		print(message)
		PlannerResponse.optimal_path = message
		return plannerResponse

	def DoAstar(self,req):
		begin_end_coords= req.coords_msg
		begin_x = begin_end_coords[0]
		begin_y = begin_end_coords[1]
		goal_x = begin_end_coords[2]
		goal_y = begin_end_coords[3]

		#print('passed initial a star')


		OpenSet = []
		ClosedSet = []
		ExploredSet = []

		begin_node = XYNode()
		goal_node = XYNode()
		test_node1 = XYNode()
		begin_node = self.NodeInitialize(begin_node,begin_x,begin_y)
		goal_node = self.NodeInitialize(goal_node,goal_x,goal_y)
		test_node1 = self.NodeInitialize(test_node1,3.5,4.5)
		
		OpenSet.append(begin_node)

		ExploredSet.append(begin_node)
		ExploredSet.append(test_node1)
		
		ClosedSet.append(test_node1)

		obstacle_list,obstacles_x,obstacles_y = self.Obstacles()
		#print('obstacles: %d' %(obstacle_list))




		while(OpenSet):

			OpenSet.sort(key=lambda xy: xy.f_score)
			current_node = OpenSet[0]

			if self.goalReached(current_node,goal_node) == 'true':
				#print('FOUND A PATH')

				OpenSet.pop(0)
				optimal_path = self.ReproducePath(current_node,ClosedSet)

				path_coords = self.PathtoMessage(optimal_path)

				#print('path coordinates::')
				#print(path_coords)


				# plannerResponse = path_coords

				# req.optimal_path = path_coords

				return plannerResponse(path_coords)

				break
			else:		
				OpenSet.pop(0)

				ClosedSet.append(current_node)
				self.RemoveFromExploredSet(current_node,ExploredSet)

				neighbor_nodes = self.GetNeighbors(current_node)

				for ns_node in neighbor_nodes:
					if self.CompareClosedSet(ns_node,ClosedSet) != 'true':
						es_node = self.CompareExploredSet(ns_node,ExploredSet)
						if es_node:

							tentative_gscore = current_node.g_score + self.g_score(current_node,ns_node)

							if tentative_gscore < es_node.g_score:

								es_node.parent = current_node.key
								es_node.g_score = tentative_gscore
								es_node.f_score = tentative_gscore + self.heuristic_cost(ns_node,goal_node)
								es_node = self.CompareObstacleSet(obstacle_list,es_node)
										
						else:
							ns_node.g_score = current_node.g_score + self.g_score(current_node,ns_node)
							ns_node.f_score = ns_node.g_score + self.heuristic_cost(ns_node,goal_node)
							#ns_node = self.CompareObstacleSet(obstacle_list,ns_node)
							OpenSet.append(ns_node)
							ExploredSet.append(ns_node)


def planner_server():
	a_planner = XYPlanner()
	rospy.init_node('planner_server')
	planner_srv = rospy.Service('planner_service', Planner, a_planner.DoAstar)
	rospy.spin()



if __name__ == '__main__':
	planner_server()
