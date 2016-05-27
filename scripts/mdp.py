#!/usr/bin/env python
import numpy as np
import math as m
from copy import deepcopy
import rospy
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_3.msg import AStarPath, PolicyList
import json
import image_util
from read_config import read_config

class MDP:

	def __init__(self):
		self.config = read_config()

		self.probF = self.config["prob_move_forward"]
		self.probB = self.config["prob_move_backward"]
		self.probL = self.config["prob_move_left"]
		self.probR = self.config["prob_move_right"]
		self.probAct = [ self.probF, self.probB, self.probL, self.probR]

		self.maxI = self.config["max_iterations"]
		self.threshDiff = self.config["threshold_difference"]
		
		self.stepRwrd = self.config["reward_for_each_step"]
		self.wallRwrd = self.config["reward_for_hitting_wall"]
		self.goalRwrd = self.config["reward_for_reaching_goal"]
		self.pitRwrd = self.config["reward_for_falling_in_pit"]

                #self.resPub = rospy.Publisher("/results/path_list", AStarPath, queue_size=10)
                #self.resPub.publish([1,1])
                self.row = self.config["map_size"][0]
                self.column = self.config["map_size"][1]
    
                self.tempRow = []
		self.tempRowP = []
		self.mmap1 = []
                #creating the map 
		for i in range( (self.column) ):
			self.tempRowP.append("")
			self.tempRow.append(0)
	
		for j in range( (self.row)):
			self.mmap1.append(deepcopy(self.tempRow) )

                self.mmap2 = deepcopy(self.mmap1) 

		self.policyMap = []
		for j in range( (self.row)):
                        self.policyMap.append(deepcopy(self.tempRowP) )
               	
		#print np.reshape(self.mmap1, (self.row, self.column))
		#print "\n"
		#print np.reshape(self.mmap2, (self.row, self.column))
		#print "\n"
		#print np.reshape(self.policyMap, (self.row, self.column))
 
		self.start = self.config["start"]
                self.goal = self.config["goal"]
                self.goal_r = self.goal[0]
                self.goal_c = self.goal[1]
                self.start_r = self.start[0]
                self.start_c = self.start[1]
                self.pits = self.config["pits"]
                self.walls = self.config["walls"]
	
		self.act = [ [[1, 0 ], [-1,0], [0, -1], [0, 1]],
				 [[-1, 0],[1, 0], [0,1], [0,-1]],
				 [[0,-1], [0,1], [-1,0], [1,0]],
				 [[0,1],[0, -1],[1, 0],[-1, 0]]
				]
					#forward   backward   left      right 
				#north  r+1, c+0   r-1, c+0   r+0, c-1  r+0, c+1
				#south  r-1, c+0   r+1, c+0   r+0, c+1  r+0, c-1
				#west	r+0, c-1   r+0, c+1   r-1, c+0  r+1, c+0
				#east   r+0, c+1   r+0, c-1   r+1,c+0   r-1, c+0 
		
		self.MDP_Algo()		
        
	def is_InMap(self, curr_r, curr_c):
                c_Bound = self.column - 1
                r_Bound = self.row - 1
                if curr_r < 0 or curr_c < 0 or curr_r > r_Bound or curr_c > c_Bound:
                        return False
                return True
	
	def is_Wall(self, curr_r, curr_c):
                #checking for walls
                for i in range( len(self.walls) ):
                        if self.walls[i][0] == curr_r and self.walls[i][1] == curr_c:
                                return True
		return False 
	
	def is_Pit(self, curr_r, curr_c):
                #checking for pits      
                for i in range( len(self.pits) ):
                        if self.pits[i][0] == curr_r and self.pits[i][1] == curr_c:
                                return True 
		return False

	def MDP_Algo(self):
		
		self.mmap1[self.goal_r][self.goal_c] = self.goalRwrd	
		self.mmap2[self.goal_r][self.goal_c] = self.goalRwrd	

		for i in range(len(self.walls) ):
			wall_r = self.walls[i][0]	
			wall_c = self.walls[i][1]	
			self.mmap1[wall_r][wall_c] = self.wallRwrd	
			self.mmap2[wall_r][wall_c] = self.wallRwrd	
				
		for i in range(len(self.pits) ):
			pit_r = self.pits[i][0]	
			pit_c = self.pits[i][1]	
			self.mmap1[pit_r][pit_c] = self.pitRwrd	
			self.mmap2[pit_r][pit_c] = self.pitRwrd	
		
		#self.calculateNewRewardsPolicies()
		#self.mmap1 = deepcopy(self.mmap2)
		
		#self.calculateNewRewardsPolicies()
		#self.mmap1 = deepcopy(self.mmap2)
		
		for itr in range(self.maxI):
			self.calculateNewRewardsPolicies()
			
			tempMap = deepcopy(self.mmap1)
			self.mmap1 = deepcopy(self.mmap2)

			#self.mmap2 = []
			#tempRow = []
			#for i in range( (self.column) ):
				#tempRow.append(0)
			
			#for j in range( (self.row)):
				#self.mmap2.append(deepcopy(tempRow) )

			#what happens in the last iteration?
	def calculateNewRewardsPolicies(self):
		dF = self.config["discount_factor"]
		
		for r in range ( len (self.mmap1) ):
			
			for c in range ( len (self.mmap1[r]) ):
			
				if self.is_Pit(r,c):
					continue
				if self.is_Wall(r,c):
					continue
				if self.goal_r == r and self.goal_c ==  c:
					continue

				#act
					#forward   backward   left      right 
				#north  r+1, c+0   r-1, c+0   r+0, c-1  r+0, c+1
				#south  r-1, c+0   r+1, c+0   r+0, c+1  r+0, c-1
				#west	r+0, c-1   r+0, c+1   r-1, c+0  r+1, c+0
				#east   r+0, c+1   r+0, c-1   r+1,c+0   r-1, c+0 
	
				#probAct: forward backward left right

				#for every action there is one reward
				actnRwrd = []
				for a in range( len ( self.act ) ):
					#rwrd_allDrctns_gvnActn = []
					probRwrd = []	
					for d in range(len(self.act[a]) ):
						movRow = r + self.act[a][d][0] #for eg: moving forward with action north
						movCol = c + self.act[a][d][1] #fod eg: moving forward with action north

						if self.goal_r == movRow and self.goal_c == movCol:
						#prev_reward + goalRwrd					
							ttl_rwrd = dF*self.mmap1[movRow][movCol] + self.stepRwrd + self.goalRwrd
							#nlocP = [ r+act[a][d][0], c+act[a][d][1]], self.probAct[d] ]
					
						elif self.is_Pit(movRow, movCol):
							ttl_rwrd = dF*self.mmap1[movRow][movCol] + self.stepRwrd + self.pitRwrd
							#nloc_i.append( [r+act[a][d][0], c+act[a][d][1] )
							
						elif self.is_Wall(movRow, movCol):
						#stay put, so get the prev reward of staying put + wallRwrd
							ttl_rwrd = dF*self.mmap1[r][c] + self.wallRwrd
							#nloc_i.append([r, c])
						elif self.is_InMap(movRow, movCol):
							#nloc_i.append( [r+act[a][d][0], c+act[a][d][1] )
							ttl_rwrd = dF*self.mmap1[movRow][movCol] + self.stepRwrd
						elif not self.is_InMap(movRow, movCol):
							#nloc_i.append( [r, c] )
							ttl_rwrd = dF*self.mmap1[r][c] + self.wallRwrd 
						
						#rwrd_allDrctns_gvnActn.append(ttl_rwrd)
						probRwrd.append(ttl_rwrd*self.probAct[d])							
					actionRwrd_i = 0
					for pr in range( len(probRwrd) ):
						actionRwrd_i += probRwrd[pr]
					actnRwrd.append(actionRwrd_i)
					#print "len(actnRwrd)"
					#print len(actnRwrd)

				maxRwrd = 0
				maxRwrdA = 0
				for mr in range(len(actnRwrd)):
					if actnRwrd[mr] > maxRwrd:
						maxRwrdA = mr
						maxRwrd = actnRwrd[mr]

				self.mmap2[r][c] = maxRwrd
				if maxRwrdA == 0:
					self.policyMap[r][c] = "N"
				elif maxRwrdA == 1:
					self.policyMap[r][c] = "S"
				elif maxRwrdA == 2:
					self.policyMap[r][c] = "W"
				else:
					self.policyMap[r][c] = "E"
				
		print np.reshape(self.mmap1, (self.row, self.column))
		print "\n"
		print np.reshape(self.mmap2, (self.row, self.column))
		print "\n"
		print np.reshape(self.policyMap, (self.row, self.column))

