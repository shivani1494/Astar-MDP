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

                self.row = self.config["map_size"][0]
                self.column = self.config["map_size"][1]
		
		self.start = self.config["start"]
                self.goal = self.config["goal"]
                self.goal_r = self.goal[0]
                self.goal_c = self.goal[1]
                self.start_r = self.start[0]
                self.start_c = self.start[1]
                self.pits = self.config["pits"]
                self.walls = self.config["walls"]
	
		self.act = [     [[1,0],[-1,0],[0,-1],[0,1]],
				 [[-1,0],[1,0],[0,1],[0,-1]],
				 [[0,-1],[0,1],[-1,0],[1,0]],
				 [[0,1],[0,-1],[1,0],[-1,0]]
				]
					#forward   backward   left      right 
				#south  r+1, c+0   r-1, c+0   r+0, c-1  r+0, c+1
				#north  r-1, c+0   r+1, c+0   r+0, c+1  r+0, c-1
				#west	r+0, c-1   r+0, c+1   r-1, c+0  r+1, c+0
				#east   r+0, c+1   r+0, c-1   r+1,c+0   r-1, c+0 
		
		self.mmap1 = []
                self.mmap2 = []
		self.policyMap = []
		self.noChange = False
		self.setValueMap = True	
	
		#creating maps 
		self.createMap(self.mmap1, 0, False)
		self.createMap(self.mmap2, 0, False)#this is the value map, and it contains temp values from temp policy	
		self.createMap(self.policyMap, 0, False)#every location contains an arbitrary starting policy
		
		self.MDP_Algo()	
		
		print "\n"
		print np.reshape(self.policyMap, (self.row, self.column))
                
		self.resPub = rospy.Publisher("/results/policy_list", PolicyList, queue_size=10)
		policyList = PolicyList()


		policyL = []

		for r in range( len(self.policyMap)):
			for c in range(len(self.policyMap[r]) ):
				if self.policyMap[r][c] == 0 and not self.is_Wall(r, c):
					policyL.append("S")	
                
				if self.policyMap[r][c] == 1:
					policyL.append("N")	
				
				if self.policyMap[r][c] == 2:
					policyL.append("W")	
				
				if self.policyMap[r][c] == 3:
					policyL.append("E")	
					
				if self.is_Pit(r,c):
					policyL.append("PIT")	
				
				if self.is_Wall(r,c):
					policyL.append("WALL")	
				
				if self.goal_r == r and self.goal_c ==  c:
					policyL.append("GOAL")	

		print policyL 
		
		policyList.data = policyL
		rospy.sleep(0.3)
		self.resPub.publish(policyList)

	

 
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
	
	def createMap(self, mmap, minp, isPol):
                
		tempRow = []
		for i in range( (self.column) ):
			tempRow.append(minp)
	
		for j in range( (self.row)):
			mmap.append(deepcopy(tempRow) )
			
		if isPol:
			mmap[self.goal_r][self.goal_c] = "GOAL"
		else:
			mmap[self.goal_r][self.goal_c] = self.goalRwrd	

		for i in range(len(self.walls) ):
			wall_r = self.walls[i][0]	
			wall_c = self.walls[i][1]	
			if isPol:
				mmap[wall_r][wall_c] = "WALL"
			else:
				mmap[wall_r][wall_c] = self.wallRwrd	
				
		for i in range(len(self.pits) ):
			pit_r = self.pits[i][0]	
			pit_c = self.pits[i][1]	
			if isPol:
				mmap[pit_r][pit_c] = "PIT"
			else:
				mmap[pit_r][pit_c] = self.pitRwrd	

	def MDP_Algo(self):
		
		self.noChange = False
		while not self.noChange:
			self.noChange = True
			#initial policies were set when the map was created
			#value map --that is mmap1 gets updated only here
			self.setValueMap = True
			self.calculateNewRewardsPolicies()
			
			tempMap = deepcopy(self.mmap1)
			self.mmap1 = deepcopy(self.mmap2)
			self.mmap2 = []
			self.createMap(self.mmap2, 0, False)
			
			self.setValueMap = False
			#policy map is updated only here
			self.calculateNewRewardsPolicies()
			#what happens in the last iteration?
			print self.noChange			
			
	
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
				
				QBest = self.mmap1[r][c]				

				#act
					#forward   backward   left      right 
				#south  r+1, c+0   r-1, c+0   r+0, c-1  r+0, c+1
				#north  r-1, c+0   r+1, c+0   r+0, c+1  r+0, c-1
				#west	r+0, c-1   r+0, c+1   r-1, c+0  r+1, c+0
				#east   r+0, c+1   r+0, c-1   r+1,c+0   r-1, c+0 
	
				#probAct: forward backward left right

				#for every action there is one reward
				actnRwrd = []
				for a in range( len ( self.act ) ):
					#rwrd_allDrctns_gvnActn = []
			
					if a != self.policyMap[r][c] and self.setValueMap:
						print "action not implemtned"
						print a
						continue
					#0 -- south
					#1 -- north
					#2 -- west
					#3 -- east
					
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
					
					
					if self.setValueMap:
						self.mmap2[r][c] = actnRwrd[0]
						break				

					#all this is executed only when setValueMap is false bcuz we break early if not.		
					Qsa = actnRwrd[0] #this will only be 1 when we execute the following if stmt		
			
					if Qsa > QBest:
						print "action"
						print a
						self.policyMap[r][c] = a
						QBest = Qsa
						self.noChange = False
						print self.noChange

		print np.reshape(self.mmap1, (self.row, self.column))
		print "\n"
		print np.reshape(self.mmap2, (self.row, self.column))
		print "\n"
		print np.reshape(self.policyMap, (self.row, self.column))

