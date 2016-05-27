#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_3.msg import AStarPath, PolicyList
import json
import image_util
from read_config import read_config


class MDP:

	def __init__(self):
		self.config = read_config()

		self.probF = self.config["probability_move_forward"]
		self.probB = self.config["probability_move_backward"]
		self.probL = self.config["probability_move_left"]
		self.probR = self.config["probability_move_right"]
		self.probAct = [ self.probF, self.probB, self.probL, self.probR]

		self.maxI = self.config["max_iterations"]
		self.threshDiff = self.config["thershold_difference"]
		
		self.stepRwrd = self.config["reward_for_each_step"]
		self.wallRwrd = self.config["reward_for_hitting_wall"]
		self.goalRwrd = self.config["reward_for_reaching_goal"]
		self.pitRwrd = self.config["reward_for_falling_in_pit"]

                #self.resPub = rospy.Publisher("/results/path_list", AStarPath, queue_size=10)
                #self.resPub.publish([1,1])
                self.row = self.config["map_size"][0]
                self.column = self.config["map_size"][1]
    
                self.tempRow = []
                self.mmap1 = []
                #creating the map 
                for i in range( (self.column) ):
                        self.tempRow.append([0 , 0 , 0, 0])

                for j in range( (self.row)):
                        self.mmap1.append(deepcopy(self.tempRow) )

                self.mmap2 = []

                for j in range( (self.row)):
                        self.mmap2.append(deepcopy(self.tempRow) )

                self.start = self.config["start"]
                self.goal = self.config["goal"]
                self.goal_r = self.goal[0]
                self.goal_c = self.goal[1]
                self.start_r = self.start[0]
                self.start_c = self.start[1]
                self.pits = self.config["pits"]
                self.walls = self.config["walls"]
	
		self.actions = [ [[1, 0 ], [-1,0], [0, -1], [0, 1] ],
				 [[-1, 0],[1, 0], [0,1], [0,-1]],
				 [[0,-1], [0,1], [-1,0], [1,0]],
				 [[0,1],[0, -1],[1, 0],[-1, 0]]
				]
					#forward   backward   left      right 
				#north  r+1, c+0   r-1, c+0   r+0, c-1  r+0, c+1
				#south  r-1, c+0   r+1, c+0   r+0, c+1  r+0, c-1
				#west	r+0, c-1   r+0, c+1   r-1, c+0  r+1, c+0
				#east   r+0, c+1   r+0, c-1   r+1,c+0   r-1, c+0 
		
		
        
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
			wallX = self.walls[i][0]	
			wallY = self.walls[i][1]	
			self.mmap1[wallY][wallX] = self.wallRwrd	
			self.mmap2[wallY][wallX] = self.wallRwrd	
				
		for i in range(len(self.pits) ):
			pitX = self.pits[i][0]	
			pitY = self.pits[i][1]	
			self.mmap1[pitY][pitX] = self.pitRwrd	
			self.mmap2[pitY][pitX] = self.pitRwrd	

		for r in range ( len (self.mmap) ):
			
			for c in range ( len (self.mmap[i]) ):
					#forward   backward   left      right 
				#north  r+1, c+0   r-1, c+0   r+0, c-1  r+0, c+1
				#south  r-1, c+0   r+1, c+0   r+0, c+1  r+0, c-1
				#west	r+0, c-1   r+0, c+1   r-1, c+0  r+1, c+0
				#east   r+0, c+1   r+0, c-1   r+1,c+0   r-1, c+0 
	
				#prob: forward backward left right
				for a in range( len ( self.actions ) ):
					for d in range(len(self.actions[a]) ):

					if isGoal(r + act[a][d][0], c + act[a][d][1] ):
						#prev_reward + goalRwrd					
						reward = self.mmap1[r+act[a][d][0]][c+act[a][d][1]] + goalRwrd
						nlocP = [ [r+act[a][d][0], c+act[a][d][1]], self.probAct[d] ]
					
					elif self.is_Pit(r + act[a][d][0], c + act[a][d][1]):
						
						reward = self.mmap1[r+act[a][d][0]][c+act[a][d][1]] + pitRwrd
						nlocP = [ [r+act[a][d][0], c+act[a][d][1]], self.probAct[d] ]
							
					elif self.is_Wall(r + act[a][d][0], c + act[a][d][1]):
						#stay put, so get the prev reward of staying put + wallRwrd
						reward = self.mmap1[r][c] + wallRwrd
						nlocP = [ [r, c], self.probAct[d] ]
					elif self.is_InMap(r + act[a][d][0], c + act[a][d][1]):

						nlocP = [ [r+act[a][d][0], c+act[a][d][1]], self.probAct[d] ]
						reward = stepRwrd
					elif not self.is_InMap(r + act[a][d][0], c + act[a][d][1]):
	
						nlocP = [ [r, c], self.probAct[d] ]
						reward = stepRwrd
