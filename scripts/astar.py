#!/usr/bin/env python
import sys
import rospy
import math as m
import numpy as np
from copy import deepcopy
from read_config import read_config
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_3.msg import AStarPath, PolicyList
import heapq as h

class RAstar:
	def __init__(self):
		self.config = read_config()
		self.resPub = rospy.Publisher("/results/path_list", AStarPath, queue_size=10)
		astarp = AStarPath()
		astarp.data = [0, 0]
		#print ("are you publishing??")
		self.resPub.publish(astarp)		
		self.row = self.config["map_size"][0]
		self.column = self.config["map_size"][1]
		#print ("self.row")
		#print self.row
		#print ("self.column")
		#print self.column

		self.tempRow = []
		self.mmap = []
		#creating the map 
		for i in range( (self.column) ): 
			self.tempRow.append([0,0,0,0])	

		for j in range( (self.row)):
			self.mmap.append(deepcopy(self.tempRow) ) 
	
		#m = deepcopy(self.mmap)
		#print np.reshape(m, (self.row, self.column) )	
		#print ("self.mmap")
		#print len (self.mmap)
		#print len (self.mmap[0])
		#print "aaA"
		#print  ("self.mmap")
		#print  (self.mmap)

		self.start = self.config["start"]
		self.goal = self.config["goal"]
		self.goal_r = self.goal[0]
		self.goal_c = self.goal[1]
		self.start_r = self.start[0]
		self.start_c = self.start[1]
		self.pits = self.config["pits"]
		self.walls = self.config["walls"]
		self.openList = set([])
		self.AStar()

	def is_Wall_Pit_InMap(self, curr_r, curr_c):
		c_Bound = self.column - 1
		r_Bound = self.row - 1
		if curr_r < 0 or curr_c < 0 or curr_r > r_Bound or curr_c > c_Bound: 
			#print "out of bounds: curr row and column"
			#print curr_r
			#print curr_c
			return False
		#checking for pits	
		for i in range( len(self.pits) ):
			if self.pits[i][0] == curr_r and self.pits[i][1] == curr_c:
				return False
		#checking for walls
		for i in range( len(self.walls) ):
			if self.walls[i][0] == curr_r and self.walls[i][1] == curr_c:
				return False
		return True

	def initializeMap(self):
		for j in range( len(self.mmap) ):
			for i in range( len(self.mmap[j])  ):
				hrstic = abs(self.goal_c - i) + abs(self.goal_r - j)
				backC = sys.maxint
				f_hb = hrstic + backC
				parent = None
				
				
				if j == self.start_r and i == self.start_c:
					#print "entering for the start case"
					#print j
					#print i
					self.mmap[j][i][0] = 0 
					self.mmap[j][i][1] = 0 
					self.mmap[j][i][2] = 0
					self.mmap[j][i][3] = parent
						
				elif j == self.goal_r and i == self.goal_c:
					self.mmap[j][i][0] = 0 
					self.mmap[j][i][1] = backC 
					self.mmap[j][i][2] = backC 
					self.mmap[j][i][3] = parent
				
				else:		
					self.mmap[j][i][0] = hrstic 
					self.mmap[j][i][1] = backC 
					self.mmap[j][i][2] = f_hb
					self.mmap[j][i][3] = parent
		#print self.mmap

	def addToOpenList(self, currX, currY):
		strCurr = str(currX) + str(currY)
		self.openList.add(strCurr)	
	def isInOpenList(self, currX, currY):
		strCurr = str(currX) + str(currY)
		if strCurr in self.openList:
			return True
		return False
	def removeFrOpenList(self, currX, currY):
		strCurr = str(currX) + str(currY)
		self.openList.remove(strCurr)
	
	def AStar(self):	
		#set all the initial values
		self.initializeMap()
		#keep a track of all the tentative paths
		allPaths = []
		#start = [ 7, '30'] 
		start = []
		start.append(self.mmap[self.start_r][self.start_c][2])
		start.append(str(self.start_r)+ str(self.start_c) )	
		self.addToOpenList(self.start_r, self.start_c)
		allPaths.append(start)
		h.heapify(allPaths)
		
		while allPaths:		
			curr = h.heappop(allPaths)	
			temp = list(curr[1])			
			curr_r = int(temp[0])
			curr_c = int(temp[1])
			suc = []
			self.removeFrOpenList(curr_r, curr_c)
			#print "curr row and column"
			#print curr_r
			#print curr_c
			if curr_c == self.goal_c and curr_r == self.goal_r:
				#print ("goal found")
				oPath = []
				rw = curr_r
				clm = curr_c
				pathC = True
				while pathC:
					parentL = self.mmap[rw][clm][3]
					oPath.append([rw, clm])
					if parentL is None:
						pathC = False
						break

					rw = parentL[0]
					clm = parentL[1]
				
				optimalPath = list(reversed(oPath))
				print optimalPath	
				#print optimalPath
				for i in range(len(optimalPath)):
					#print "optimalPath[i]"
					#print optimalPath[i]
					astar_path = AStarPath()
					astar_path.data = optimalPath[i]
					self.resPub.publish(astar_path)		
		
				break
			else:
				if self.is_Wall_Pit_InMap(curr_r+1, curr_c):
					suc.append( [curr_r+1, curr_c] )
				if self.is_Wall_Pit_InMap(curr_r-1, curr_c):
					suc.append( [curr_r-1, curr_c] )
				if self.is_Wall_Pit_InMap(curr_r, curr_c+1):
					suc.append( [curr_r, curr_c+1] )
				if self.is_Wall_Pit_InMap(curr_r, curr_c-1):
					suc.append( [curr_r, curr_c-1] )
				#print "succesor:"
				#print suc		
				for i in range (len (suc) ):
					ncurr_r = suc[i][0]
					ncurr_c = suc[i][1]
					
					setParam = False
					if self.mmap[ncurr_r][ncurr_c][1] == sys.maxint:
						self.addToOpenList(ncurr_r, ncurr_c)
						#print "new row_column to be added"
						#print ncurr_r
						#print ncurr_c
						#print "self.mmap[ncurr_r][ncurr_c]"	
						#print self.mmap[ncurr_r][ncurr_c]	
						setParam = True
		
					elif self.isInOpenList(ncurr_r, ncurr_c):
						if self.mmap[ncurr_r][ncurr_c][1] > self.mmap[curr_r][curr_c][1] + 1:
							print "did we ever enter into this one?"	
							setParam = True
					if setParam:							
						self.mmap[ncurr_r][ncurr_c][3] = [curr_r, curr_c]
						self.mmap[ncurr_r][ncurr_c][1] = 1 + self.mmap[curr_r][curr_c][1]
						self.mmap[ncurr_r][ncurr_c][2] = self.mmap[ncurr_r][ncurr_c][0] + self.mmap[ncurr_r][ncurr_c][1]
						tempAdd = [ self.mmap[ncurr_r][ncurr_c][2] ]
						tempAdd.append(str(ncurr_r)+ str(ncurr_c))	
						h.heappush(allPaths, tempAdd) 
						#print "self.mmap[ncurr_r][ncurr_c]"	
						#print self.mmap[ncurr_r][ncurr_c]	
