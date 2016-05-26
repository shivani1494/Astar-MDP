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
#astar implementation

class RAstar:
	def __init__(self):
		self.config = read_config()
		self.resPub = rospy.Publisher("/results/path_list", AStarPath, queue_size=10)
		self.resPub.publish([1,1])	
		self.row = self.config["map_size"][0]
		self.column = self.config["map_size"][1]
		print ("self.row")
		print self.row
		print ("self.column")
		print self.column

		self.tempRow = []
		self.mmap = []
		#creating the map 
		for i in range( (self.column) ): 
			self.tempRow.append([0,0,0,0])	

		for j in range( (self.row)):
			self.mmap.append(deepcopy(self.tempRow) ) 
		
		print ("self.mmap")
		print self.mmap

		self.start = self.config["start"]
		self.goal = self.config["goal"]
		self.goalX = self.goal[0]
		self.goalY = self.goal[1]
		self.startX = self.start[0]
		self.startY = self.start[1]
		self.pits = self.config["pits"]
		self.walls = self.config["walls"]

		self.openList = set([])
		self.closedList = set([])		
		self.AStar()


	def addToOpenList(self, currX, currY):
		strCurr = str(currX) + str(currY)
		self.openList.add(strCurr)	

	def addToClosedList(self, currX, currY):	
		strCurr = str(currX) + str(currY)
		self.closedList.add(strCurr)	

	def isInOpenList(self, currX, currY):
		strCurr = str(currX) + str(currY)
		if strCurr in self.openList:
			return true
		return false

	def isInClosedList(self, currX, currY):
		strCurr = str(currX) + str(currY)
		if strCurr in self.closedList: 
			return true
		return false

	def is_Wall_Pit_InMap(self, currX, currY):
		xBound = self.column - 1
		yBound = self.row - 1
		if currX < 0 or currY < 0 or currX > xBound or currY > yBound: 
			return false
		#checking for pits	
		for i in range( len(self.pits) ):
			if self.pits[i][0] == currX and self.pits[i][1] == currY:
				return false
		#checking for walls
		for i in range( len(self.walls) ):
			if self.walls[i][0] == currX and self.walls[i][1] == currY:
				return false
		return true

	def initializeMap(self):
		for i in range( len(self.mmap) ):
			for j in range( len(self.mmap[i]) ):
				hrstic = abs(self.goalY - j) + abs(self.goalX - i)
				backC = sys.maxint
				f_hb = hrstic + backC
				zero = 0
				parent = None
				if i == self.startX and j == self.startY:
					self.mmap[i][j][0] = hrstic
					self.mmap[i][j][1] = zero 
					self.mmap[i][j][2] = hrstic
					self.mmap[i][j][3] = parent
				
				if i == self.goalX and j == self.goalY:
					self.mmap[i][j][0] = zero
					self.mmap[i][j][1] = backC 
					self.mmap[i][j][2] = backC 
					self.mmap[i][j][3] = parent
				
				else:		
					self.mmap[i][j][0] = hrstic 
					self.mmap[i][j][1] = backC 
					self.mmap[i][j][2] = f_hb
					self.mmap[i][j][3] = parent
				#this is a tuple of all the values we keep track of
		print ("self.mmap")
		print self.mmap
	
	def AStar(self):	
		#set all the initial values
		self.initializeMap()
		#keep a track of all the tentative paths
		allPaths = []
	
		#start = [ 7, '30'] 
		start = []
		start.append(self.mmap[self.startX][self.startY][2])
		start.append(str(self.startX)+ str(self.startY) )	

		allPaths.append(start)
		h.heapify(allPaths)

		#does the map wrap around?	
		while not allPaths:		
			curr = h.heappop(allPaths)	
			
			temp = list(curr[1])			
			currX = int(temp[0])
			currY = int(temp[1])
			suc = []
			#goal test 
			if mmap[currX][currY] == mmap[self.goalX][self.goalY]:
				print ("goal found")
				oPath = []
				corX = currX
				corY = currY
				pathC = true
				while pathC:
					corP = mmap[corX][corY][3]
					oPath.append([corX, corY])
					if corP is not None:
						pathC = false
						break

					tempL = list(corP)			
					corX = int(temp[0])
					corY = int(temp[1])
				
				#reverse oPath
				optimalPath = list(reversed(oPath))
				'''#find out the steps the robot will have to take based on
				self.moves = []
				for i in range( len (oPath) - 1):
					#either x has to be equal
					if oPath[i][0] == oPath[i+1][0]:
						if oPath[i][1] + 1 == oPath[i+1][1]:
							moves.append([0,1])
						elif oPath[i][1] - 1 == oPath[i+1][1]:	
							moves.append([0,-1])

					#or y has to be equal
					elif oPath[i][1] == oPath[i][1]:
						if oPath[i][0] + 1 == oPath[i+1][0]:
							moves.append([1,0])
						elif oPath[i][0] - 1 == oPath[i+1][0]:	
							moves.append([-1,0])
				'''
				#that and then publish it
				for i in range(len(optimalPath)):
					self.resPub.publish(optimalPath[i])		
		
				break
				#are you sure you want to break?
			else:

				#find out all the neighbors
				#these will be the successors		
				if self.is_Wall_Pit_InMap(currX+1, currY):
					suc.append( [currX+1, currY] )
				if self.is_Wall_Pit_InMap(currX-1, currY):
					suc.append( [currX-1, currY] )
				if self.is_Wall_Pit_InMap(currX, currY+1):
					suc.append( [currX, currY+1] )
				if self.is_Wall_Pit_InMap(currX, currY-1):
					suc.append( [currX, currY-1] )
				
				for i in range (len (suc) ):
					suc[i][0] = ncurrX
					suc[i][1] = ncurrY	

					if self.isNewNode(ncurrX, ncurrY):
						self.mmap[ncurrX][ncurrY][3] = [currX, currY]
						addToOpenList(ncurrX, ncurrY)
						tempAdd = [ mmap[ncurrX][ncurrY][2] ]
						tempAdd.append(str(ncurrX)+ str(ncurrY))	
						h.heappush(allPaths, tempAdd) 
					#this case will never happen?
					elif self.isInOpenList(ncurrX, ncurrY):
						if self.mmap[ncurrX][ncurrY][1] > self.mmap[currX][currY][1] + 1:
							self.mmap[ncurrX][ncurrY][3] = [currX, currY]
							self.mmap[ncurrX][ncurrY][1] = 1 + self.mmap[currX][currY][1]
							self.mmap[ncurrX][ncurrY][2] = self.mmap[ncurrX][ncurrY][0] + self.mmap[ncurrX][ncurrY][1]
								
				#add the current to closed list
				self.addToClosedList(currX, currY)
