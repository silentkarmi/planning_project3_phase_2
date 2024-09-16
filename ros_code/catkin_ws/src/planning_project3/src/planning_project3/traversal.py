#!/usr/bin/env python3
# Author @ Kartikeya Mishra @ Chang-Hong Chen

import cv2
import heapq
from dataclasses import dataclass

from . constants import CONSTANT
from . canvas import Canvas
from . node import Node
from . utility import Utility
from . circleObstacle import CircleObstacle
from . rectangularObject import RectangularObstacle

import numpy as np

MAP_RESOLUTION_SCALE = 10
MAP_THRESOLD_REGION = int(0.5 * MAP_RESOLUTION_SCALE)
ANGLE_RESOLUTION = 10 # degree

@dataclass
class Traversal:

    def __init__(self):
        self._closedList = set() 
        self._closeListNodes = []
        self._openList = []
        self._listSolution = []
        
        self.canvaArea = Canvas()
        
        objRect = RectangularObstacle([(0, 950 * CONSTANT.SCALE ),
                                       (0, 1000 * CONSTANT.SCALE),
                                       (CONSTANT.CANVAS_WIDTH, 1000 * CONSTANT.SCALE),
                                       (CONSTANT.CANVAS_WIDTH, 950 * CONSTANT.SCALE)])
        self.canvaArea.addObstacle(objRect)
        
        objRect = RectangularObstacle([(925 * CONSTANT.SCALE, 425 * CONSTANT.SCALE),
                                       (925 * CONSTANT.SCALE, 575 * CONSTANT.SCALE),
                                       (1075 * CONSTANT.SCALE, 575 * CONSTANT.SCALE),
                                       (1075 * CONSTANT.SCALE, 425 * CONSTANT.SCALE)])
        self.canvaArea.addObstacle(objRect)
        
        objRect = RectangularObstacle([(1425 * CONSTANT.SCALE, 650 * CONSTANT.SCALE),
                                       (1425 * CONSTANT.SCALE, 800 * CONSTANT.SCALE),
                                       (1575 * CONSTANT.SCALE, 800 * CONSTANT.SCALE),
                                       (1575 * CONSTANT.SCALE, 650 * CONSTANT.SCALE)])
        self.canvaArea.addObstacle(objRect)
        
        objRect = RectangularObstacle([(1425 * CONSTANT.SCALE, 200 * CONSTANT.SCALE),
                                       (1425 * CONSTANT.SCALE, 350 * CONSTANT.SCALE),
                                       (1575 * CONSTANT.SCALE, 350 * CONSTANT.SCALE),
                                       (1575 * CONSTANT.SCALE, 200 * CONSTANT.SCALE)])
        self.canvaArea.addObstacle(objRect)
        
        objRect = RectangularObstacle([(1925 * CONSTANT.SCALE, 425 * CONSTANT.SCALE),
                                       (1925 * CONSTANT.SCALE, 575 * CONSTANT.SCALE),
                                       (2075 * CONSTANT.SCALE, 575 * CONSTANT.SCALE),
                                       (2075 * CONSTANT.SCALE, 425 * CONSTANT.SCALE)])
        self.canvaArea.addObstacle(objRect)
        
        objRect = RectangularObstacle([(0, 0),
                                       (0, 50 * CONSTANT.SCALE),
                                       (CONSTANT.CANVAS_WIDTH, 50 * CONSTANT.SCALE),
                                       (CONSTANT.CANVAS_WIDTH, 0)])
        self.canvaArea.addObstacle(objRect)
        
        objRect = RectangularObstacle([(CONSTANT.CANVAS_WIDTH - 10, 50 * CONSTANT.SCALE),
                                       (CONSTANT.CANVAS_WIDTH - 10, 950 * CONSTANT.SCALE),
                                       (CONSTANT.CANVAS_WIDTH, 950 * CONSTANT.SCALE),
                                       (CONSTANT.CANVAS_WIDTH, 50 * CONSTANT.SCALE)])
        self.canvaArea.addObstacle(objRect)
        
        self.canvaArea.formObstaclesMap()

        # Used for recording closed node in a higher resolution map
        self.closedNodeMap = np.zeros((CONSTANT.CANVAS_HEIGHT * MAP_RESOLUTION_SCALE, 
                                       CONSTANT.CANVAS_WIDTH * MAP_RESOLUTION_SCALE, 
                                       (360 // ANGLE_RESOLUTION)+ 1), np.uint8)
        
        self.startNode = None
        self.endNode = None
        self.solutionNode = None
        
    def expandNodesAndPushIntoWorkspace(self, node):
        listNodes = node.createNodes()
        for subNode in listNodes:
            self.pushNode(subNode)       

    def isNodeClosed(self, node): 
        # Transform x, y cart coord to w, h image coord
        w, h = Utility.getCoordinatesInWorldFrame(node.coord)
        angle = int(Utility.actionInDegree(node.coord[2]) // ANGLE_RESOLUTION)
        return self.closedNodeMap[int(h * MAP_RESOLUTION_SCALE), 
                                  int(w * MAP_RESOLUTION_SCALE), 
                                  angle] != 0

    def pushNode(self, node):
        if node != None:
            isNodeSafe = self.canvaArea.isOutsideObstacleSpaceByMap(node.coord)
            
            if isNodeSafe:
                if not self.isNodeClosed(node):
                    heapq.heappush(self._openList, node)

                
    def isThisGoalNode(self, nodeToCheck):
        xcentre = self.endNode.coord[0]
        ycentre = self.endNode.coord[1]
        x = nodeToCheck.coord[0]
        y = nodeToCheck.coord[1]
        in_goal = x > xcentre and (y > ycentre - CONSTANT.GOAL_THRESOLD and y < ycentre + CONSTANT.GOAL_THRESOLD)
        return in_goal

    def AddtoClosedNodeMap(self, node):
        # Transform x, y cart coord to w, h image coord
        w, h = Utility.getCoordinatesInWorldFrame(node.coord)
        matrix_x = int(h * MAP_RESOLUTION_SCALE - MAP_THRESOLD_REGION)
        matrix_y = int(w * MAP_RESOLUTION_SCALE - MAP_THRESOLD_REGION)
        matrix_degree = int(Utility.actionInDegree(node.coord[2]) // ANGLE_RESOLUTION)
        self.closedNodeMap[matrix_x, matrix_y, matrix_degree] = 1
        
        for counter_x in range(1, 11):
            for counter_y in range(1, 11):
                self.closedNodeMap[matrix_x + counter_x , 
                                   matrix_y + counter_y, matrix_degree] = 1
 
    
    def createNodeTree(self):
        print("Generating Node Tree...")
        self.pushNode(self.startNode)
        while(self._openList):

            # Run spinning cursor while creating node tree
            Utility.run_spinning_cursor()
            
            # pops an element from the top of the list
            tempNode = heapq.heappop(self._openList)     

            if self.isNodeClosed(tempNode):
                continue

            # self.canvaArea.drawNode(tempNode)
            
            self._closeListNodes.append(tempNode)
            self._closedList.add((round(tempNode.coord[0]),
                                        round(tempNode.coord[1])))  

            # Add closed node to a higher resolution map
            self.AddtoClosedNodeMap(tempNode)

             
            if(self.isThisGoalNode(tempNode)):
                # print("Total cost: ", tempNode.cost2come)
                self.solutionNode = tempNode
                break
            
            self.expandNodesAndPushIntoWorkspace(tempNode)
            
        else:
            print("SOLUTION NOT FOUND")
            
    def drawNodeTree(self):
        print("Drawing Node Tree...")
        counter = 0
        for tempNode in self._closeListNodes:
            self.canvaArea.drawNode(tempNode)
            
    def backTrack(self):
        print("Backtracking...")
        self._listSolution = []
        tempNode = self.solutionNode
        while tempNode != None:
            self._listSolution.append(tempNode)
            tempNode = tempNode.parentNode

    def getSolution(self):
        actionsList = []
        for node in self._listSolution[::-1]:
            if (len(node.coord) < 5): 
                continue
            actionsList.append(node.coord[4])
        
        actionsList.append((0,0)) #For stopping the robot

        return actionsList

    def drawSolution(self):  
        print("Drawing the solution...")     
        
        mobileRobotStepSize = int(CONSTANT.MOBILE_ROBOT_RADIUS)
        if mobileRobotStepSize <= 0:
            mobileRobotStepSize = 1
        
        # self.canvaArea.drawMobileRobot(self.startNode, CONSTANT.COLOR_YELLOW)
        # for node in self._listSolution[::mobileRobotStepSize]:
        #     self.canvaArea.drawMobileRobot(node)
        # self.canvaArea.drawMobileRobot(self.solutionNode, CONSTANT.COLOR_GREEN)
            
        for node in self._listSolution[::-1]:
            # print(node.coord)
            self.canvaArea.drawNode(node, CONSTANT.COLOR_BLUE)
