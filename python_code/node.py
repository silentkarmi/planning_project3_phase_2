#!/usr/bin/env python3
# Author @ Kartikeya Mishra

from dataclasses import dataclass
import math

from constants import CONSTANT
from utility import Utility

@dataclass
class Node:
    ACTION_SET = [-60, -30, 0, 30, 60]
    
    coord: tuple()
    cost2come: int = 0
    cost2go: int = 0
    
    def __init__(self, coord, parent):
        self.coord = coord
        self.parentNode = parent
        self.cost2come = 0
        self.cost2go = 0
        self.calculateCost2Go()
        
    def __lt__(self, other):
        return self.cost2come + self.cost2go < other.cost2come + other.cost2go
        
    def __le__(self, other):
        return self.cost2come + self.cost2go <= other.cost2come + other.cost2go
    
    def calculateCost2Go(self):
        x, y, _ = self.coord
        x1, y1, _ = CONSTANT.GOAL_NODE
        
        #manhattan heuristic
        self.cost2go = abs(x1 - x) + abs(y1 - y)
        
        #euclidean heuristic
        # self.cost2go = round(math.sqrt((x1 - x)**2 + (y1 - y)**2))
    
    @staticmethod
    def actionCost():
        return CONSTANT.VECTOR_LEN
        
    @staticmethod   
    def isCoordValid(coord):
        x, y,_ = coord
        if x < CONSTANT.CANVAS_WIDTH - CONSTANT.CLEARANCE and \
        y < CONSTANT.CANVAS_HEIGHT - CONSTANT.CLEARANCE and \
        x > CONSTANT.CLEARANCE  and \
        y > CONSTANT.CLEARANCE:
            return True
        else:
            return False
        
    def createNodes(self):
        nodes = []
        for action in Node.ACTION_SET:
            objNode = self._createNode(action)
            if objNode != None:
                nodes.append(objNode)    
        return nodes
    
    def _createNode(self, action):
        
        objNode = None
        x, y, originalThetha = self.coord
        
        thetha = Utility.actionInDegree(originalThetha, action)
        newX = x + CONSTANT.VECTOR_LEN * math.cos(math.radians(thetha))
        newY = y + CONSTANT.VECTOR_LEN * math.sin(math.radians(thetha))
        
        res = (newX, newY, thetha)
        
        if Node.isCoordValid(res):
            # if res != self.coord:
                objNode = Node(res, self)
                #calculating the cost2come by adding the parent and action cost2come
                objNode.cost2come = round(Node.actionCost() + self.cost2come, 3)
            
        return objNode
    
    def isEqual(self, other):
        return (round(self.coord[0]) == round(other.coord[0]) and
                round(self.coord[1]) == round(other.coord[1]) and
                Utility.actionInDegree(self.coord[2]) == Utility.actionInDegree(other.coord[2]))
