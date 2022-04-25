#!/usr/bin/env python3
# Author @ Kartikeya Mishra

from dataclasses import dataclass
import math
import numpy as np

from . constants import CONSTANT
from . utility import Utility

@dataclass
class Node:
    ACTION_SET = CONSTANT.get_action_sets() # list of (vL, vR)
    
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
        return self.cost2come + 0.90  * self.cost2go < other.cost2come + 0.90 * other.cost2go
        
    def __le__(self, other):
        return self.cost2come + 0.90  * self.cost2go <= other.cost2come + 0.90  * other.cost2go
    
    def calculateCost2Go(self):
        x = self.coord[0]
        y = self.coord[1]
        x1 = CONSTANT.GOAL_NODE[0]
        y1 = CONSTANT.GOAL_NODE[1]
        
        reward = 0
        
        # if len(self.coord) > 4:
        #     action = self.coord[4]
    
        #     if action[0] == action[1]:
        #         reward = 1

        #manhattan heuristic
        self.cost2go = abs(x1 - x) + abs(y1 - y) + reward
        
        #euclidean heuristic
        # self.cost2go = round(math.sqrt((x1 - x)**2 + (y1 - y)**2))
    
    @staticmethod
    def actionCost(delthaThetha, R):
        cost2come = 0
        if delthaThetha == 0:
            cost2come = R
        else:
            cost2come = (delthaThetha / 360) * 2 * math.pi * R
            
        return cost2come
        
    @staticmethod   
    def isCoordValid(coord):
        x = coord[0]
        y = coord[1]
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
        
        x = self.coord[0]
        y = self.coord[1]
        thetha = math.radians(self.coord[2])
        
        objNode = None
        vL, vR = action
        
        if vL < 0 and vR < 0:
            pass # don't handle backward motion
        else:
            # forward motion, slight turn and sharp turns
            R = 0
            newX = 0
            newY = 0
            if vL != vR:
                # DIFFERENTIAL KINEMATICS
                R = abs(0.5 * ((vL + vR) / (vR - vL))) + CONSTANT.ROBOT_RADIUS_INNER
                angle = (vR - vL) / (2 * CONSTANT.ROBOT_RADIUS_INNER)
                # angle  = thetha + angle * CONSTANT.TIME_INTERVAL
                # newThetha = Utility.actionInDegree(math.degrees(angle))
                
                # newX = x + R * math.cos(angle)
                # newY = y + R * math.sin(angle)
                
                #JACOBIAN KINTEMATICS
                omega_dt = angle * CONSTANT.TIME_INTERVAL
                
                ICCx = x - R * math.sin(thetha)
                ICCy = y + R * math.cos(thetha)
                
                ICC_ORIGIN = np.array([[x - ICCx],
                                      [y - ICCy],
                                      [thetha]])
                
                ICC = np.array([[ICCx],
                               [ICCy],
                               [omega_dt]])
                
                ROT_ICC = np.array([[math.cos(omega_dt), -math.sin(omega_dt), 0],
                                   [math.sin(omega_dt), math.cos(omega_dt) , 0],
                                   [                 0,                  0 , 1]])
                
                new_pose = ROT_ICC @ ICC_ORIGIN + ICC
                newX = new_pose[0][0]
                newY = new_pose[1][0]
                newThetha = new_pose[2][0]
                newThetha = Utility.actionInDegree(math.degrees(newThetha))
                
            else:
                R = vL * CONSTANT.TIME_INTERVAL
                newX = x + R  * math.cos(thetha)
                newY = y + R * math.sin(thetha)
                newThetha = Utility.actionInDegree(math.degrees(thetha))
            
            deltaThetha = abs(newThetha - self.coord[2])
            res = (newX, newY, newThetha, R, action, Node.actionCost(deltaThetha, R))
            
            if Node.isCoordValid(res):
                # if res != self.coord:
                    objNode = Node(res, self)
                    #calculating the cost2come by adding the parent and action cost2come
                    objNode.cost2come = round(Node.actionCost(deltaThetha, R) + self.cost2come, 3)
            
        return objNode
    
    def isEqual(self, other):
        return (round(self.coord[0]) == round(other.coord[0]) and
                round(self.coord[1]) == round(other.coord[1]) and
                Utility.actionInDegree(self.coord[2]) == Utility.actionInDegree(other.coord[2]))
