#!/usr/bin/env python3
# Author @ Kartikeya Mishra @ Chang-Hong Chen

from constants import CONSTANT
import cv2
from traversal import Traversal
from node import Node
import time
import math

if __name__ == "__main__":
    
    while (1):
        
        # startXCoord, startYCoord, startGoalThetha = (input("\nStart X, Y, Theta [Add Space in between the values]:")).split()
        # startXCoord, startYCoord, startGoalThetha = int(startXCoord), int(startYCoord), int(startGoalThetha)
        startXCoord, startYCoord, startGoalThetha = int(16), int(16), 0
        
        
        CONSTANT.START_NODE = (startXCoord, startYCoord, startGoalThetha, None)
        
        if startGoalThetha % 30 != 0:
                print("Invalid Start Goal Thetha, should be multiple of 30")
                continue
        
        objTraversal = Traversal()
        if objTraversal.canvaArea.isOutsideObstacleSpace(CONSTANT.START_NODE):
                
            # endXCoord, endYCoord, endGoalThetha = (input("End X, Y, Theta [Add Space in between the values]:")).split()
            # endXCoord, endYCoord, endGoalThetha = int(endXCoord), int(endYCoord), math.radians(float(endGoalThetha))
        
            endXCoord, endYCoord, endGoalThetha = int(150), int(150), math.radians(0.0)
            
            CONSTANT.GOAL_NODE = (endXCoord, endYCoord, endGoalThetha, None)
            
            if (objTraversal.canvaArea.isOutsideObstacleSpace(CONSTANT.GOAL_NODE)):
                
                # CONSTANT.WALL_CLEARANCE = int(input("What's the Clearance from the Mobile Robot:"))
                
                objTraversal.endNode = Node(CONSTANT.GOAL_NODE , None)
                objTraversal.startNode = Node(CONSTANT.START_NODE, None)
                            
                start_time = time.time()
                objTraversal.createNodeTree()
                objTraversal.backTrack() # backtracks the solution
                print(f"Total Nodes Searched:{len(objTraversal._closedList)}")
                print("--- %s seconds for finding the solution ---" % (time.time() - start_time))
                
                # VISUALIZATION PART
                objTraversal.canvaArea.drawObstacles()
                objTraversal.drawNodeTree() # draws the node tree after solution is found
                objTraversal.drawSolution() # draws the solution
                cv2.waitKey(0)
                quit()
            else:
                print("Invalid End Coordinates. Try Again.\n")
        else:
            print("Invalid Start Coordinates. Try Again.\n")
    
    
    
    
