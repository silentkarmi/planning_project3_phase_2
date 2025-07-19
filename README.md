<pre>
It uses A* algorithm to find a path for the given map
--------------------------------------------------------------------------------------------------
>> When obstacle objects are drawn, they are inflated internally but not drawn on the screen.
>> Mobile robot is considered as a point robot during calculation
   and obstacles are inflated by the radius of the mobile robot during calculation
>> Color Meaning
    - Red : obstacle
    - White : Straight Line
    - Black : clearance from the boundary and the obstacles (in the end of traversal) / empty space
    - Yellow : Curving Left side
    - LightBlue: Curving Right side
    
>> Even if, the solution found in seconds, 
   but worst Condition of drawing of whole graph traversal can take upto 5 minutes
>> Structure of the Program
    - main.py - main starting point of the Program
    - node.py - each coordinate is considered a node
    - canvas.py - deals with drawing opencv related details
    - obstacles package - contains the boomerang polygon obstacle, circle obstacle and regular hexagon obstacle related files
    - utility.py - functions which used throughout Program
    - constants.py - declare constants which used throughout the Program
    - traversal.py - this is where the A* algorithm is implemented
>> This project has been done in a GROUP:
    - Kartikeya Mishra (UID: 118106755)
    - Chang-Hong Chen (UID: 117397857)
>> Note: Python3 is used to write the code
>> Command to run from terminal:
python3 main.py
</pre>
