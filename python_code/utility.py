#!/usr/bin/env python3
# Author @ Kartikeya Mishra @ Chang-Hong Chen

import sys 
from dataclasses import dataclass
from constants import CONSTANT

@dataclass
class Utility:
    @staticmethod
    def getCoordinatesInWorldFrame(coord):
        """
            Can be used for transforming x, y cart coord to w, h image coord, vice versa.
        """
        x = coord[0]
        y = coord[1]
            
        y = CONSTANT.ORIGIN_POINT_OFFSET - y
        return (int(round(x)), int(round(y)))
    
    @staticmethod
    def actionInDegree(thetha, d_thetha = 0):
        result = thetha + d_thetha
        remainder = abs(result) % 360
        
        if result < 0:
            result = -1 * remainder
        else:
            result = remainder
            
        # if result >= 360:
        #     result = result - 360
        # elif result < 0:
        #     result = 360 + result
        
        return result

    @classmethod
    def run_spinning_cursor(self): 

        def spinning_cursor():
            while True:
                for cursor in '|/-\\':
                    yield cursor

        try: 
            if self.count % 100 == 0: 
                sys.stdout.write(next(self.spinner))
                sys.stdout.flush()
                sys.stdout.write('\b')
            self.count += 1
        except AttributeError:
            self.spinner = spinning_cursor()
            self.count = 0
            
         
