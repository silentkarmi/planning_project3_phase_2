#!/usr/bin/env python3
# Author @ Kartikeya Mishra

from abc import ABC, abstractmethod
from dataclasses import dataclass

@dataclass
class ObstacleInterface(ABC):
    
    # if the coordinate is outside of the obstacle
    @abstractmethod
    def isOutside(self, coord):
        pass
    
    @abstractmethod
    def draw(self, canvasArea):
        pass