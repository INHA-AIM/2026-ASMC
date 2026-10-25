"""Explicit MORAI ActorState ENU heading -> local UTM52 map contract.

Actor/bounding-box positions already use local map coordinates. Only heading
is rotated; matching raw IMU and ActorState headings does not prove grid yaw.
"""
import math
from pyproj import Transformer


class Coordinates:
    def __init__(self,origin):
        self.origin=origin
        self.inverse=Transformer.from_crs(32652,4326,always_xy=True)
        self.forward=Transformer.from_crs(4326,32652,always_xy=True)

    def yaw_offset(self,x,y):
        lon,lat=self.inverse.transform(self.origin[0]+x,self.origin[1]+y)
        a=self.forward.transform(lon,lat-1e-5);b=self.forward.transform(lon,lat+1e-5)
        return math.atan2(b[1]-a[1],b[0]-a[0])-math.pi/2

    def grid_yaw(self,x,y,enu_degrees):
        return math.remainder(math.radians(enu_degrees)+self.yaw_offset(x,y),2*math.pi)

    def enu_yaw_degrees(self,x,y,grid_radians):
        return math.degrees(math.remainder(grid_radians-self.yaw_offset(x,y),2*math.pi))
