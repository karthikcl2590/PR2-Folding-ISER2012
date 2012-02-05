import heapq

import roslib
roslib.load_manifest("folding_geometry")
import rospy
import sys
from geometry_msgs.msg import PointStamped,PoseStamped
from arm_navigation_msgs.msg import Shape as GeometricShape
from folding_msgs.msg import PolyStamped,Point2D,FoldTraj
from shape_window import Geometry2D
from folding_geometry.msg import gPoint
import tf

scale_factor = 5/0.0254
poly_frame = "base_footprint"
gui_frame = "stations/table_front_scoot" # same as world_frame
z_offset = 0.80

listener = None
mode = "towel"
TfTime = 0.0

BUSY = False


class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.

      Note that this PriorityQueue does not allow you to change the priority
      of an item.  However, you may insert the same item multiple times with
      different priorities.
    """
    def  __init__(self):
        self.heap = []

    def __getitem__(self,num):
        return self.heap[num][1]
        
    def push(self, item, priority):
        pair = (priority,item)
        heapq.heappush(self.heap,pair)

    def pop(self):
        (priority,item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

def dupl_PointStamped(pt3D):
    newPt = PointStamped()
    newPt.header.stamp = rospy.Time.now()
    newPt.header.frame_id = pt3D.header.frame_id
    newPt.point.x = pt3D.point.x
    newPt.point.y = pt3D.point.y
    newPt.point.z = pt3D.point.z
    return newPt

def dupl_gPoint(pt):
    newPt = gPoint()
    newPt.ps = dupl_PointStamped(pt.ps)
    newPt.hangedge = pt.hangedge
    return newPt

def convert_to_world_frame(pt2D):
    global TfTime
    # first scale from gui to gui_frame
    world_y = -(pt2D.x()-95-150)/scale_factor
    world_x = -(pt2D.y()-550 + 100)/scale_factor
    newPt = gPoint()    
    newPt.ps.header.stamp=rospy.Time(0)
    newPt.ps.header.frame_id = gui_frame
    newPt.ps.point.x = world_x
    newPt.ps.point.y = world_y
    newPt.ps.point.z = z_offset - pt2D.get_zOffset()/scale_factor
    newPt.hangedge = pt2D.get_plane()
    
    # now convert to base_footprint
    #t = rospy.get_time()
    #listener.waitForTransform(poly_frame,newPt.ps.header.frame_id,newPt.ps.header.stamp,rospy.Duration(10.0))
    #newPt.ps = listener.transformPoint(poly_frame,newPt.ps)
    #TfTime += rospy.get_time() - t 
    print "convert_to_world_frame", (pt2D.x(),pt2D.y()), "------->",(newPt.ps.point.x,newPt.ps.point.y,newPt.ps.point.z)
    return newPt

# Project a 3D world point onto the 2D window                                                                                   
def convert_from_world_frame(pt3D):            
    global TfTime
    # convert to base_footprint -- Hack. 
    world_x = -pt3D.y
    world_y = -pt3D.x
    #print "original point in ",poly_frame,(world_x,world_y)
    # convert to gui_frame so that it draws nicely 
    pt = PointStamped()
    pt.header.stamp = rospy.Time(0)
    pt.header.frame_id = poly_frame
    pt.point.x = world_x
    pt.point.y = world_y
    pt.point.z = z_offset

    t = rospy.get_time()    
    #listener.waitForTransform(gui_frame,pt.header.frame_id,pt.header.stamp,rospy.Duration(10.0))    
    pt_transformed = listener.transformPoint(gui_frame,pt)
    TfTime += rospy.get_time() - t

    world_x = pt_transformed.point.x
    world_y = pt_transformed.point.y
    #print "final point in ",gui_frame,(world_x,world_y)
    # scale to gui
    x = -scale_factor*world_y + 95 +150
    y = -scale_factor*world_x + 550 - 100
    return Geometry2D.Point3d(Geometry2D.Point(x,y),0,None)
