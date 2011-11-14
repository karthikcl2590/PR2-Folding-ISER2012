#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_geometry")
import rospy
import sys
from geometry_msgs.msg import PointStamped,PoseStamped
from geometric_shapes_msgs.msg import Shape as GeometricShape
from folding_msgs.msg import PolyStamped,Point2D,FoldTraj
from folding_srvs.srv import *
from folding_geometry.FoldingGUI import *
from shape_window import Geometry2D
from numpy import *
import tf
import time

ADJUST_FOLDLINE = False 	# Turn on to visually detect the fold line location and compensate for it

# "Bridges" 2D polygons and the 3D points / fold trajectories that they require
class PolyGUIBridge():
    def __init__(self):
        self.count = 0
        self.listener = tf.TransformListener()
        self.mode = rospy.get_param('~mode',"default")
        self.gui = FoldingGUI(name="poly_gui_bridge")
        self.gui.setFoldCallback(self.received_fold)
        self.gui.setGripperLimit(2)
        
        self.traj_pub = rospy.Publisher("trajectory_output",FoldTraj)
        self.scale_factor = self.x_offset = self.y_offset = self.poly_frame = False
        self.poly_points = []  
        now = time.localtime()
        self.logfile = open('/tmp/folding_%04d-%02d-%02d__%02d-%02d-%02d.log'%(now.tm_year,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec),'w')
        self.poly_sub = rospy.Subscriber("input",PolyStamped,self.poly_handler)
        self.start_time = rospy.Time.now()
    
	#Receives a stream of polygon vertices and updates the poly appropriately
    def poly_handler(self,stamped_poly):
        self.poly_frame = stamped_poly.header.frame_id
        self.z_offset = stamped_poly.z_offset
        points = [Geometry2D.Point(point.x,point.y) for point in stamped_poly.vertices]
        vertices = self.center_and_bound(points,500)
        poly = Geometry2D.Polygon(*vertices)
        self.poly_cache = poly
        cvPoly = CVPolygon(Colors.GREEN,self.gui.front(),poly)
        self.gui.clearShapes()
        self.gui.addCVShape(cvPoly)
        self.handle_automatic_folds(vertices)
    
	#Waits til it has received enough, then folds the article sketched
    def handle_automatic_folds(self,vertices):
        
        if len(vertices) == 10 and self.mode == "shirt":
            self.start_logging()
            self.gui.foldShirt_v3()
            self.stop_logging()
        elif len(vertices) == 10 and self.mode == "tee":
            self.start_logging()
            self.gui.foldTeeNoSleeve()
            self.stop_logging()
        elif len(vertices) == 7 and self.mode == "pants":
            self.start_logging()
            self.gui.foldPants_v2()
            self.stop_logging()
        elif len(vertices) == 4 and self.mode == "towel":
            self.start_logging()
            self.gui.foldTowelThirds()
            self.stop_logging()
        
	#Logging
    def start_logging(self):
        self.start_time = rospy.Time.now()
        msg = "Starting to execute fold of type %s"%self.mode
        rospy.loginfo(msg)
        self.logfile.write("%s\n"%msg)
    def stop_logging(self):
        self.end_time = rospy.Time.now()
        dur = self.end_time - self.start_time
        msg = "Finished %s. Duration: %d.%d seconds"%(self.mode,dur.secs,dur.nsecs)
        rospy.loginfo(msg)
        self.logfile.write("%s\n"%msg)
        
        
    # Centers the polygon and scales it appropriately, so it will fit in the window    
    def center_and_bound(self,points,bound):
        avgx = sum([pt.x() for pt in points]) / len(points)
        avgy = sum([pt.y() for pt in points]) / len(points)
        centered_pts = [Geometry2D.Point(pt.x()-avgx,pt.y()-avgy) for pt in points]
        scale = max([max(fabs(pt.x()),fabs(pt.y())) for pt in centered_pts])
        self.scale_factor = bound/(2.3*scale)
        self.x_offset = bound/2 - avgx*bound/(2.3*scale)
        self.y_offset = bound/2 - avgy*bound/(2.3*scale)
        return [Geometry2D.Point(pt.x()*bound/(2.3*scale)+bound/2,pt.y()*bound/(2.3*scale)+bound/2) for pt in centered_pts]
    
	# Convert a 2D vertex to a 3D world point
    def convert_to_world_frame(self,pt2D):
        newx = pt2D.x() - self.x_offset
        newy = pt2D.y() - self.y_offset
        newx /= self.scale_factor
        newy /= self.scale_factor
        newPt = PointStamped()
        newPt.header.stamp = rospy.Time.now()
        newPt.header.frame_id = self.poly_frame
        newPt.point.x = -1*newy
        newPt.point.y = -1*newx
        newPt.point.z = self.z_offset
        return newPt
        
	# Project a 3D world point onto the 2D window
    def convert_from_world_frame(self,pt3D):
        now = rospy.Time.now()
        self.listener.waitForTransform(self.poly_frame,pt3D.header.frame_id,now,rospy.Duration(20.0))
        newpt = self.listener.transformPoint(self.poly_frame,pt3D)
        x = newpt.point.y * -1
        y = newpt.point.x * -1
        x *= self.scale_factor
        y *= self.scale_factor
        x += self.x_offset
        y += self.y_offset
        return Geometry2D.Point(x,y)
        
    #Called by the GUI once a fold has been drawn. Tells me to start folding.    
    def received_fold(self,foldline,active_vertices,red):
        pre_fold_world_frame = []
        start_fold_world_frame = []
        quarter_fold_world_frame = []
        weight_fold_world_frame = []
        mid_fold_world_frame = []
        end_fold_world_frame = []
        tilts = []
        
        if self.mode == "pants" and self.count == 1:
            if len(active_vertices) > 1:
                active_vertices.remove(min(active_vertices,key=lambda v: v.x))
                
        
        #Active vertices will be sorted by order of pickup
        for active_vert in sorted(active_vertices,key=lambda pt: -1 * Geometry2D.ptLineDisplacement(pt,foldline).length()):
            displ = Geometry2D.ptLineDisplacement(active_vert,foldline)
            if displ.length() == 0:
                start_fold = self.convert_to_world_frame(active_vert)
            else:
                start_fold = self.convert_to_world_frame(active_vert)
            in_amt = 0.015
            start_fold = self.convert_to_world_frame(displ.extrapolate(in_amt*self.scale_factor/displ.length()))
            start_fold.point.z -= 0.00
            start_fold_world_frame.append(start_fold)
            pre_fold = self.convert_to_world_frame(displ.extrapolate(-0.015*self.scale_factor/displ.length()))
            pre_fold.point.z += 0.04
            pre_fold_world_frame.append(pre_fold)
            quarter_fold = self.convert_to_world_frame(displ.extrapolate(1.0/4.0))
            quarter_fold.point.z += displ.length()/(4*abs(self.scale_factor))
            quarter_fold_world_frame.append(quarter_fold)
            weight_fold = self.convert_to_world_frame(displ.extrapolate(1.0/2.0))
            weight_fold.point.z += displ.length()/(2*abs(self.scale_factor))
            weight_fold_world_frame.append(weight_fold)
            mid_fold = self.convert_to_world_frame(displ.end())
            mid_fold.point.z += displ.length()/abs(self.scale_factor)
            mid_fold_world_frame.append(mid_fold)
            end_fold = self.convert_to_world_frame(displ.extrapolate(2.00 - 0.01*self.scale_factor/displ.length()))#Was 2.00
            end_fold.point.z += 0.03 #was 0.035      
            #if self.count == 0 or self.count == 3:
            #    end_fold = mid_fold
            end_fold_world_frame.append(end_fold)
            now = rospy.Time.now()
            self.listener.waitForTransform("base_footprint",mid_fold.header.frame_id,now,rospy.Duration(10.0))
            mid_fold_base = self.listener.transformPoint("base_footprint",mid_fold)
            start_fold_base = self.listener.transformPoint("base_footprint",start_fold)
            dx = mid_fold_base.point.x - start_fold_base.point.x
            dy = mid_fold_base.point.y - start_fold_base.point.y
            tilt = arctan(dy /(dx+0.00001))
            if dx < 0 and not red:
                tilt += pi
            elif dx > 0 and red:
                tilt += pi
            tilt = (tilt + pi) % (2*pi) - pi
            tilts.append(tilt)
        
        fold_traj = FoldTraj()
        fold_traj.approach_points = pre_fold_world_frame
        fold_traj.grip_points = start_fold_world_frame
        fold_traj.quarter_points = quarter_fold_world_frame
        fold_traj.weight_points = weight_fold_world_frame
        fold_traj.vertical_points = mid_fold_world_frame
        fold_traj.goal_points = end_fold_world_frame
        fold_traj.tilts = tilts
        fold_traj.red = red
        fold_traj.smooth_center = self.convert_to_world_frame(foldline.center())
        fold_traj.smooth_edges = [self.convert_to_world_frame(pt) for pt in sorted(foldline.pts(),key=lambda pt: pt.x())]
        if foldline.length() < 0.14:
            fold_traj.ignore_smooth = True
            
        
        success = self.send_traj(fold_traj)
		if ADJUST_FOLDLINE:
        	adjusted_foldline = self.adjustFoldline(intended=foldline)
		else:
			adjusted_foldline = foldline
        self.count = self.count + 1
        return adjusted_foldline
        
    def adjustFoldline(self,intended):
        #intended.expand(10)
        start = self.convert_to_world_frame(intended.start())
        end = self.convert_to_world_frame(intended.end())
        try:
            srv = rospy.ServiceProxy('adjust_fold',AdjustFold)
            resp = srv(start,end)
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
            return False
        new_start = self.convert_from_world_frame(resp.start)
        new_end = self.convert_from_world_frame(resp.end)
        newfold = Geometry2D.DirectedLineSegment(new_start,new_end)
        newfold.expand(0.1)
        return newfold

    def send_traj(self,fold_traj):
        try:
            srv = rospy.ServiceProxy('execute_fold',ExecuteFold)
            resp = srv(ExecuteFoldRequest(fold_traj=fold_traj))
            return True
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
            return False
            
def main(args):
    rospy.init_node("poly_gui_bridge")
    pg = PolyGUIBridge()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
