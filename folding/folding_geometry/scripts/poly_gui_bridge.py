#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_geometry")
import rospy
import sys
from geometry_msgs.msg import PointStamped,PoseStamped
from geometric_shapes_msgs.msg import Shape as GeometricShape
from folding_msgs.msg import PolyStamped,Point2D,FoldTraj
from pr2_simple_arm_motions import GripUtils
from pr2_simple_motions_srvs.srv import *
import StanceUtils
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
        self.mode = "towel" #rospy.get_param('~mode',"default")
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
        rospy.loginfo("received a point")
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
            [bl,tl,tr,br] = vertices
            
            #self.executeFold([bl,br],[tl,tr],"blue","blue")
            #self.testFold([bl,tl],[br,tr])
            #self.gui.foldTowelThirds()
            #self.drag_towel(vertices)
            self.towelDragDemo(bl,tl,tr,br)
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
    
        # Duplicated PointStamped
    def dupl_PointStamped(self,pt3D):
        newPt = PointStamped()
        newPt.header.stamp = rospy.Time.now()
        newPt.header.frame_id = self.poly_frame
        newPt.point.x = pt3D.point.x
        newPt.point.y = pt3D.point.y
        newPt.point.z = pt3D.point.z
        return newPt
        
    
    #Called by the GUI once a fold has been drawn. Tells me to start folding.    
    def received_fold(self,foldline,active_vertices,red,fold_type):
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
                
        if fold_type == 'hang':
            for active_vert in sorted(active_vertices,key=lambda pt: -1 * Geometry2D.ptLineDisplacement(pt,foldline).length()):
                displ = Geometry2D.ptLineDisplacement(active_vert,foldline)                
                in_amt = 0.015
                start_fold = self.convert_to_world_frame(displ.extrapolate(in_amt*self.scale_factor/displ.length()))
                start_fold.point.z -= 0.00
                start_fold_world_frame.append(start_fold)
                pre_fold = self.convert_to_world_frame(displ.extrapolate(-0.015*self.scale_factor/displ.length()))
                pre_fold.point.z += 0.04
                pre_fold_world_frame.append(pre_fold)
                quarter_fold = pre_fold
                quarter_fold_world_frame.append(quarter_fold)
                weight_fold = pre_fold
                weight_fold_world_frame.append(weight_fold)
                mid_fold = pre_fold
                mid_fold_world_frame.append(mid_fold)
                end_fold = self.convert_to_world_frame(displ.extrapolate(-2.00 + 0.01*self.scale_factor/displ.length()))
                end_fold.point.z += 0.03
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
                
        else:
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

    def executeActions(self,states):
        """
        now execute the actions returned by the search
        """
        for state in states:
            action = state.action
            if (action[0] == "fold"):
                SUCCESS = self.executeFold(state.gripPts,state.endPts)
            elif (action[0] == "drag"):
                SUCCESS = self.executeDrag(state.gripPts,state.endPts)            
                
            if not SUCCESS:
                rospy.loginfo("Failure to execute %s",action[0])
                break
    
    def executeDrag(self,gripPts,endPts,color='blue'):
        """
        drag from gripPts to endPts
        """          
        startpoints = []
        for vertex in gripPts:
            pt_world = self.convert_to_world_frame(vertex)
            now = rospy.Time.now()
            self.listener.waitForTransform("base_footprint",pt_world.header.frame_id,now,rospy.Duration(10.0))            
            pt_transformed = self.listener.transformPoint("base_footprint",pt_world)                        
            startpoints.append(pt_transformed)

        

        #print "grabbed (x,y,z)",startpoints[0].point.x,startpoints[0].point.y,startpoints[0].point.z
        

        if not GripUtils.grab_points(point_l=startpoints[0],roll_l=-pi/2,yaw_l=-pi/3,pitch_l=pi/4,x_offset_l=-0.01, z_offset_l=0.005,approach= True,
                                     point_r=startpoints[1],roll_r=pi/2,yaw_r= pi/3,pitch_r=pi/4,x_offset_r=-0.01, z_offset_r=0.005):
            print "failure"
            return False
        else:
            print "grabbed (x,y,z)",startpoints[0].point.x,startpoints[0].point.y,startpoints[0].point.z
            print "done"

        midpoints = []
        for vertex in endPts:
            pt_world = self.dupl_PointStamped(vertex)#self.convert_to_world_frame(vertex)            
            midpoints.append(pt_world)

        frame_l = frame_r = midpoints[0].header.frame_id
        if not GripUtils.go_to_multi (x_l=midpoints[0].point.x,y_l=midpoints[0].point.y,z_l=midpoints[0].point.z,roll_l=-pi/2,yaw_l=-pi/2,pitch_l=pi/4,grip_l=True,frame_l=frame_l
                                                                      ,x_r=midpoints[1].point.x, y_r= midpoints[1].point.y ,z_r= midpoints[1].point.z ,roll_r=pi/2,yaw_r=pi/2,pitch_r=pi/4,grip_r=True,frame_r=frame_r,dur=5):
            print "failure"
            #return False
        else:
            print "dragging done"

        if (color == 'blue'):
            GripUtils.open_grippers()
            
        return True

    def executeFold_right(self,gripPt,endPt,color_current='blue',color_next='blue'):
        """
        execute fold with right arm
        """
        print "points",gripPt,endPt
        if (color_current == 'blue'):
            startpoint = self.convert_to_world_frame(gripPt)    
            if not GripUtils.grab(x = startpoint.point.x,y=startpoint.point.y,z=startpoint.point.z,arm='r',
                                  roll=pi/2,yaw=pi/3,pitch=pi/4,approach= True,frame=startpoint.header.frame_id):
                print "failure"
                return False
        midpoint = self.dupl_PointStamped(endPt)#self.convert_to_world_frame(endPt)
        midpoint.point.x = (midpoint.point.x + startpoint.point.x)/2
        midpoint.point.y = (midpoint.point.y + startpoint.point.y)/2
        midpoint.point.z = (midpoint.point.z + 0.1)
        pitch = pi/4
        roll = pi/2    
        yaw=pi/2        
        grip=True
        frame= midpoint.header.frame_id        
        
        if not GripUtils.go_to(x=midpoint.point.x,y=midpoint.point.y,z=midpoint.point.z,roll=roll,pitch=pitch,yaw=yaw,grip=grip,frame=frame,arm='r',dur=7.5):
            print "failure to go to ",midpoint.point.x,midpoint.point.y,midpoint.point.z
            #return False
        endpoint = self.dupl_PointStamped(endPt)#self.convert_to_world_frame(endPt)
        frame=endpoint.header.frame_id
        if not GripUtils.go_to(x=endpoint.point.x,y=endpoint.point.y,z=endpoint.point.z,roll=roll,pitch=pitch,yaw=yaw,grip=grip,frame=frame,arm='r',dur=7.5):
            print "failure"
            #return False

        if(color_next == 'blue'):
            initRobot()            
        return True

    def executeFold_left(self,gripPt,endPt,color_current='blue',color_next='blue'):
        """ 
        execute fold with left arm
        """       
        #tmp = self.convert_to_world_frame(endPt)        
        #print "EndPts",tmp.point.x,tmp.point.y,tmp.point.z

        # if blue fold, move arm to gripping position. (if red fold, arm is already holding it at gripping position)
        if (color_current == 'blue'):
            startpoint = self.dupl_PointStamped(gripPt) #self.convert_to_world_frame(gripPt)            
            print "grabbing (x,y,z)",startpoint.point.x,startpoint.point.y,startpoint.point.z
            if not GripUtils.grab(x = startpoint.point.x,y=startpoint.point.y,z=startpoint.point.z,arm='l',
                                  roll=-pi/2,yaw=-pi/3,pitch=pi/4,approach= True,frame=startpoint.header.frame_id):               
                return False
        midpoint = self.dupl_PointStamped(endPt) #self.convert_to_world_frame(endPt)
        midpoint.point.x = (midpoint.point.x + startpoint.point.x)/2
        midpoint.point.y = (midpoint.point.y + startpoint.point.y)/2
        midpoint.point.z = (midpoint.point.z + 0.1)
        pitch = pi/4
        roll = -pi/2    
        yaw=-pi/2        
        grip=True
        frame= midpoint.header.frame_id        
        #print " going to ",midpoint.point.x,midpoint.point.y,midpoint.point.z

        if not GripUtils.go_to(x=midpoint.point.x,y=midpoint.point.y,z=midpoint.point.z,roll=roll,pitch=pitch,yaw=yaw,grip=grip,frame=frame,arm='l',dur=7.5):
            print "failure to go to ",midpoint.point.x,midpoint.point.y,midpoint.point.z
            #return False
        endpoint = self.dupl_PointStamped(endPt) #self.convert_to_world_frame(endPt)
        frame=endpoint.header.frame_id
        if not GripUtils.go_to(x=endpoint.point.x,y=endpoint.point.y,z=endpoint.point.z,roll=roll,pitch=pitch,yaw=yaw,grip=grip,frame=frame,arm='l',dur=7.5):
            print "failure"
            #return False

        if(color_next == 'blue'):
            initRobot()            
        return True

    def executeFold(self,gripPts,endPts,color_current='blue',color_next='blue'):
        """
        execute a fold 
        """

        #--- TODO--- convert gripPts,endPts to current frame of robot
        gripPts_new = []
        for pt in gripPts:
            if pt == None:
                continue
            pt_world = self.convert_to_world_frame(pt)            
            now = rospy.Time.now()
            self.listener.waitForTransform("base_footprint",pt_world.header.frame_id,now,rospy.Duration(10.0))            
            pt_transformed = self.listener.transformPoint("base_footprint",pt_world)            
            gripPts_new.append(pt_transformed)

        endPts_new = []
        for pt in endPts:
            if pt == None:
                continue
            pt_world = self.convert_to_world_frame(pt)            
            now = rospy.Time.now()
            self.listener.waitForTransform("base_footprint",pt_world.header.frame_id,now,rospy.Duration(10.0))            
            pt_transformed = self.listener.transformPoint("base_footprint",pt_world)            
            endPts_new.append(pt_transformed)

        gripPts = gripPts_new
        endPts = endPts_new

        if(gripPts[1] == None):
            self.executeFold_left(gripPt=gripPts[0],endPt=endPts[0],color_current=color_current,color_next=color_next)
            return

        elif(gripPts[0] == None):
            self.executeFold_right(gripPt=gripPts[1],endPt=endPts[1],color_current=color_current,color_next=color_next)
            return

        # if blue fold, move arms to gripping position. (if red fold, arms already holding cloth at gripping position)

        if (color_current == 'blue'):
            if (len(gripPts) > 2) or (len(endPts) >  2):            
                rospy.loginfo("Requires too many grippers")
                return False
            startpoints = []
            for pt in gripPts:
                #pt_world = self.convert_to_world_frame(pt)
                pt_world = self.dupl_PointStamped(pt)
                startpoints.append(pt_world)
            
        # determine approach yaws        
        #yaw_l = -pi/3 if gripPts[0].point.x < endPts[0].point.x else -4*pi/3 # left gripper
        #yaw_r = pi/3 if gripPts[1].point.x > endPts[1].point.x else  4*pi/3  # right gripper                
          
            
        # move both arms
            if not GripUtils.grab_points(point_l=startpoints[0],roll_l=-pi/2,yaw_l=-pi/3,pitch_l=pi/4,x_offset_l=-0.01, z_offset_l=0.005,approach= True,
                                         point_r=startpoints[1],roll_r=pi/2,yaw_r=pi/3,pitch_r=pi/4,x_offset_r=-0.01, z_offset_r=0.005):
                print "failure"
                return False
                         
        midpoints1 = []        
        midpoints = []
        for pt in gripPts:
            pt_world = pt# self.convert_to_world_frame(pt)            
            midpoints1.append(pt_world)
        i = 0
        for pt in endPts:
            pt_world = self.dupl_PointStamped(pt) #self.convert_to_world_frame(pt)
            pt_world.point.x = (pt_world.point.x + midpoints1[i].point.x)/2.0
            pt_world.point.y = (pt_world.point.y + midpoints1[i].point.y)/2.0
            pt_world.point.z = pt_world.point.z + 0.1
            midpoints.append(pt_world)        
            i +=1
        pitch_l = pitch_r = pi/4
        roll_l = -pi/2
        roll_r = pi/2
        yaw_l=-pi/2
        yaw_r= pi/2
        grip_l=grip_r=True
        frame_l=frame_r = midpoints[0].header.frame_id
        if not GripUtils.go_to_multi (x_l=midpoints[0].point.x,y_l=midpoints[0].point.y,z_l=midpoints[0].point.z,roll_l=roll_l,pitch_l=pitch_l,yaw_l=yaw_l,grip_l=grip_l,frame_l=frame_l,
                                        x_r=midpoints[1].point.x,y_r=midpoints[1].point.y,z_r=midpoints[1].point.z,roll_r=roll_r,pitch_r=pitch_r,yaw_r=yaw_r,grip_r=grip_r,frame_r=frame_r,dur=7.5):
            print "failure"
            #return False        
        endpoints = []
        for pt in endPts:
             pt_world = self.dupl_PointStamped(pt) #self.convert_to_world_frame(pt)
             endpoints.append(pt_world)
        frame_l=frame_r = endpoints[0].header.frame_id
        if not GripUtils.go_to_multi (x_l=endpoints[0].point.x,y_l=endpoints[0].point.y,z_l=endpoints[0].point.z,roll_l=roll_l,pitch_l=pitch_l,yaw_l=yaw_l,grip_l=grip_l,frame_l=frame_l
                                        ,x_r=endpoints[1].point.x,y_r=endpoints[1].point.y,z_r=endpoints[1].point.z,roll_r=roll_r,pitch_r=pitch_r,yaw_r=yaw_r,grip_r=grip_r,frame_r=frame_r
                                        ,dur=7.5):
            print "failure"
            #return False

        if(color_next == 'blue'):
            initRobot()
        return True

    def towelDragDemo(self,bl,tl,tr,br):
        """
        drag towel by 30 cm and fold
        """        
           
        # drag 30 cm
        drag_d = 0.2
        gripPts = [bl,br]
        endPts = []
        for pt in gripPts:                        
            pt_world = self.convert_to_world_frame(pt)
            print pt_world.point.y
            pt_world.point.x = pt_world.point.x - drag_d        
            endPts.append(self.convert_from_world_frame(pt_world))

        print "gripPts",gripPts,"endPts",endPts        
        if not self.executeDrag(gripPts,endPts,'red'):
            print "Dragging failed"
            return
        
        # fold in half
        gripPts = list(endPts)
        endPts = []
        for pt in [tl,tr]:
            pt_world = self.convert_to_world_frame(pt)
            pt_world.point.x =  pt_world.point.x - drag_d
            endPts.append(self.convert_from_world_frame(pt_world))
        if not self.executeFold(gripPts,endPts,color_current='red',color_next='blue'):
            print "Fold in half failed"
            return

        # fold left to right    
        gripPts = []
        pt_world_top = self.convert_to_world_frame(tl)
        pt_world_bottom = self.convert_to_world_frame(bl)
        pt_world_top.point.x = pt_world_top.point.x - 0.25*(pt_world_top.point.x - pt_world_bottom.point.x) - drag_d
        pt_world_top.point.y = (pt_world_top.point.y + pt_world_bottom.point.y)/2
        #print pt_world_top.point.y
        gripPts = [self.convert_from_world_frame(pt_world_top),None]
        
        pt_world_ctr_right = self.convert_to_world_frame(tr)
        pt_world_tmp = self.convert_to_world_frame(br)
        pt_world_ctr_right.point.x = pt_world_ctr_right.point.x - 0.25*(abs(pt_world_ctr_right.point.x - pt_world_tmp.point.x))  - drag_d
        pt_world_ctr_right.point.y = (pt_world_ctr_right.point.y + pt_world_tmp.point.y)/2
        
        #print "top",pt_world_top.point.y,"ctr_right",pt_world_ctr_right.point.y
        pt_world_end = pt_world_ctr_right
        pt_world_end.point.y = pt_world_top.point.y - 2.0/3.0*(abs(pt_world_top.point.y-pt_world_ctr_right.point.y))        
        #print pt_world_top.point.y,pt_world_end.point.y
        endPts = [self.convert_from_world_frame(pt_world_end),None]          
        if not self.executeFold(gripPts,endPts,color_current='blue',color_next='blue'):
            print "Fold left to right failed"            

        # fold right to left
        gripPts = []
        pt_world_top = self.convert_to_world_frame(tr)
        pt_world_bottom = self.convert_to_world_frame(br)
        pt_world_top.point.x = pt_world_top.point.x - 0.25*(pt_world_top.point.x - pt_world_bottom.point.x) - drag_d
        pt_world_top.point.y = (pt_world_top.point.y + pt_world_bottom.point.y)/2
        #print pt_world_top.point.y
        gripPts = [None,self.convert_from_world_frame(pt_world_top)]
        
        pt_world_ctr_left = self.convert_to_world_frame(tl)
        pt_world_tmp = self.convert_to_world_frame(bl)
        pt_world_ctr_left.point.x = pt_world_ctr_left.point.x + 0.25*(abs(pt_world_ctr_left.point.x - pt_world_tmp.point.x))  - drag_d
        pt_world_ctr_left.point.y = (pt_world_ctr_left.point.y + pt_world_tmp.point.y)/2        
        pt_world_end = pt_world_ctr_right
        pt_world_end.point.y = pt_world_top.point.y + 2.0/3.0*(abs(pt_world_top.point.y-pt_world_ctr_left.point.y))            
        endPts = [None,self.convert_from_world_frame(pt_world_end)]          
        if not self.executeFold(gripPts,endPts,color_current='blue',color_next='blue'):
            print "Fold left to right failed"
            return

def initRobot():
    # Look Down
    if not StanceUtils.call_stance('look_down',5.0):
        print "Look Down: Failure"
        return False
      
    # ARMS UP
    height = 0.35
    lateral_amount = 0.65
    forward_amount = 0.3
    if not GripUtils.go_to_multi(   x_l=forward_amount, y_l=lateral_amount, z_l=height,
                                    roll_l=0, pitch_l=0, yaw_l=0, grip_l=False,
                                    x_r=forward_amount, y_r=-lateral_amount, z_r=height,
                                    roll_r=0, pitch_r=0, yaw_r=0, grip_r=False,
                                    frame_l="torso_lift_link", frame_r="torso_lift_link", dur=4.0):
        return False
    else:
        return True
            
def main(args):
    rospy.init_node("poly_gui_bridge")
    rospy.sleep(1)
    print ("initializing",initRobot())
    pg = PolyGUIBridge()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
