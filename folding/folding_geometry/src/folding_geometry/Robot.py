#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_geometry")

"""
Robot.py
Contains the robot model and interface to functions of the PR2 used by folding

Authors: Karthik Lakshmanan
         Apoorva Sachdev
         Ziang Xie
"""
import rospy
import sys
import util
from geometry_msgs.msg import PointStamped,PoseStamped,Quaternion,Point
from arm_navigation_msgs.msg import Shape as GeometricShape
from folding_msgs.msg import PolyStamped,Point2D,FoldTraj
from pr2_simple_arm_motions import GripUtils
from pr2_simple_base_motions import base_move
from pr2_simple_motions_srvs.srv import *
from station_nav_server import StationNavServer
from inverse_reach import reach_viz
import StanceUtils
from folding_geometry.FoldingGUI import *
from shape_window import Geometry2D
from pr2_costs import gpp_costs
from numpy import *
import tf
import math
import time
from visualization_msgs.msg import Marker
from rll_utils.TFUtils import rpy_to_quaternion
from rll_utils.RvizUtils import draw_axes
from folding_geometry.msg import gPoint

DEBUG = False

class Robot():    
    def __init__(self):
        self.num_grippers = 2        
        self.drag_directions = ["b"] # can be "b","f","l","r"        
        self.init_robot_pose()
        self.basemover = base_move.BaseMover()
        self.IKcalculator = reach_viz.InverseReachViz()                
        self.listener = util.listener        
        print ("LISTENER",self.listener)
        #self.nav_server = StationNavServer()        
        self.robotposition = "table_front"
        self.costcalculator = gpp_costs.GPPCosts()
        #self.execute_move("table_front")
	self.marker_pub = rospy.Publisher('visualization_marker', Marker)
        rospy.loginfo("Robot is up")

    def arms_test(self,gripPt,arm):                     
        #pt = util.convert_to_world_frame(pt3d)
        #pt 
        if not GripUtils.grab(x = 0.3255,y=-0.14767,z=0.8128 ,arm='r',
                              roll=pi/2,yaw=pi/2,pitch=pi/4,approach= True,frame="base_footprint"):
            print "I FAIL.com"
            return False
        return

        gripPt = gripPt.ps        
        yaw = pi/2 if arm == 'r' else -pi/2
        if not GripUtils.grab(x = gripPt.point.x,y=gripPt.point.y,z=gripPt.point.z+0.003 ,arm=arm,
                              roll=pi/2,yaw=yaw,pitch=pi/4,approach= True,frame=gripPt.header.frame_id):
            return False
            
        
    def convert_to_robot_frame(self,pt_world,robotposition):
        """
        Takes a 3D point and converts it to the current frame of the robot's base
        """
        robotposition = "stations/"+robotposition+"_scoot" if self.robotposition!=robotposition else "base_footprint"
        now = rospy.Time.now()                
        #print "converting to",robotposition,"from",pt_world.header.frame_id
        #print "waiting for transform"
        self.listener.waitForTransform(robotposition,pt_world.ps.header.frame_id,now,rospy.Duration(10.0))        
        #print "transforming"
        pt_transformed = self.listener.transformPoint(robotposition,pt_world.ps)
        pt_transformed.header.frame_id = util.poly_frame # relabel point as base_footprint
        #print (pt_world.point.x,pt_world.point.y),"converts to",(pt_transformed.point.x,pt_transformed.point.y)
        newPt= gPoint()
        newPt.ps = pt_transformed
        newPt.hangedge = pt_world.hangedge
        return newPt

    def can_reach(self,(x,y,z),arm,roll,pitch,yaw):
        """
        checks to see if gripper of "arm" can reach "point3D" at "roll,pitch,yaw"
        """
        # IK uses wrist, so find pose of wrist if gripper is at point3D with (roll,pitch,yaw)
        gripper_l = 0.16
        wrist_x = x - gripper_l*math.cos(yaw)
        wrist_y = y - gripper_l*math.sin(yaw)
        wrist_z = z + gripper_l*math.cos(pitch)
        #print ("wrist: x,y,z=",wrist_x,wrist_y,wrist_z)
        #TODO Use markers to check this/replace w/ frame transform
        return (self.IKcalculator.ik_feasible((wrist_x,wrist_y,wrist_z),arm=arm))

    def sort_gripPts(self,gripPts):
        """
        Given gripPts, assign them to arms
        """
        if len(gripPts) == 1:
            # if gripPt is on left side of robot, use left arm. likewise for right.
            gripPts = [gripPts[0],None] if (gripPts[0].ps.point.y > 0) else [None,gripPts[0]]
            
        elif len(gripPts) == 2:
            # if a pt is hanging, assign it to the nearest gripper
            if(gripPts[0].hangedge != None):
                gripPts = [gripPts[0],gripPts[1]] if (gripPts[0].ps.point.y > 0) else [gripPts[1],gripPts[0]]
            elif(gripPts[1].hangedge != None):
                gripPts = [gripPts[1],gripPts[0]] if (gripPts[1].ps.point.y > 0) else [gripPts[0],gripPts[1]]
            else:
                # assign leftmost gripPt to left gripper etc
                gripPts = sorted(gripPts,key=lambda Pt: -1*Pt.ps.point.y)
        return gripPts           

    def feasible_fold(self,gripPts,endPts,robotposition,color="blue"):
        #Try to execute a fold from a given pose.                                                                                                                                                                                     
        #If possible kinematically, return (True,cost)                                                                                                                                                                                
        #If not, return False                                                                                                                                                                                                         
        if DEBUG:
            print "checking feasible fold. robot position = ",robotposition
        if len(gripPts) > self.num_grippers:
            return (False,float("infinity"))
        
        (l_arm_points,r_arm_points) = self.compute_xyzrpy_fold(gripPts,endPts,robotposition,color)

        if len(l_arm_points) == 0:
            for e in r_arm_points:
                l_arm_points.append(None)
        if len(r_arm_points) == 0:
            for e in l_arm_points:
                r_arm_points.append(None)

        #print l_arm_points,r_arm_points
        #raw_input()
        l_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, l_arm_points)
        r_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, r_arm_points)
                                                                                                                                                                                                     
        for k in xrange(len(l_arm_poses)):
            if l_arm_poses[k] != None:
                ps = PoseStamped()
                ps.header.frame_id = util.poly_frame
                ps.pose.position = l_arm_poses[k][0]
                ps.pose.orientation = l_arm_poses[k][1]
                l_arm_poses[k] = ps
            if r_arm_poses[k] != None:
                ps = PoseStamped()
                ps.header.frame_id = util.poly_frame
                ps.pose.position = r_arm_poses[k][0]
                ps.pose.orientation = r_arm_poses[k][1]
                r_arm_poses[k] = ps

        cost = self.costcalculator.move_arm_sequence_cost(l_arm_poses, r_arm_poses, 2)
        return (True,cost)
    
        
    def compute_xyzrpy_fold(self,gripPts,endPts,robotposition,color="blue"):                                                                                                                                                                                                                 

        l_arm_points = []
        r_arm_points = []

        # Transform points to current frame of robot                                                                                                                                                                
        gripPts = [util.convert_to_world_frame(gripPt) for gripPt in gripPts]
        gripPts = [self.convert_to_robot_frame(gripPt,robotposition) for gripPt in gripPts]
        #print "\ngripPts after convert_to_robot_frame"                                                                                                                                                             
        #for pt in gripPts:                                                                                                                                                                                         
        #    print (pt.point.x,pt.point.y,pt.point.z),                                                                                                                                                              
        endPts = [util.convert_to_world_frame(endPt) for endPt in endPts]
        endPts = [self.convert_to_robot_frame(endPt,robotposition) for endPt in endPts]

        # Assign points to grippers and sort endpoints accordingly                                                                                                                                                   
        gripPts_sorted = self.sort_gripPts(gripPts)
        endPts_sorted = [endPts[gripPts.index(gripPt_sorted)] if gripPt_sorted !=None else None for gripPt_sorted in gripPts_sorted]
        gripPts = gripPts_sorted
        endPts = endPts_sorted
        #print "gripPts_sorted"
        #for pt in gripPts:
        #    if pt!=None:
        #        print (pt.ps.point.x,pt.ps.point.y,pt.ps.point.z)

        # Midpoints                                                                                                                                                                                                 
        midpoints = []
        i = 0
        for pt in endPts:
            if pt == None:
                midpoints.append(None)
            else:
                midpoint = util.dupl_gPoint(pt)
                midpoint.ps.point.x = (midpoint.ps.point.x + gripPts[i].ps.point.x)/2.0
                midpoint.ps.point.y = (midpoint.ps.point.y + gripPts[i].ps.point.y)/2.0
                midpoint.ps.point.z = util.z_offset  + math.sqrt(math.pow(midpoint.ps.point.x - gripPts[i].ps.point.x,2)
                                                                 + math.pow(midpoint.ps.point.y - gripPts[i].ps.point.y,2))
                midpoints.append(midpoint)
            i+=1

        # calculate yaws                                                                                                                                                                                            
        fold_direction = []
        if(gripPts[0]!=None):
            fold_direction.append(math.atan2(midpoints[0].ps.point.y - gripPts[0].ps.point.y, midpoints[0].ps.point.x - gripPts[0].ps.point.x))
            yaw_l = self.calc_grip_yaw(direction = fold_direction[0],arm = 'l')
            #print "yaw_l",yaw_l
        else:
            fold_direction.append(None)
        if(gripPts[1]!=None):
            fold_direction.append(math.atan2(midpoints[1].ps.point.y - gripPts[1].ps.point.y, midpoints[1].ps.point.x - gripPts[1].ps.point.x))
            yaw_r =self.calc_grip_yaw(direction = fold_direction[1],arm = 'r')
            #print "yaw_r",yaw_r
        else:
            fold_direction.append(None)

        point_direction = [self.calc_hangdirection_robot(robotposition,gripPt.hangedge) if ((gripPt != None) and (gripPt.hangedge!=None)) else None for gripPt in gripPts]
        #if not (point_direction[0] == None and point_direction[1] == None):
        #    print point_direction
        #    raw_input("+++++++++++ Hanging fold ++++++++++")
        # reach for gripPts if fold is not a red fold                                                                                                                                                                                                                                                                        
        if (color == "blue"):
            # left arm                                                                                                                                                                                                                                                                                                      
            if(gripPts[0]!=None) and (point_direction[0] != None): # hanging grip point                                                                                                                                                                                                                                     
                yaw_l_hang = self.calc_grip_yaw_hanging(arm='l', robotposition = robotposition, hangedge = gripPts[0].hangedge)
                # calculate direction wrt robot                                                                                                                                                                                                                                                                             
                direction = point_direction[0]
                scoot_back = 0.5 if (direction == 'f') else 0    # if direction is 'f', scoot back to grab point 

                l_arm_points.append( ((gripPts[0].ps.point.x + scoot_back,gripPts[0].ps.point.y,gripPts[0].ps.point.z),(0,0,yaw_l_hang)))
                
                if DEBUG:
                    if not(self.can_reach((gripPts[0].ps.point.x + scoot_back,gripPts[0].ps.point.y,gripPts[0].ps.point.z),arm='l',roll=0,pitch=0,yaw=yaw_l_hang)):                                                                                                                                           
                        print "left arm cannot reach hanging grippt",(gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                        return (False,float("infinity"))               

            elif (gripPts[0]!= None):
                l_arm_points.append ( ((gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),(pi/2,pi/4,yaw_l)))
                if DEBUG:
                    if not (self.can_reach((gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                        print "left arm cannot reach grippt",(gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                        return (False,float("infinity"))                        
            else:
                l_arm_points.append(None)

            # right arm                                                                                                                                                                                                                                                                                                     
            if(gripPts[1]!=None) and (point_direction[1] != None):  # hanging grip point                                                                                                                                                                                                                                     
                yaw_r_hang = self.calc_grip_yaw_hanging(arm='r', robotposition = robotposition, hangedge = gripPts[1].hangedge)
                direction = point_direction[1]
                scoot_back = 0.5 if (direction == 'f') else 0 # if direction is 'f', scoot back to grab point                                                          
                r_arm_points.append( ((gripPts[1].ps.point.x + 0.5,gripPts[1].ps.point.y,gripPts[1].ps.point.z),(0,0,yaw_r_hang)))
                if DEBUG:
                    if not(self.can_reach((gripPts[1].ps.point.x + 0.5,gripPts[1].ps.point.y,gripPts[1].ps.point.z),arm='l',roll=0,pitch=0,yaw=yaw_r_hang)):
                        print "right arm cannot each grippt",(gripPts[1].ps.point.x + 0.5,gripPts[1].ps.point.y,gripPts[1].ps.point.z)
                        return (False,float("infinity"))

            elif (gripPts[1]!=None):
                r_arm_points.append( ((gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),(pi/2,pi/4,yaw_r)))
                if DEBUG:
                    if not (self.can_reach((gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                        print "right arm cannot reach grippt",(gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003)
                        return (False,float("infinity"))
            else:
                r_arm_points.append(None)

        x_adjusts = []
        y_adjusts = []
        z_adjusts = []
        for i in range(2):
            if (point_direction[i] in [None,'f']):
                x_adjust = y_adjust  = z_adjust = 0
            elif (point_direction[i] == 'l'):
                (x_adjust,y_adjust,z_adjust) = (0, abs(util.z_offset - gripPts[i].ps.point.z),0)
            elif (point_direction[i] == 'r'):
                (x_adjust,y_adjust,z_adjust) = (0, -abs(util.z_offset - gripPts[i].ps.point.z),0)
            x_adjusts.append(x_adjust)
            y_adjusts.append(y_adjust)
            z_adjusts.append(z_adjust)

        if (point_direction[0] not in [None,'f']):
            if (gripPts[0]!= None):
                l_arm_points.append( ((gripPts[0].ps.point.x + x_adjusts[0] ,gripPts[0].ps.point.y + y_adjusts[0] ,gripPts[0].ps.point.z + z_adjusts[0] + 0.003),(pi/2,pi/4,yaw_l)))
                if DEBUG:
                    if not (self.can_reach((gripPts[0].ps.point.x + x_adjusts[0] ,gripPts[0].ps.point.y + y_adjusts[0] ,gripPts[0].ps.point.z + z_adjusts[0] + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                        print "left arm cannot reach grippt",(gripPts[0].ps.point.x + x_adjusts[0],gripPts[0].ps.point.y + y_adjusts[0],gripPts[0].ps.point.z + z_adjusts[0] + 0.003)
                        return (False,float("infinity"))
            else:
                l_arm_points.append(None)
        else:
            l_arm_points.append(None)

        if (point_direction[1] not in [None,'f']):
            if (gripPts[1]!=None):
                r_arm_points.append( ((gripPts[1].ps.point.x + x_adjusts[1],gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003),(pi/2,pi/4,yaw_r)))
                if DEBUG:
                    if not (self.can_reach((gripPts[1].ps.point.x + x_adjusts[1],gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                        print "right arm cannot reach grippt",(gripPts[1].ps.point.x + x_adjusts[1],gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003)
                        return (False,float("infinity"))
            else:
                r_arm_points.append(None)
        else:
            r_arm_points.append(None)

        if (midpoints[0]!= None):
            l_arm_points.append( ((midpoints[0].ps.point.x + x_adjusts[0],midpoints[0].ps.point.y + y_adjusts[0] ,midpoints[0].ps.point.z + z_adjusts[0]),(pi/2,pi/2,yaw_l)))
            if DEBUG:
                if not self.can_reach((midpoints[0].ps.point.x + x_adjusts[0],midpoints[0].ps.point.y + y_adjusts[0] ,midpoints[0].ps.point.z + z_adjusts[0]),arm='l',roll=pi/2,pitch=pi/2, yaw=yaw_l):
                    print "left arm cannot reach midpoint",(midpoints[0].ps.point.x + x_adjusts[0],midpoints[0].ps.point.y+y_adjusts[0],midpoints[0].ps.point.z)
                    return (False,float("infinity"))
        else:
            l_arm_points.append(None)

        if (midpoints[1]!=None):
            r_arm_points.append( ((midpoints[1].ps.point.x + x_adjusts[1], midpoints[1].ps.point.y + y_adjusts[1],midpoints[1].ps.point.z + z_adjusts[1]),(pi/2,pi/2,yaw_r)))
            if DEBUG:
                if not (self.can_reach((midpoints[1].ps.point.x + x_adjusts[1], midpoints[1].ps.point.y + y_adjusts[1],midpoints[1].ps.point.z + z_adjusts[1]),arm='r',roll=pi/2,pitch=pi/2, yaw=yaw_r)):
                    print "right arm cannot reach midpoint",(midpoints[1].ps.point.x + x_adjusts[1],midpoints[1].ps.point.y + y_adjusts[1],midpoints[1].ps.point.z)
                    return (False,float("infinity"))
        else:
            r_arm_points.append(None)
         # Endpoints                                                                                                                                                                                                                                                                                                          
        if (endPts[0]!= None):
            l_arm_points.append( ((endPts[0].ps.point.x + x_adjusts[0],endPts[0].ps.point.y + y_adjusts[0],endPts[0].ps.point.z + z_adjusts[0]),(pi/2,pi/2,yaw_l)))
            if DEBUG:
                if not (self.can_reach((endPts[0].ps.point.x + x_adjusts[0],endPts[0].ps.point.y + y_adjusts[0],endPts[0].ps.point.z + z_adjusts[0]),arm='l',roll=pi/2,pitch=pi/2,yaw=yaw_l)):                                                                                                               
                    print "left arm cannot reach endpoint",(endPts[0].ps.point.x + x_adjusts[0],endPts[0].ps.point.y + y_adjusts[0], util.z_offset)                                                                     
                    return (False,float("infinity"))                                                                                                                                                                    
        else:
            l_arm_points.append(None)

        if (endPts[1]!=None):
            r_arm_points.append( ((endPts[1].ps.point.x + x_adjusts[1],endPts[1].ps.point.y + y_adjusts[1],endPts[1].ps.point.z+z_adjusts[1]), (pi/2,pi/2,yaw_r)))
            if DEBUG:
                if not (self.can_reach((endPts[1].ps.point.x + x_adjusts[1],endPts[1].ps.point.y + y_adjusts[1],endPts[1].ps.point.z+z_adjusts[1]),arm='r',roll=pi/2,pitch=pi/2,yaw=yaw_r)):                                                                                                                  
                    print "right arm cannot reach endpoint",(endPts[1].ps.point.x + x_adjusts[1],endPts[1].ps.point.y + y_adjusts[1], util.z_offset)                                                                                                                                                                        
                    return (False,float("infinity"))                                                                                                                                                                                                                                                                        
        else:
            r_arm_points.append(None) 
        
        return (l_arm_points,r_arm_points)
                       
    """
    def feasible_fold(self,gripPts,endPts,robotposition,color="blue"):
               
        Try to execute a fold from a given pose.
        If possible kinematically, return (True,cost)
        If not, return False
                        
        print "checking feasible fold. robot position = ",robotposition
        #print "gripPts being sent"
        #for gripPt in gripPts:
        #    print gripPt
        #print "endPts being sent"
        #for endPt in endPts:
        #    print endPt        
        if len(gripPts) > self.num_grippers:
            return (False,float("infinity"))        

        # Transform points to current frame of robot        
        gripPts = [util.convert_to_world_frame(gripPt) for gripPt in gripPts]        
        gripPts = [self.convert_to_robot_frame(gripPt,robotposition) for gripPt in gripPts]
        #print "\ngripPts after convert_to_robot_frame"
        #for pt in gripPts:
        #    print (pt.point.x,pt.point.y,pt.point.z),
        endPts = [util.convert_to_world_frame(endPt) for endPt in endPts]
        endPts = [self.convert_to_robot_frame(endPt,robotposition) for endPt in endPts]

        # Assign points to grippers and sort endpoints accordingly
        gripPts_sorted = self.sort_gripPts(gripPts)
        endPts_sorted = [endPts[gripPts.index(gripPt_sorted)] if gripPt_sorted !=None else None for gripPt_sorted in gripPts_sorted]
        gripPts = gripPts_sorted
        endPts = endPts_sorted              
        print "gripPts_sorted"
        for pt in gripPts:
            if pt!=None:
                print (pt.ps.point.x,pt.ps.point.y,pt.ps.point.z)
        # Midpoints
        midpoints = []
        i = 0
        for pt in endPts:
            if pt == None:
                midpoints.append(None)                
            else:    
                midpoint = util.dupl_gPoint(pt)
                midpoint.ps.point.x = (midpoint.ps.point.x + gripPts[i].ps.point.x)/2.0
                midpoint.ps.point.y = (midpoint.ps.point.y + gripPts[i].ps.point.y)/2.0
                midpoint.ps.point.z = util.z_offset  + math.sqrt(math.pow(midpoint.ps.point.x - gripPts[i].ps.point.x,2) 
                                                                 + math.pow(midpoint.ps.point.y - gripPts[i].ps.point.y,2))
                midpoints.append(midpoint)
            i+=1
        #print "\ngripPts"
        #for pt in gripPts:
        #    if pt!=None:
        #        print (pt.point.x,pt.point.y,pt.point.z),
        #print "\nmidPts"
        #for pt in midpoints:
        #    if pt!=None:
        #        print (pt.point.x,pt.point.y,pt.point.z),
        #print "\nendPts"
        #for pt in endPts:
        #    if pt!=None:
        #        print (pt.point.x,pt.point.y,pt.point.z),
        
        # calculate yaws
        fold_direction = []
        if(gripPts[0]!=None):
            fold_direction.append(math.atan2(midpoints[0].ps.point.y - gripPts[0].ps.point.y, midpoints[0].ps.point.x - gripPts[0].ps.point.x))        
            yaw_l = self.calc_grip_yaw(direction = fold_direction[0],arm = 'l')            
            print "yaw_l",yaw_l
        else:
            fold_direction.append(None)
        if(gripPts[1]!=None):
            fold_direction.append(math.atan2(midpoints[1].ps.point.y - gripPts[1].ps.point.y, midpoints[1].ps.point.x - gripPts[1].ps.point.x))
            yaw_r =self.calc_grip_yaw(direction = fold_direction[1],arm = 'r')            
            print "yaw_r",yaw_r
        else:
            fold_direction.append(None)        
        
        point_direction = [self.calc_hangdirection_robot(robotposition,gripPt.hangedge) if ((gripPt != None) and (gripPt.hangedge!=None)) else None for gripPt in gripPts]        
        #if not (point_direction[0] == None and point_direction[1] == None):
            #print point_direction
            #raw_input("+++++++++++ Hanging fold ++++++++++")

        # reach for gripPts if fold is not a red fold                                                  
        if (color == "blue"):            
            # left arm
            if(gripPts[0]!=None) and (point_direction[0] != None): # hanging grip point
                yaw_l_hang = self.calc_grip_yaw_hanging(arm='l', robotposition = robotposition, hangedge = gripPts[0].hangedge)               
                # calculate direction wrt robot
                direction = point_direction[0]
                scoot_back = 0.5 if (direction == 'f') else 0    # if direction is 'f', scoot back to grab point
                if not(self.can_reach((gripPts[0].ps.point.x + scoot_back,gripPts[0].ps.point.y,gripPts[0].ps.point.z),arm='l',roll=0,pitch=0,yaw=yaw_l_hang)):
                    #print "util.z_offset",util.z_offset
                    #raw_input()
                    print "left arm cannot reach hanging grippt",(gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                    return (False,float("infinity"))

            elif (gripPts[0]!= None) and not (self.can_reach((gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                print "left arm cannot reach grippt",(gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                return (False,float("infinity"))
            
            # right arm
            if(gripPts[1]!=None) and (point_direction[1] != None):  # hanging grip point
                yaw_r_hang = self.calc_grip_yaw_hanging(arm='r', robotposition = robotposition, hangedge = gripPts[1].hangedge)
                direction = point_direction[1]
                scoot_back = 0.5 if (direction == 'f') else 0 # if direction is 'f', scoot back to grab point
                if not(self.can_reach((gripPts[1].ps.point.x + 0.5,gripPts[1].ps.point.y,gripPts[1].ps.point.z),arm='l',roll=0,pitch=0,yaw=yaw_r_hang)):
                    print "right arm cannot each grippt",(gripPts[1].ps.point.x + 0.5,gripPts[1].ps.point.y,gripPts[1].ps.point.z)
                    return (False,float("infinity"))

            elif (gripPts[1]!=None) and not (self.can_reach((gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                print "right arm cannot reach grippt",(gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003)
                return (False,float("infinity"))        

        x_adjusts = []
        y_adjusts = []
        z_adjusts = []
        for i in range(2):
            if (point_direction[i] in [None,'f']):
                x_adjust = y_adjust  = z_adjust = 0
            elif (point_direction[i] == 'l'):
                (x_adjust,y_adjust,z_adjust) = (0, abs(util.z_offset - gripPts[i].ps.point.z),0)
            elif (point_direction[i] == 'r'):
                (x_adjust,y_adjust,z_adjust) = (0, -abs(util.z_offset - gripPts[i].ps.point.z),0)
            x_adjusts.append(x_adjust)
            y_adjusts.append(y_adjust)
            z_adjusts.append(z_adjust)

        if (point_direction[0] not in [None,'f']):
            if (gripPts[0]!= None) and not (self.can_reach((gripPts[0].ps.point.x + x_adjusts[0] ,gripPts[0].ps.point.y + y_adjusts[0] ,gripPts[0].ps.point.z + z_adjusts[0] + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                print "left arm cannot reach grippt",(gripPts[0].ps.point.x + x_adjusts[0],gripPts[0].ps.point.y + y_adjusts[0],gripPts[0].ps.point.z + z_adjusts[0] + 0.003)
                return (False,float("infinity"))
            
        if (point_direction[1] not in [None,'f']):
            if (gripPts[1]!=None) and not (self.can_reach((gripPts[1].ps.point.x + x_adjusts[1],gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                print "right arm cannot reach grippt",(gripPts[1].ps.point.x + x_adjusts[1],gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003)
                return (False,float("infinity"))

        if (midpoints[0]!= None) and not self.can_reach((midpoints[0].ps.point.x + x_adjusts[0],midpoints[0].ps.point.y + y_adjusts[0] ,midpoints[0].ps.point.z + z_adjusts[0]),arm='l',roll=pi/2,pitch=pi/2, yaw=yaw_l):
            print "left arm cannot reach midpoint",(midpoints[0].ps.point.x + x_adjusts[0],midpoints[0].ps.point.y+y_adjusts[0],midpoints[0].ps.point.z)
            return (False,float("infinity"))

        if (midpoints[1]!=None) and not (self.can_reach((midpoints[1].ps.point.x + x_adjusts[1], midpoints[1].ps.point.y + y_adjusts[1],midpoints[1].ps.point.z + z_adjusts[1]),arm='r',roll=pi/2,pitch=pi/2, yaw=yaw_r)):
            print "right arm cannot reach midpoint",(midpoints[1].ps.point.x + x_adjusts[1],midpoints[1].ps.point.y + y_adjusts[1],midpoints[1].ps.point.z)
            return (False,float("infinity"))
                
        # Endpoints
        if (endPts[0]!= None) and not (self.can_reach((endPts[0].ps.point.x + x_adjusts[0],endPts[0].ps.point.y + y_adjusts[0],endPts[0].ps.point.z + z_adjusts[0]),arm='l',roll=pi/2,pitch=pi/2,yaw=yaw_l)):
            print "left arm cannot reach endpoint",(endPts[0].ps.point.x + x_adjusts[0],endPts[0].ps.point.y + y_adjusts[0], util.z_offset)
            return (False,float("infinity"))

        if (endPts[1]!=None) and not (self.can_reach((endPts[1].ps.point.x + x_adjusts[1],endPts[1].ps.point.y + y_adjusts[1],endPts[1].ps.point.z+z_adjusts[1]),arm='r',roll=pi/2,pitch=pi/2,yaw=yaw_r)):
            print "right arm cannot reach endpoint",(endPts[1].ps.point.x + x_adjusts[1],endPts[1].ps.point.y + y_adjusts[1], util.z_offset) 
            return (False,float("infinity"))

            
        return (True,1)
    """
    
    def calc_hangdirection_robot(self,robotposition,hangedge):
         # hackily simplify robot positions                                                                                                                                                                                                                                                                                 
        if robotposition in ["table_front_left","table_left"]:
            robotposition = "table_left"
        elif robotposition in ["table_front_right","table_right"]:
            robotposition = "table_right"

        # find plane of hanging wrt robot    
        direction = None

        #print "hangEdge", hangedge, " direction" , direction
        if robotposition == hangedge:
            direction = "f"

        elif robotposition == "table_front":
            if hangedge == "table_right":
                direction = "r"
            elif hangedge == "table_left":
                direction = "l"

        elif robotposition == "table_right":
            if hangedge == "table_front":
                direction = "l"

        elif robotposition == "table_left":
            if hangedge == "table_front":
                direction = "r"

        if direction !=None:
            print "robotposition",robotposition,"hangedge",hangedge,"direction",direction
        return direction

    def calc_grip_yaw_hanging(self,arm,robotposition,hangedge):
        """
        Helper function. Takes hangedge (table_front,table_left,table_right) and robotposition, and determines yaw
        """
        direction = self.calc_hangdirection_robot(robotposition,hangedge)
        if direction == "f":
            yaw = pi/2 if arm == "r" else -pi/2
        else:
            yaw = 0
        return yaw

    def calc_grip_yaw(self,direction,arm):
        """
        Helper function that takes a direction (in radians) and calculates yaw
        """
        direction = direction%(2*math.pi)
        if arm == 'l':
            if math.pi/4 <= direction <= 3*math.pi/4:
                return math.pi/2
            else:
                return -math.pi/2
        elif arm == 'r':            
            if 5*math.pi/4 <= direction <= 7*math.pi/4:
                return -math.pi/2
            else:
                return math.pi/2

    def feasible_drag(self,gripPts,d,direction,robotposition):
        """                                                                                                                                                                                                                                                                                                                 
        Try to execute a drag from a given pose.                                                                                                                                                                                                                                                                            
        If possible kinematically, return (True,cost)                                                                                                                                                                                                                                                                       
        If not, return False                                                                                                                                                                                                                                                                                                
        """
        if DEBUG:
            print "checking feasible drag. robot position = ",robotposition, "direction = ",direction
        if len(gripPts) > self.num_grippers:
            return (False,float("infinity"))

        (l_arm_points,r_arm_points) = self.compute_xyzrpy_drag(gripPts,d,direction,robotposition)
        
        # compute costs
        distance = d/util.scale_factor if (direction !='f') else 0
        if len(l_arm_points) == 0:
            for e in r_arm_points:
                l_arm_points.append(None)
        if len(r_arm_points) == 0:
            for e in l_arm_points:
                r_arm_points.append(None)
        l_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, l_arm_points)
        r_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, r_arm_points)
        for k in xrange(len(l_arm_poses)):
            if l_arm_poses[k] != None:
                ps = PoseStamped()
                ps.header.frame_id = util.poly_frame
                ps.pose.position = l_arm_poses[k][0]
                ps.pose.orientation = l_arm_poses[k][1]
                l_arm_poses[k] = ps
            if r_arm_poses[k] != None:
                ps = PoseStamped()
                ps.header.frame_id = util.poly_frame
                ps.pose.position = r_arm_poses[k][0]
                ps.pose.orientation = r_arm_poses[k][1]
                r_arm_poses[k] = ps
        cost = self.costcalculator.move_arm_sequence_cost(l_arm_poses, r_arm_poses, 2)
        cost += self.costcalculator.pbc.scoot_cost(distance)

        return (True,cost)


    def compute_xyzrpy_drag(self,gripPts,d,direction,robotposition):
        """                                                                                                                          
        Compute (xyz) and (rpy) for waypoints during a drag
        """        
        l_arm_points = []
        r_arm_points = []

        direction = drag_direction(direction,robotposition)
        # Transform points to current frame of robot                                                                                       
        gripPts = [util.convert_to_world_frame(gripPt) for gripPt in gripPts]
        gripPts = [self.convert_to_robot_frame(gripPt,robotposition) for gripPt in gripPts]
        
        # Assign points to grippers        
        gripPts = self.sort_gripPts(gripPts)              
        # Call on Inverse Kinematics for critical points in the trajectory                                                                                               
        # gripPts                                             
        if direction == "f":
            angle = 0
        elif direction == "r":
            angle = -pi/2
        elif direction == "l":
            angle = pi/2
        elif direction == "b":
            angle = pi

        yaw_l = self.calc_grip_yaw(direction = angle,arm = 'l')
        yaw_r =self.calc_grip_yaw(direction = angle,arm = 'r')
        if (gripPts[0]!= None):
            l_arm_points.append( ((gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),(pi/2,pi/4,yaw_l)))
            if DEBUG:
                if not (self.can_reach((gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                    print "left arm cannot reach grippt",(gripPts[0].ps.point.x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                    return (False,float("infinity"))
        else:
            l_arm_points.append(None)

        if (gripPts[1]!=None):
            r_arm_points.append( ((gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),(pi/2,pi/4,yaw_r)))
            if DEBUG:
                if not (self.can_reach((gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                    print "right arm cannot reach grippt",(gripPts[1].ps.point.x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003)
                    return (False,float("infinity"))
        else:
            r_arm_points.append(None)

        # if dragging away from robot, check IK of endPts also    
        if (direction == "f"):
            endPts = []
            for pt in gripPts:
                if gripPts == None:
                    endPts.append(None)
                else:
                    endPt = util.dupl_gPoint(pt)
                    endPt.ps.point.x += d
                    endPts.append(endPt)            
            if (endPts[0]!= None):
                l_arm_points.append( ((endPts[0].ps.point.x,endPts[0].ps.point.y,endPts[0].ps.point.z + 0.003),(pi/2,pi/4,yaw_l)))
                if DEBUG:
                    if not (self.can_reach((endPts[0].ps.point.x,endPts[0].ps.point.y,endPts[0].ps.point.z + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):                
                        print "left arm cannot reach endpt",(endPts[0].ps.point.x,endPts[0].ps.point.y,endPts[0].ps.point.z + 0.003)
                        return (False,float("infinity"))
            else:
                l_arm_points.append(None)

            if (endPts[1]!=None):
                r_arm_points.append( ((endPts[1].ps.point.x,endPts[1].ps.point.y,endPts[1].ps.point.z + 0.003),(pi/2,pi/4,yaw_r)))
                if DEBUG:
                    if not (self.can_reach((endPts[1].ps.point.x,endPts[1].ps.point.y,endPts[1].ps.point.z + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                        print "right arm cannot reach endpt",(endPts[1].ps.point.x,endPts[1].ps.point.y,endPts[1].ps.point.z + 0.003)
                        return (False,float("infinity"))
            else:
                r_arm_points.append(None)
        else:
            l_arm_points.append(None)
            r_arm_points.append(None)

        return (l_arm_points,r_arm_points)

    def execute_fold(self,gripPts,endPts,color_current='blue',color_next='blue'):
        """
        execute a fold
        """
        if DEBUG:
            print "checking feasible fold. robot position = ",robotposition
        if len(gripPts) > self.num_grippers:
            return (False,float("infinity"))

        (l_arm_points,r_arm_points) = self.compute_xyzrpy_fold(gripPts,endPts,robotposition,color)

        # if color_current is blue, grab first points.        
        if color_current=='blue':
            # two arm grab
            if None not in (l_arm_points[0],r_arm_points[0]):                
                (l_x,l_y,l_z) = l_arm_points[0][0]
                (l_roll,l_pitch,l_yaw) = l_arm_points[0][1]
                ps_l = PointStamped()
                ps_l.point.x = l_x
                ps_l.point.y = l_y
                ps_l.point.z = l_z
                ps_l.header.frame_id = util.poly_frame
                (r_x,r_y,r_z) = r_arm_points[0][0]
                (r_roll,r_pitch,r_yaw) = r_arm_points[0][1]
                ps_r = PointStamped()
                ps_r.point.x = r_x
                ps_r.point.y = r_y
                ps_r.point.z = r_z
                ps_r.header.frame_id = util.poly_frame
                if not GripUtils.grab_points(point_l=ps_l,roll_l=l_roll,yaw_l=l_yaw,pitch_l=l_pitch,x_offset_l=0,z_offset_l=0.003,approach= True,
                                             point_r=ps_r,roll_r=r_roll,yaw_r=r_yaw,pitch_r=r_pitch,x_offset_r=0,z_offset_r=0.003):
                    print "Both arms failed to grab startpoints"
                    return False

            elif (l_arm_points[0] != None):
                (x,y,z) = l_arm_points[0][0]
                (roll,pitch,yaw) = l_arm_points[0][1]
                if not GripUtils.grab(x = x,y=y,z=z ,arm='l',roll=roll,yaw=yaw,pitch=pitch,approach= True,frame=util.poly_frame):
                    print "Left arm failed to grab startpoint"
                    return False

            elif (r_arm_points[0] != None):
                (x,y,z) = r_arm_points[0][0]
                (roll,pitch,yaw) = r_arm_points[0][1]
                if not GripUtils.grab(x = x,y=y,z=z ,arm='r',
                                      roll=roll,yaw=yaw,pitch=pitch,approach= True,frame=util.poly_frame):
                    print "Right arm failed to grab startpoint"
                    return False


        for l_arm_point,r_arm_point in zip(l_arm_points[1:],r_arm_points[1:]):
            # two arm go to
            if None not in (l_arm_point,y_arm_point):
                (l_x,l_y,l_z) = l_arm_point[0]
                (l_roll,l_pitch,l_yaw) = l_arm_point[1]
                (r_x,r_y,r_z) = r_arm_point[0]
                (r_roll,r_pitch,r_yaw) = r_arm_point[1]
                if not GripUtils.go_to_multi (x_l= l_x,y_l= l_y,z_l= l_z,roll_l= l_roll,pitch_l=l_pitch,yaw_l=l_yaw,grip_l=True,frame_l= util.poly_frame,
                                      x_r=r_x,y_r=r_y,z_r=r_z,roll_r=r_roll,pitch_r=r_pitch,yaw_r=r_yaw,grip_r=True,frame_r = util.poly_frame,dur=7.5):
                    print "two arm failure"
                    return False
            # one arm go to
            elif l_arm_point != None:
                 (x,y,z) = l_arm_point[0]
                 (roll,pitch,yaw) = l_arm_point[1]
                 if not GripUtils.go_to(x=x,y=y,z=z,roll=roll,pitch=pitch,yaw=yaw,grip=True,frame=util.poly_frame,arm='l',dur=7.5):
                     print "left arm failure"
                     return False
            elif r_arm_point != None:
                 (x,y,z) = r_arm_point[0]
                 (roll,pitch,yaw) = r_arm_point[1]
                 if not GripUtils.go_to(x=x,y=y,z=z,roll=roll,pitch=pitch,yaw=yaw,grip=True,frame=util.poly_frame,arm='r',dur=7.5):
                     print "right arm failure"
                     return False
                  
        # If the next fold/action is 'blue', open grippers                                                                                                                                                                                                                                                                  
        if(color_next == 'blue'):
            self.init_robot_pose()
        return True

    def execute_drag(self,gripPts,d=0.1,direction='+y',color_next='blue'):
        """
        Grabs gripPts and moves back through distance d
        """        
        direction = drag_direction(direction,self.robotposition)
        # Assign points to grippers                                                                                                                                             
        gripPts_sorted = self.sort_gripPts(gripPts)

        if direction =="f":
            angle = 0
        elif direction == "r":
            angle = -pi/2
        elif direction == "l":
            angle = pi/2
        elif direction == "b":
            angle = pi

        yaw_l = self.calc_grip_yaw(direction = angle,arm = 'l')
        yaw_r =self.calc_grip_yaw(direction = angle,arm = 'r')
        
        # Start points                        
        if not GripUtils.grab_points(point_l=gripPts[0],roll_l=pi/2,yaw_l=yaw_l,pitch_l=pi/4,x_offset_l=0,z_offset_l=0.003,approach= True,point_r=gripPts[1],roll_r=pi/2,yaw_r=yaw_r,pitch_r=pi/4,x_offset_r=0,z_offset_r=0.003):
            print "Failure to grab startpoints"
            return False
        
        # Move through distance d
        pt = Point2D()
        if direction == 'b':
            # Move back through distance d                    
            pt.x = -d
            pt.y = 0
            self.basemover.move_base(pt)
        elif direction == 'r':
            # Move right through distance d
            pt.x = 0
            pt.y = -d
            self.basemover.move_base(pt)
        elif direction == 'l':
            # Move left through distance d
            pt.x = 0
            pt.y = d
            self.basemover.move_base(pt)                    
        elif (direction == "f"):
            endPts = []
            for pt in gripPts:
                if gripPts == None:
                    endPts.append(None)
                else:
                    endPt = util.dupl_PointStamped(pt)
                    endPt.point.x += d
                    endPts.append(endPt)
            if None not in endPts:
                if not GripUtils.go_to_multi (x_l=endPts[0].point.x,y_l=endPts[0].point.y,z_l=endPts[0].point.z,roll_l=pi/2,pitch_l=pi/4,yaw_l=yaw_l,grip_l=True,frame_l=frame_l,x_r=endPts[1].point.x,y_r=endPts[1].point.y,z_r=endPts[1].point.z,roll_r=pi/2,pitch_r=pi/4,yaw_r=yaw_r,grip_r=True,frame_r=frame_r,dur=7.5):
                    print "Failure to go to endpoints"
                    return False
        # if next fold is blue, open grippers
        if(color_next == 'blue'):
            self.init_robot_pose()    

        if direction in ['b','r','l']:
            # return to original pose                
            pt.x = -pt.x
            pt.y = -pt.y
            self.basemover.move_base(pt)
        
        return True

    def move_cost(self, start_station, end_station):
	return self.costcalculator.station_nav_cost(start_station, end_station)

    def execute_move(self,dest):
        self.nav_server.go_to_station(dest)
        self.robotposition = dest

    def init_robot_pose(self):
        """
        makes PR2 look down at table and put arms up
        """
        # Look Down
        if not StanceUtils.call_stance('look_down',5.0):
            print "Look Down: Failure"
            return False      
        # Raise Arms
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

    def print_costs(self):
        print "overhead", self.costcalculator.get_overhead(), "ik time",self.costcalculator.get_ik_time()        
        

def drag_direction(direction,robotposition):
    """
    hacky conversion between what FoldingSearch returns and a drag direction relative to robot
    """
    if robotposition in ["table_front"]:
        if direction == "-y":
            return "f"
        elif direction == "+y":
            return "b"        
        elif direction == "+x":
            return "r"
        elif direction == "-x":            
            return "l"
        
    elif robotposition in ["table_left","table_front_left"]:
        if direction == "+y":
            return "r"
        elif direction == "-y":
            return "l"
        elif direction == "-x":
            return "b"
        elif direction == "+x":
            return "f"

    elif robotposition in ["table_right","table_front_right"]:
        if direction == "+y":
            return "l"
        elif direction == "-y":
            return "r"
        elif direction == "+x":
            return "b"
        elif direction == "-x":
            return "f"

    return None
