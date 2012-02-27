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
from pr2_simple_base_motions import base_move_gpp
from pr2_simple_motions_srvs.srv import *
from pr2_simple_torso_motions import torso_mover
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
from time import strftime, gmtime
from visualization_msgs.msg import Marker
from rll_utils.TFUtils import rpy_to_quaternion
from rll_utils.RvizUtils import draw_axes
from folding_geometry.msg import gPoint
from gpp_navigation import set_sim_state
import os
from folding_main import RECORD_FLAG, SIM_FLAG
import json
from util import mode
import commands

DEBUG = False

LOG_FILE = strftime('/tmp/fold_actions_%Y-%m-%d-%H-%M-%S.log', gmtime())
LOG_FILE = LOG_FILE.replace('fold', mode)
flog = open(LOG_FILE, 'w')

base_move = base_move_gpp

def log_action(name, base_movements, joint_angles):
    action_dict = {}
    action_dict['name'] = name
    action_dict['base_movements'] = base_movements
    action_dict['joint_angles'] = joint_angles
    s = json.dumps(action_dict)
    flog.write(s+'\n')

class Robot():    
    def __init__(self):
        self.num_grippers = 2        
        self.drag_directions = ["b"] # can be "b","f","l","r"        
        self.init_robot_pose()
        commands.getoutput('rosrun pr2_mechanism_controllers send_periodic_cmd_srv.py laser_tilt_controller linear 0 0 0.0')
        if not False:#SIM_FLAG:
            self.basemover = base_move.BaseMover()
            self.torsomover = torso_mover.TorsoMover()
            self.IKcalculator = reach_viz.InverseReachViz()                
            self.nav_server = StationNavServer()        

        self.listener = util.listener        

        if os.environ['ROBOT_MODE'] == 'sim':
            set_sim_state.set_station('/stations/table_front_scoot', self.listener)

        #print ("LISTENER",self.listener)
        self.robotposition = "table_front"
        self.costcalculator = gpp_costs.GPPCosts()
        #self.execute_move("table_front")
        self.marker_pub = rospy.Publisher('visualization_marker', Marker)
        self.marker_id = 0
        if not False:#SIM_FLAG:
            rospy.loginfo('-----------Moving torso up-------------')
            self.torsomover.move_torso(0.29)
        rospy.loginfo("Robot is up")

    def arms_test(self,pt_l,pt_r):           
        pt_l = self.convert_to_robot_frame(util.convert_to_world_frame(pt_l),self.robotposition)
        pt_r = self.convert_to_robot_frame(util.convert_to_world_frame(pt_r),self.robotposition)
        print "pt_l",(pt_l.ps.point.x,pt_l.ps.point.y,pt_l.ps.point.z)
        print "pt_r",(pt_r.ps.point.x,pt_r.ps.point.y,pt_r.ps.point.z)
        
        self.basemover.move_base(-0.4,0)
        if not GripUtils.grab(x = pt_l.ps.point.x+0.4,y=pt_l.ps.point.y,z=pt_l.ps.point.z ,arm='r',roll=pi/2,yaw=0,pitch=pi/4,approach= True,frame=util.poly_frame):
                    print "Left arm failed to grab startpoint"
                    raw_input()
                    
        midpoint = util.dupl_gPoint(pt_l)
        midpoint.ps.point.x = (midpoint.ps.point.x + pt_r.ps.point.x)/2.0
        midpoint.ps.point.y = (midpoint.ps.point.y + pt_r.ps.point.y)/2.0
        midpoint.ps.point.z = util.z_offset  + math.sqrt(math.pow(midpoint.ps.point.x - pt_r.ps.point.x,2)
                                                         + math.pow(midpoint.ps.point.y - pt_r.ps.point.y,2))

        
        self.basemover.move_base(0.4,0)
        
        if not GripUtils.go_to(x=midpoint.ps.point.x,y=midpoint.ps.point.y,z=midpoint.ps.point.z,roll=pi,pitch=pi/2,yaw=pi/2,grip=True,frame=util.poly_frame,arm='r',dur=7.5):
                     print "left arm failure"
                     raw_input()
        
        if not GripUtils.go_to(x=pt_r.ps.point.x,y=pt_r.ps.point.y,z=pt_r.ps.point.z,roll=pi,pitch=pi/2,yaw=pi/2,grip=True,frame=util.poly_frame,arm='r',dur=7.5):
                     print "left arm failure"
                     raw_input()
        GripUtils.open_grippers()             
        self.init_robot_pose()
        """
        pt_l = util.dupl_gPoint(midpoint)
        pt_l.ps.point.z = util.z_offset

    def arms_test(self,gripPts,endPts):                             
        #gripPts = [self.convert_to_robot_frame(util.convert_to_world_frame(gripPt),self.robotposition) for gripPt in gripPts]
        #endPts = [self.convert_to_robot_frame(util.convert_to_world_frame(endPt),self.robotposition) for endPt in endPts]        
        self.execute_fold(gripPts,endPts,color_current='blue',color_next='blue')
        return
        if not GripUtils.grab_points(point_l=gripPts[0].ps,roll_l=pi/2,yaw_l=-pi/2,pitch_l=pi/4,x_offset_l=0,z_offset_l=0.003,approach= True,point_r=gripPts[1].ps,roll_r=pi/2,yaw_r=pi/2,pitch_r=pi/4,x_offset_r=0,z_offset_r=0.003):
                print "Failure to grab startpoints"
                raw_input()
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

        if not GripUtils.grab_points(point_l=pt_l.ps,roll_l=pi/2,yaw_l=-pi/2,pitch_l=pi/4,x_offset_l=0,z_offset_l=0.003,approach= True,point_r=pt_r.ps,roll_r=pi/2,yaw_r=pi/2,pitch_r=pi/4,x_offset_r=0,z_offset_r=0.003):
            print "Failure to grab startpoints"
            raw_input()        
                     
        if not GripUtils.go_to_multi (x_l=pt_l.ps.point.x,y_l=pt_l.ps.point.y,z_l=pt_l.ps.point.z + 0.3,roll_l=pi/2,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,frame_l=util.poly_frame,x_r=pt_r.ps.point.x,y_r=pt_r.ps.point.y,z_r=pt_r.ps.point.z + 0.3,roll_r=pi/2,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,frame_r=util.poly_frame,dur=7.5):
            print "Failure to go to endpoints"

        self.basemover.move_base(-0.3,0)
        raw_input("hit a key to shake")
        print "width = ", abs(pt_l.ps.point.y - pt_r.ps.point.y)
        if not self.shake_arms(3,pt_l,pt_r):
            print "shaking failed"
            """

    def test_Gshake(self,pt_l,pt_r):
        pt_l = self.convert_to_robot_frame(util.convert_to_world_frame(pt_l),self.robotposition)
        pt_r = self.convert_to_robot_frame(util.convert_to_world_frame(pt_r),self.robotposition)

        if not GripUtils.grab(x = pt_l.ps.point.x,y=pt_l.ps.point.y,z=pt_l.ps.point.z ,arm='l',roll=pi/2,yaw=-pi/2,pitch=pi/4,approach= True,frame=util.poly_frame):
                    print "Left arm failed to grab startpoint"
                    raw_input()

        midpoint = util.dupl_gPoint(pt_l)
        midpoint.ps.point.x = (midpoint.ps.point.x + pt_r.ps.point.x)/2.0
        midpoint.ps.point.y = (midpoint.ps.point.y + pt_r.ps.point.y)/2.0
        midpoint.ps.point.z = util.z_offset  + math.sqrt(math.pow(midpoint.ps.point.x - pt_r.ps.point.x,2)
                                                         + math.pow(midpoint.ps.point.y - pt_r.ps.point.y,2))

        if not GripUtils.go_to(x=midpoint.ps.point.x,y=midpoint.ps.point.y,z=midpoint.ps.point.z,roll=pi/2,pitch=pi/2,yaw=-pi/2,grip=True,frame=util.poly_frame,arm='l',dur=7.5):
                     print "left arm failure"
                     raw_input()

        if not GripUtils.go_to(x=pt_r.ps.point.x,y=pt_r.ps.point.y,z=pt_r.ps.point.z,roll=pi/2,pitch=pi/2,yaw=-pi/2,grip=True,frame=util.poly_frame,arm='l',dur=7.5):
                     print "left arm failure"
                     raw_input()
        GripUtils.open_grippers()
        self.init_robot_pose()
        pt_l = util.dupl_gPoint(midpoint)
        pt_l.ps.point.z = util.z_offset

        if not GripUtils.grab_points(point_l=pt_l.ps,roll_l=pi/2,yaw_l=-pi/2,pitch_l=pi/4,x_offset_l=0,z_offset_l=0.003,approach= True,point_r=pt_r.ps,roll_r=pi/2,yaw_r=pi/2,pitch_r=pi/4,x_offset_r=0,z_offset_r=0.003):
            print "Failure to grab startpoints"
            raw_input()

        if not GripUtils.go_to_multi (x_l=pt_l.ps.point.x,y_l=pt_l.ps.point.y,z_l=pt_l.ps.point.z + 0.3,roll_l=pi/2,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,frame_l=util.poly_frame,x_r=pt_r.ps.point.x,y_r=pt_r.ps.point.y,z_r=pt_r.ps.point.z + 0.3,roll_r=pi/2,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,frame_r=util.poly_frame,dur=7.5):
            print "Failure to go to endpoints"

        self.basemover.move_base(-0.3,0)
        raw_input("hit a key to shake")
        print "width = ", abs(pt_l.ps.point.y - pt_r.ps.point.y)
        if not self.shake_arms(3,pt_l,pt_r):
            print "shaking failed"


    def shake_arms(self,num,pt_l,pt_r):
        """
        shakes arms
        """
        pt_l_top = util.dupl_gPoint(pt_l)
        pt_r_top = util.dupl_gPoint(pt_r)
        pt_l_top.ps.point.z += 0.1
        pt_r_top.ps.point.z += 0.1

        pt_l_bottom = util.dupl_gPoint(pt_l)
        pt_r_bottom = util.dupl_gPoint(pt_r)
        pt_l_bottom.ps.point.z -= 0.1
        pt_r_bottom.ps.point.z -= 0.1

        for i in range(num):            
            if not GripUtils.go_to_multi (x_l=pt_l_top.ps.point.x,y_l=pt_l_top.ps.point.y,z_l=pt_l_top.ps.point.z + 0.3,roll_l=pi/2,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,frame_l=util.poly_frame,x_r=pt_r_top.ps.point.x,y_r=pt_r_top.ps.point.y,z_r=pt_r_top.ps.point.z + 0.3,roll_r=pi/2,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,frame_r=util.poly_frame,dur=0.3):
                print "Failure to shake"
            if not GripUtils.go_to_multi (x_l=pt_l_bottom.ps.point.x,y_l=pt_l_bottom.ps.point.y,z_l=pt_l_bottom.ps.point.z + 0.3,roll_l=pi/2,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,frame_l=util.poly_frame,x_r=pt_r_bottom.ps.point.x,y_r=pt_r_bottom.ps.point.y,z_r=pt_r_bottom.ps.point.z + 0.3,roll_r=pi/2,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,frame_r=util.poly_frame,dur=0.3):
                print "Failure to shake"
                            
    def convert_to_robot_frame(self,pt_world,robotposition):
        """
        Takes a 3D point and converts it to the current frame of the robot's base
        """
        robotposition = "stations/"+robotposition+"_scoot" if self.robotposition!=robotposition else "base_footprint"
        now = rospy.Time.now()                
        #print "converting to",robotposition,"from",pt_world.header.frame_id
        #print "waiting for transform"
        #self.listener.waitForTransform(robotposition,pt_world.ps.header.frame_id,now,rospy.Duration(10.0))        
        #print "transforming"
        pt_world.ps.header.stamp = rospy.Time(0)
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
        global DEBUG    
                                                                                                                                           
        if DEBUG:
            print "\n\n\nchecking feasible fold. robot position = ",robotposition, "foldcolor = ",color, "num GripPoints", gripPts
        if len(gripPts) > self.num_grippers:
            return (False,float("infinity"))
        
        (l_arm_points,r_arm_points,scoots) = self.compute_xyzrpy_fold(gripPts,endPts,robotposition,color)
        if (l_arm_points == None) and (r_arm_points == None):
            return (False,float("infinity"))

        if len(l_arm_points) == 0:
            for e in r_arm_points:
                l_arm_points.append(None)
        if len(r_arm_points) == 0:
            for e in l_arm_points:
                r_arm_points.append(None)

        if DEBUG:
            print "l_arm_points",l_arm_points
            print "r_arm_points",r_arm_points
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
        if DEBUG:
            print "cost of fold", cost

        return (True,cost)
    
        
    def calc_scoot_amt(self,pt_l,pt_r):
        """
        takes pt_l,pt_r and returns the new coords (x_l,x_r,scoot_amt)
        """
        REACH_AMT = 0.45
        if (pt_l != None) and (pt_r == None):
            # left arm fold
            point_x = pt_l.ps.point.x
            point_x = max(REACH_AMT,point_x)
            scoot_amt = pt_l.ps.point.x - point_x
            #rint "scoot_amt",scoot_amt
            return (point_x,0,scoot_amt)

        elif (pt_r!=None) and (pt_l == None):
            # right arm fold
            point_x = pt_r.ps.point.x
            point_x = max(REACH_AMT,point_x)
            scoot_amt = pt_r.ps.point.x - point_x
            #rint "scoot_amt",scoot_amt
            return (0,point_x,scoot_amt)
        
        elif (pt_r!=None) and (pt_l != None):
            # two arm fold            
            if pt_l.ps.point.x < pt_r.ps.point.x:
                nearest_arm = 'l'
                point_x = max(pt_l.ps.point.x,REACH_AMT)
                x_l = point_x
                x_r = x_l + (pt_r.ps.point.x - pt_l.ps.point.x)
                scoot_amt = pt_l.ps.point.x - x_l
            else:
                nearest_arm = 'r'
                point_x = max(pt_r.ps.point.x,REACH_AMT)
                x_r = point_x
                x_l = x_r + (pt_l.ps.point.x - pt_r.ps.point.x)
                scoot_amt = pt_l.ps.point.x - x_l
            return (x_l,x_r,scoot_amt)
        else:
            print "No arms"
            return (0,0,0)
            
            
    def calc_scoot_diff(self,scoot_total,scoot):
        """
        returns scoot_now, scoot_total
        """
        # already scooted by scoot_total. need to arrive at orig minus scoot
        if (scoot) <= 0:
            scoot_now = scoot - scoot_total                                            
            return (scoot_now,scoot)
        else:
            print "dont scoot"
            return (0,scoot_total)

    def compute_xyzrpy_fold(self,gripPts,endPts,robotposition,color="blue",scoot_prev=0):                                                                                                                                                                 #print "in compute xyzrpy fold, scoot_prev = ", scoot_prev
        
        l_arm_points = []
        r_arm_points = []
        scoots = []
        yaw_l = yaw_r = -1
        """
        print "GRIPPOINTS from GUI"
        for gripPt in gripPts:
            print gripPt

        print "ENDPOINTS from GUI"
        for endPt in endPts:
            print endPt
            """
        # Transform points to current frame of robot                                                                                                                                                                
        gripPts = [util.convert_to_world_frame(gripPt) for gripPt in gripPts]
        gripPts = [self.convert_to_robot_frame(gripPt,robotposition) for gripPt in gripPts]
        #print "\ngripPts after convert_to_robot_frame"                                                                                                                                                            
        #for pt in gripPts:                                                                                                                                                                                         
        #    print (pt.point.x,pt.point.y,pt.point.z),                                                                                                                                                             
        endPts = [util.convert_to_world_frame(endPt) for endPt in endPts]
        endPts = [self.convert_to_robot_frame(endPt,robotposition) for endPt in endPts]
        
        """
        print "GRIPPOINTS unsorted"
        for gripPt in gripPts:
            if gripPt != None:
                print (gripPt.ps.point.x,gripPt.ps.point.y,gripPt.ps.point.z)

        print "ENDPOINTS unsorted"
        for endPt in endPts:
            if endPt != None:                                                                                                                                                                                                                                                                                               
                 print (endPt.ps.point.x,endPt.ps.point.y,endPt.ps.point.z)
        """

        for gripPt in gripPts:
            gripPt.ps.point.x += scoot_prev
        for endPt in endPts:
            endPt.ps.point.x += scoot_prev
            
        # fail fast. if any point is more than 1m away from robot, return false

        # Assign points to grippers and sort endpoints accordingly                                                                                                                                                   
        gripPts_sorted = self.sort_gripPts(gripPts)
        endPts_sorted = [endPts[gripPts.index(gripPt_sorted)] if gripPt_sorted !=None else None for gripPt_sorted in gripPts_sorted]
        gripPts = gripPts_sorted
        endPts = endPts_sorted
        """
        print "GRIPPOINTS"
        for gripPt in gripPts:                                                                                                                                                                                                                                                                                              
            if gripPt != None:                                                                                                                                                                                                                                                                                              
                print (gripPt.ps.point.x,gripPt.ps.point.y,gripPt.ps.point.z)
                          
        print "ENDPOINTS"
        for endPt in endPts:                                                                                                                                                                                                                                                                                                
            if endPt != None:                                                                                                                                                                                                                                                                                                                print (endPt.ps.point.x,endPt.ps.point.y,endPt.ps.point.z)
        """
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
                                                                 + math.pow(midpoint.ps.point.y - gripPts[i].ps.point.y,2)) - 0.02
                midpoint.ps.point.z = max(midpoint.ps.point.z,util.z_offset + 0.02)
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
        #print point_direction
        #if not (point_direction[0] == None and point_direction[1] == None):
        #    print point_direction
        #    raw_input("+++++++++++ Hanging fold ++++++++++")
        # reach for gripPts if fold is not a red fold                                                                                                                                            

        scoot_back = 0        
        scoot_total = 0 # can never become positive
        if (color == "blue"):
            (x_l,x_r,scoot) = self.calc_scoot_amt(gripPts[0],gripPts[1])                                                                                                                           
            (scoot_now,scoot_total) = self.calc_scoot_diff(scoot_total,scoot)
            scoots.append(scoot_now)
            
            # left arm
            if(gripPts[0]!=None) and (point_direction[0] != None): # hanging grip point                                                                                                                                                                                                                                     
                yaw_l_hang = self.calc_grip_yaw_hanging(arm='l', robotposition = robotposition, hangedge = gripPts[0].hangedge, folddirection = fold_direction[0])
                # calculate direction wrt robot                                                                                                                                                                                                                                                                             
                direction = point_direction[0]
                point_x = x_l 
                l_arm_points.append( ((point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z),(0,0,yaw_l_hang)))
                
                if DEBUG:
                    if not(self.can_reach((point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z),arm='l',roll=0,pitch=0,yaw=yaw_l_hang)):                                                                                                                                           
                        print "left arm cannot reach hanging grippt",(point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                        #return (False,float("infinity"))               

            elif (gripPts[0]!= None):
                point_x = x_l
                l_arm_points.append ( ((point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),(pi/2,pi/4,yaw_l)))
                if DEBUG:
                    if not (self.can_reach((point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                        print "left arm cannot reach grippt",(point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                        #return (False,float("infinity"))                        
            else:
                l_arm_points.append(None)

            # right arm                                                                                                                                                                                                                                                                                                     
            if(gripPts[1]!=None) and (point_direction[1] != None):  # hanging grip point                                                                                                                                                                                                            
                yaw_r_hang = self.calc_grip_yaw_hanging(arm='r', robotposition = robotposition, hangedge = gripPts[1].hangedge, folddirection = fold_direction[1])
                direction = point_direction[1]
                #scoot_back = 0.5 if (direction == 'f') else 0 # if direction is 'f', scoot back to grab point                            
                point_x = x_r                
                r_arm_points.append( ((point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z),(pi,0,yaw_r_hang)))
                if DEBUG:
                    if not(self.can_reach((point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z),arm='l',roll=0,pitch=0,yaw=yaw_r_hang)):
                        print "right arm cannot reach hanging grippt",(point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z)
                        #return (False,float("infinity"))

            elif (gripPts[1]!=None):
                point_x = x_r            
                r_arm_points.append( ((point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),(pi/2,pi/4,yaw_r)))
                if DEBUG:
                    if not (self.can_reach((point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                        print "right arm cannot reach grippt",(point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003)
                        #return (False,float("infinity"))
            else:
                r_arm_points.append(None)
        
        else:
            l_arm_points.append(None)
            r_arm_points.append(None)
            scoots.append(0)

        x_adjusts = []
        y_adjusts = []
        z_adjusts = []
        for i in range(2):
            if (point_direction[i] in [None]):
                x_adjust = y_adjust  = z_adjust = 0
            elif (point_direction[i] == 'f') and (color == "blue"):
                x_adjust = 0#-0.15
                y_adjust = 0
                z_adjust = 0#(util.z_offset - gripPts[i].ps.point.z) + 0.05
                print 'z_adjust', z_adjust            
            elif (point_direction[i] == 'l'):
                (x_adjust,y_adjust,z_adjust) = (0, abs(util.z_offset - gripPts[i].ps.point.z),0)
            elif (point_direction[i] == 'r'):
                (x_adjust,y_adjust,z_adjust) = (0, -abs(util.z_offset - gripPts[i].ps.point.z),0)
            else:
                x_adjust = y_adjust = z_adjust = 0
            x_adjusts.append(x_adjust)
            y_adjusts.append(y_adjust)
            z_adjusts.append(z_adjust)

        if (color == "blue"):
            (x_l,x_r,scoot) = self.calc_scoot_amt(gripPts[0],gripPts[1])
            (scoot_now,scoot_total) = self.calc_scoot_diff(scoot_total,scoot)
            scoots.append(scoot_now)            

            if (point_direction[0] not in [None,"f"]): # was also f
                if (gripPts[0]!= None):
                    point_x = x_l + x_adjusts[0]                    
                    l_arm_points.append( ((point_x,gripPts[0].ps.point.y + y_adjusts[0] ,gripPts[0].ps.point.z + z_adjusts[0] + 0.003),(pi/2,pi/4,yaw_l)))
                    if DEBUG:
                        if not (self.can_reach((point_x,gripPts[0].ps.point.y + y_adjusts[0] ,gripPts[0].ps.point.z + z_adjusts[0] + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                            print "left arm cannot reach second grippt",(point_x,gripPts[0].ps.point.y + y_adjusts[0],gripPts[0].ps.point.z + z_adjusts[0] + 0.003)
                        #return (False,float("infinity"))
                else:
                    l_arm_points.append(None)
            else:
                l_arm_points.append(None)

            if (point_direction[1] not in [None,"f"]):
                if (gripPts[1]!=None):
                    point_x = x_r + x_adjusts[1]                    
                    r_arm_points.append( ((point_x,gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003),(pi/2,pi/4,yaw_r)))
                    if DEBUG:
                        if not (self.can_reach((point_x,gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                            print "right arm cannot reach second grippt",(point_x,gripPts[1].ps.point.y + y_adjusts[1],gripPts[1].ps.point.z + z_adjusts[1] + 0.003)
                        #return (False,float("infinity"))
                else:
                    r_arm_points.append(None)
            else:
                r_arm_points.append(None)
        else:
            l_arm_points.append(None)
            r_arm_points.append(None)
            scoots.append(0)

        RELAX_AMT = 0.015
         
        (x_l,x_r,scoot) = self.calc_scoot_amt(midpoints[0],midpoints[1])
        (scoot_now,scoot_total) = self.calc_scoot_diff(scoot_total,scoot)
        scoots.append(scoot_now)

        if (yaw_l == 0) or (yaw_r == 0):
            roll = pi/2
        else:
            roll = pi/2

        if midpoints[0]!=None:
            roll_inc_l = self.calc_roll_increment(fold_direction[0],'l')
        if midpoints[1]!=None:
            roll_inc_r = self.calc_roll_increment(fold_direction[1],'r')
        #print "left_roll_inc",roll_inc_l
        #print "right_roll_inc",roll_inc_r
        x_adjusts[0] = y_adjusts[0] = z_adjusts[0] = 0    
        if (midpoints[0]!= None):            
            point_x = x_l + x_adjusts[0] #+ SCOOT_FRONT            
            l_arm_points.append( ((point_x,midpoints[0].ps.point.y +y_adjusts[0] - RELAX_AMT ,midpoints[0].ps.point.z + z_adjusts[0]),(roll+roll_inc_l,pi/4,yaw_l)))
            if DEBUG:
                if not self.can_reach((point_x ,midpoints[0].ps.point.y + y_adjusts[0] - RELAX_AMT,midpoints[0].ps.point.z + z_adjusts[0]),arm='l',roll=roll+roll_inc_l,pitch=pi/4, yaw=yaw_l):
                    print "left arm cannot reach midpoint",(point_x,midpoints[0].ps.point.y+y_adjusts[0],midpoints[0].ps.point.z)
                    #return (False,float("infinity"))
        else:
            l_arm_points.append(None)

        if (midpoints[1]!=None):
            x_adjusts[1] = 0
            point_x = x_r + x_adjusts[1] #+ SCOOT_FRONT            
            y_adjusts[1] = 0
            r_arm_points.append( ((point_x, midpoints[1].ps.point.y + y_adjusts[1] + RELAX_AMT,midpoints[1].ps.point.z),(roll + roll_inc_r,pi/4,yaw_r)))
            if DEBUG:
                if not (self.can_reach((point_x, midpoints[1].ps.point.y + y_adjusts[1] + RELAX_AMT,midpoints[1].ps.point.z ),arm='r',roll=roll+roll_inc_r,pitch=pi/4, yaw=yaw_r)):
                    print "right arm cannot reach midpoint",(point_x,midpoints[1].ps.point.y + y_adjusts[1],midpoints[1].ps.point.z)
                    #return (False,float("infinity"))
        else:
            r_arm_points.append(None)

         # Endpoints                                                                                                                                                                                                                                                                                                        
        (x_l,x_r,scoot) = self.calc_scoot_amt(endPts[0],endPts[1])
        (scoot_now,scoot_total) = self.calc_scoot_diff(scoot_total,scoot)
        scoots.append(scoot_now)

        if (endPts[0]!= None):
            point_x = x_l + x_adjusts[0] #+ SCOOT_FRONT2
            l_arm_points.append( ((point_x,endPts[0].ps.point.y + y_adjusts[0],endPts[0].ps.point.z + 0.03),(roll+2*roll_inc_l,pi/4,yaw_l)))
            if DEBUG:
                if not (self.can_reach((point_x,endPts[0].ps.point.y + y_adjusts[0],endPts[0].ps.point.z + 0.03),arm='l',roll=roll+ 2*roll_inc_l,pitch=pi/4,yaw=yaw_l)):                                                                                                               
                    print "left arm cannot reach endpoint",(point_x,endPts[0].ps.point.y + y_adjusts[0], util.z_offset)                                                                     
                    #return (False,float("infinity"))                                                                                                                                                                    
        else:
            l_arm_points.append(None)

        if (endPts[1]!=None):
            point_x = x_r + x_adjusts[1] #+ SCOOT_FRONT2            
            r_arm_points.append( ((point_x,endPts[1].ps.point.y + y_adjusts[1],endPts[1].ps.point.z + 0.03), (roll+2*roll_inc_r,pi/4,yaw_r)))
            if DEBUG:
                if not (self.can_reach((point_x,endPts[1].ps.point.y + y_adjusts[1],endPts[1].ps.point.z + 0.03),arm='r',roll=roll+2*roll_inc_r,pitch=pi/4,yaw=yaw_r)):                                                                                                                  
                    print "right arm cannot reach endpoint",(point_x,endPts[1].ps.point.y + y_adjusts[1], util.z_offset)                                                                                                                                                                        
              #return (False,float("infinity"))                                                                                                                                                                                                                                                                        
        else:
            r_arm_points.append(None) 
        """
        print "l_arm_points"
        for pt in l_arm_points:
            print pt
        print "r_arm_points"
        for pt in r_arm_points:
            print pt
            """        

        return (l_arm_points,r_arm_points,scoots)
    
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

        #if direction !=None:
        #    print "robotposition",robotposition,"hangedge",hangedge,"direction",direction

        #print "HANG DIRECTION IS",direction
        return direction

    def calc_grip_yaw_hanging(self,arm,robotposition,hangedge,folddirection=None):
        """
        Helper function. Takes hangedge (table_front,table_left,table_right) and robotposition, and determines yaw
        """
        direction = self.calc_hangdirection_robot(robotposition,hangedge)
        if direction == "f":
            yaw = pi/2 if arm == "r" else -pi/2
        else:
            folddirection = folddirection%(2*pi) if folddirection!=None else None
            if (folddirection == None):
                yaw = 0
            elif (folddirection >= 3*pi/4 and folddirection <= 5*pi/4):
                yaw = pi
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
            #elif (0 <= direction < math.pi/4) or (7*math.pi/4 <= direction <= 2*math.pi):
            #    return 0
            else:
                return -math.pi/2
        elif arm == 'r':            
            if 5*math.pi/4 <= direction <= 7*math.pi/4:
                return -math.pi/2
            #elif (0 <= direction < math.pi/4) or (7*math.pi/4 <= direction <= 2*math.pi):
            #    return 0                
            else:
                return math.pi/2

    def calc_roll_increment(self,direction,arm):
        """
        Calculates how much to roll gripper between trajectory points
        """
        direction = direction%(2*math.pi)
        if (7*math.pi/4 <= direction) or (direction <= math.pi/4):
            roll_inc = math.pi/2
        elif (3*math.pi/4 <= direction <= 5*math.pi/4):
            roll_inc = -math.pi/2
        else:
            roll_inc = 0

        if arm == 'l':
            roll_inc = -1 * roll_inc  # left arm rolls are signed the other way

        return roll_inc
        

    def feasible_drag(self,gripPts,d,direction,robotposition):
        """                                                                                                                                                                                                                                                                                                       Try to execute a drag from a given pose.                                                                                                                                                                                                                                                                           
        If possible kinematically, return (True,cost)                                                                                                                                                                                                                                                                       
        If not, return False                                                                                                                                                                                                                                                                                                
        """
        if DEBUG:
            print "checking feasible drag. robot position = ",robotposition, "direction = ",direction
        if len(gripPts) > self.num_grippers:
            return (False,float("infinity"))

        (l_arm_points,r_arm_points,scoots) = self.compute_xyzrpy_drag(gripPts,d,direction,robotposition)
        if (l_arm_points == None) and (r_arm_points == None):
            return (False,float("infinity"))
        
        
        # compute costs
        distance = d/util.scale_factor if (direction !='f') else 0
        #print "Drag distance is",distance
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
        cost += self.costcalculator.pbc.scoot_cost(2*distance)

        return (True,cost)


    def compute_xyzrpy_drag(self,gripPts,d,direction,robotposition):
        """                                                                                                                          
        Compute (xyz) and (rpy) for waypoints during a drag
        """        
        l_arm_points = []
        r_arm_points = []
        scoots = []
        direction = drag_direction(direction,robotposition)
        # Transform points to current frame of robot                                                                                       
        gripPts = [util.convert_to_world_frame(gripPt) for gripPt in gripPts]
        gripPts = [self.convert_to_robot_frame(gripPt,robotposition) for gripPt in gripPts]
                

        # fail fast. if any point is more than 1m away from robot, return false 
        """
        for gripPt in gripPts:
            if gripPt==None:
                continue
            if (abs(gripPt.ps.point.x) > 1) or (abs(gripPt.ps.point.y) > 1) or (abs(gripPt.ps.point.z) > 1):
                return (None, None)
        """
        # Assign points to grippers        
        gripPts = self.sort_gripPts(gripPts)              
        # Call on Inverse Kinematics for critical points in the trajectory                                                                                               

        # gripPts                                             
        if direction == "f":
            angle = pi
        elif direction == "r":
            angle = -pi/2
        elif direction == "l":
            angle = pi/2
        elif direction == "b":
            angle = 0

        yaw_l = -pi/2 #self.calc_grip_yaw(direction = angle,arm = 'l')
        yaw_r = pi/2  #self.calc_grip_yaw(direction = angle,arm = 'r')

        scoot_total = 0
        (x_l,x_r,scoot) = self.calc_scoot_amt(gripPts[0],gripPts[1])
        (scoot_now,scoot_total) = self.calc_scoot_diff(scoot_total,scoot)
        scoots.append(scoot_now)

        if (gripPts[0]!= None):
            point_x = x_l #gripPts[0].ps.point.x
            #point_x = 0.5 if point_x < 0.5 else point_x
            l_arm_points.append( ((point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),(pi/2,pi/4,yaw_l)))
            if DEBUG:
                if not (self.can_reach((point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):
                    print "left arm cannot reach grippt",(point_x,gripPts[0].ps.point.y,gripPts[0].ps.point.z + 0.003)
                    #return (False,float("infinity"))
        else:
            l_arm_points.append(None)

        if (gripPts[1]!=None):
            point_x = x_r #gripPts[1].ps.point.x
            #point_x = max(0.5,point_x)
            r_arm_points.append( ((point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),(pi/2,pi/4,yaw_r)))
            if DEBUG:
                if not (self.can_reach((point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                    print "right arm cannot reach grippt",(point_x,gripPts[1].ps.point.y,gripPts[1].ps.point.z + 0.003)
                    #return (False,float("infinity"))
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
                point_x = endPts[0].ps.point.x
                point_x = max(0.5,point_x)
                l_arm_points.append( ((point_x,endPts[0].ps.point.y,endPts[0].ps.point.z + 0.003),(pi/2,pi/4,yaw_l)))
                if DEBUG:
                    if not (self.can_reach((point_x,endPts[0].ps.point.y,endPts[0].ps.point.z + 0.003),arm='l',roll=pi/2,pitch=pi/4, yaw=yaw_l)):                
                        print "left arm cannot reach endpt",(point_x,endPts[0].ps.point.y,endPts[0].ps.point.z + 0.003)
                        #return (False,float("infinity"))
            else:
                l_arm_points.append(None)

            if (endPts[1]!=None):
                point_x = endPts[1].ps.point.x
                point_x= max(0.5,point_x)
                r_arm_points.append( ((point_x,endPts[1].ps.point.y,endPts[1].ps.point.z + 0.003),(pi/2,pi/4,yaw_r)))
                if DEBUG:
                    if not (self.can_reach((point_x,endPts[1].ps.point.y,endPts[1].ps.point.z + 0.003),arm='r',roll=pi/2,pitch=pi/4, yaw=yaw_r)):
                        print "right arm cannot reach endpt",(point_x,endPts[1].ps.point.y,endPts[1].ps.point.z + 0.003)
                        #return (False,float("infinity"))
            else:
                r_arm_points.append(None)
        else:
            l_arm_points.append(None)
            r_arm_points.append(None)

        return (l_arm_points,r_arm_points,scoots)

    def point_quat_to_pose(self, pt, quat):
        ps = PoseStamped()
        ps.header.frame_id = util.poly_frame
        ps.pose.position = pt
        ps.pose.orientation = quat
        return ps

    def goto_pre_hanging_pose(self, arm = 'both'):
        """
        scoot back, reach under table, scoot ahead
        """
        pt = Point2D()
        backup_d = 0.3
        print "Hanging point. Moving base by",backup_d
        pt.x = -backup_d
        pt.y = 0
        self.basemover.move_base(pt.x,pt.y)
        
        if arm == 'both':
            if not GripUtils.go_to_multi (x_l= 0.27,y_l= 0.37,z_l= 0.62,roll_l= 0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=False,frame_l= util.poly_frame,
                                          x_r=0.27,y_r=-0.37,z_r=0.62,roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=False,frame_r = util.poly_frame,dur=2.0):
                print "two arm failure"
                raw_input()
        elif arm == 'l':
            if not GripUtils.go_to(x= 0.27,y= 0.37,z= 0.62,roll= 0,pitch=pi/4,yaw=-pi/2,grip=False,frame= util.poly_frame,dur=2.0, arm='l'):
                print "left arm failure"
                raw_input()
        elif arm == 'r':
            if not GripUtils.go_to(x=0.27,y=-0.37,z=0.62,roll=0,pitch=pi/4,yaw=pi/2,grip=False,frame = util.poly_frame,dur=2.0, arm = 'r'):
                print "right arm failure"
                raw_input()

            
        pt.x = backup_d
        pt.y = 0
        self.basemover.move_base(pt.x,pt.y)

    def goto_post_hanging_pose(self):
        if not GripUtils.go_to_relative_multi(x_offset_l=-0.1,y_offset_l=0,z_offset_l=0,grip_l=True,x_offset_r=-0.1,y_offset_r=0,z_offset_r=0,grip_r=\
                                                  True,frame=util.poly_frame, dur = 2.0):
            print "Failure to move in after grabbing hanging points"

        if not GripUtils.go_to_relative_multi(x_offset_l=0,y_offset_l=-0.1,z_offset_l=0.2,grip_l=True,x_offset_r=0,y_offset_r=0.1,z_offset_r=0.2,grip_r=\
                                                      True,frame=util.poly_frame, dur = 2.0):
            print "Failure to move up after grabbing hanging points"


    def execute_fold(self,gripPts,endPts,color_current='blue',color_next='blue',scoot_prev = 0):
        """
        execute a fold
        """       
        print "\n\n\n --------------------------------------------------------------------------------------------------------"
        print "in execute fold", self.robotposition
        if len(gripPts) > self.num_grippers:
            return False

        RAISED_ARM = False

        (l_arm_points,r_arm_points,scoots) = self.compute_xyzrpy_fold(gripPts,endPts,self.robotposition,color_current,scoot_prev)
        print "scoots are", scoots
        print "l_arm_points",l_arm_points
        print "r_arm_points",r_arm_points
        """
	# Visualize/debu
        l_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, l_arm_points)
        r_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, r_arm_points)	
	for k in xrange(len(l_arm_poses)):
	    if l_arm_poses[k] != None:
	    	self.marker_id += 1
		ps = self.point_quat_to_pose(l_arm_poses[k][0], l_arm_poses[k][1])
	    	draw_axes(self.marker_pub, self.marker_id, 'grip_poses', ps, text='l')
	    if r_arm_poses[k] != None:
	    	self.marker_id += 1
		ps = self.point_quat_to_pose(r_arm_poses[k][0], r_arm_poses[k][1])
	    	draw_axes(self.marker_pub, self.marker_id, 'grip_poses', ps, text='r')
	"""
        # Visualize/debug
        l_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, l_arm_points)
        r_arm_poses = map(lambda xyzrpy: (Point(*xyzrpy[0]), rpy_to_quaternion(*xyzrpy[1])) if xyzrpy else None, r_arm_points)	

        if RECORD_FLAG:
            base_moves = [(scoot, 0, 0) for scoot in scoots]
            l_arm_pss = [PoseStamped() for p in l_arm_poses]
            r_arm_pss = [PoseStamped() for p in r_arm_poses]
            for k in xrange(len(l_arm_pss)):
                if l_arm_poses[k] == None:
                    l_arm_pss[k] = None
                else:
                    l_arm_pss[k].header.frame_id = 'base_footprint'
                    l_arm_pss[k].pose.position = l_arm_poses[k][0]
                    l_arm_pss[k].pose.orientation = l_arm_poses[k][1]
                if r_arm_poses[k] == None:
                    r_arm_pss[k] = None
                else:
                    r_arm_pss[k].header.frame_id = 'base_footprint'
                    r_arm_pss[k].pose.position = r_arm_poses[k][0]
                    r_arm_pss[k].pose.orientation = r_arm_poses[k][1]

            cost,joint_states_sequence = self.costcalculator.move_arm_sequence_cost(l_arm_pss, r_arm_pss, 2, return_angles=True)
            print '-------------DEBUGGING JOINT STATES SEQUENCE-----------------'
            print 'jss length: ', len(joint_states_sequence)
            for k in xrange(len(joint_states_sequence)):
                print 'length of js #', k, ' ', len(joint_states_sequence[k])
            joint_states_sequence = [[js[0].position, js[1].position] for js in joint_states_sequence]
            log_action('fold', base_moves, joint_states_sequence)
            return

        for k in xrange(len(l_arm_poses)):
            if l_arm_poses[k] != None:
                self.marker_id += 1
                ps = self.point_quat_to_pose(l_arm_poses[k][0], l_arm_poses[k][1])
                draw_axes(self.marker_pub, self.marker_id, 'grip_poses', ps, text='l')
            if r_arm_poses[k] != None:
                self.marker_id += 1
                ps = self.point_quat_to_pose(r_arm_poses[k][0], r_arm_poses[k][1])
                draw_axes(self.marker_pub, self.marker_id, 'grip_poses', ps, text='r')

        pt = Point2D()
        pt.y = 0
        if color_current == "red":
            scoot_init = scoots[2] - scoot_prev
        else:
            scoot_init = scoots[0] - scoot_prev

        if (scoot_init) != 0:
            print "Moving base by ", scoot_init
            pt.x = scoot_init
            pt.y = 0
            raw_input()
            self.basemover.move_base(pt.x,pt.y)
            

        # if color_current is blue, grab first points.        
        if color_current=='blue':
            # two arm grab
            """
            scoot_init = scoots[0] - scoot_prev
            if (scoot_init) != 0:
                print "Moving base by ", scoot_init
                pt.x = scoot_init
                pt.y = 0
                raw_input()
                self.basemover.move_base(pt.x,pt.y)                
                """
            if None not in (l_arm_points[0],r_arm_points[0]):                
                (l_x,l_y,l_z) = l_arm_points[0][0]
                (l_roll,l_pitch,l_yaw) = l_arm_points[0][1]
                ps_l = PointStamped()
                ps_l.point.x = l_x - 0.04
                ps_l.point.y = l_y
                ps_l.point.z = l_z
                ps_l.header.frame_id = util.poly_frame
                (r_x,r_y,r_z) = r_arm_points[0][0]
                (r_roll,r_pitch,r_yaw) = r_arm_points[0][1]
                ps_r = PointStamped()
                ps_r.point.x = r_x - 0.04
                ps_r.point.y = r_y
                ps_r.point.z = r_z
                ps_r.header.frame_id = util.poly_frame
                
                ### HANGING POINTS HERE
                HANGING = True if (ps_l.point.z < util.z_offset) or (ps_r.point.z < util.z_offset) else False
                FRONT_HANGING = True if (HANGING and (l_yaw !=0) and (r_yaw !=0)) else False # FIXME: right now, assumes 0 yaw only if cloth is hanging on the l or r sides of table
                if FRONT_HANGING:
                    self.goto_pre_hanging_pose()
                                
                if not GripUtils.grab_points(point_l=ps_l,roll_l=l_roll,yaw_l=l_yaw,pitch_l=l_pitch,x_offset_l=0,z_offset_l=0.003,approach= True,
                                             point_r=ps_r,roll_r=r_roll,yaw_r=r_yaw,pitch_r=r_pitch,x_offset_r=0,z_offset_r=0.0015, dur = 3.0):
                    print "Both arms failed to grab startpoints"
                    raw_input()

                if FRONT_HANGING:
                    self.goto_post_hanging_pose()

            elif (l_arm_points[0] != None):
                (x,y,z) = l_arm_points[0][0]
                HANGING = True if z < util.z_offset else False
                (roll,pitch,yaw) = l_arm_points[0][1]
                FRONT_HANGING = True if (HANGING and yaw != 0) else False # FIXME: as above
                if FRONT_HANGING:
                    self.goto_pre_hanging_pose(arm='l')

                print "Grabbing start point",l_arm_points[0]                
                if not GripUtils.grab(x = x,y=y,z=z ,arm='l',roll=roll,yaw=yaw,pitch=pitch,approach= True,frame=util.poly_frame, dur = 3.0):
                    print "Left arm failed to grab startpoint"
                    raw_input()
                if FRONT_HANGING:
                    self.goto_post_hanging_pose()

            elif (r_arm_points[0] != None):                
                (x,y,z) = r_arm_points[0][0]
                HANGING = True if z < util.z_offset else False
                (roll,pitch,yaw) = r_arm_points[0][1]
                FRONT_HANGING =True if(HANGING and yaw != 0) else False # FIXME: as above                                                                                                          
                if FRONT_HANGING:
                    self.goto_pre_hanging_pose(arm='r')
                    
                print "Grabbing start point",r_arm_points[0]
                if not GripUtils.grab(x = x,y=y,z=z ,arm='r',
                                      roll=roll,yaw=yaw,pitch=pitch,approach= True,frame=util.poly_frame, dur = 3.0):
                    print "Right arm failed to grab startpoint"
                    raw_input()

                if FRONT_HANGING:
                    self.goto_post_hanging_pose()


        i = 1
        for l_arm_point,r_arm_point in zip(l_arm_points[1:],r_arm_points[1:]):            
            print "\n\n Going to point ",i," coords are ",l_arm_point,r_arm_point
            if not (i == 2 and color_current == "red"):
                if scoots[i] != 0:                
                    print "Moving base by ",scoots[i]
                    pt.x = scoots[i]
                    pt.y = 0
                    raw_input()
                    if (RAISED_ARM == False) and (i==2):
                        raise_l = raise_r = min(0.05,abs(scoots[i]))
                        RAISED_ARM = True
                    else:
                        raise_l = raise_r = 0
                        if (l_arm_point == None):
                            raise_l = 0                    
                        if (r_arm_point == None):
                            raise_r = 0                
                    print "Raising arms to scoot by",raise_l
                    if not GripUtils.go_to_relative_multi(x_offset_l=0,y_offset_l=0,z_offset_l=raise_l,grip_l=True,x_offset_r=0,y_offset_r=0,z_offset_r=raise_r,grip_r=True,frame=util.poly_frame, dur = 0.5):
                        print "Failure to move up"
                    self.basemover.move_base(pt.x,pt.y)
            
            # two arm go to
            if None not in (l_arm_point,r_arm_point):
                (l_x,l_y,l_z) = l_arm_point[0]
                (l_roll,l_pitch,l_yaw) = l_arm_point[1]
                (r_x,r_y,r_z) = r_arm_point[0]
                (r_roll,r_pitch,r_yaw) = r_arm_point[1]
                print "Going to",l_arm_point,r_arm_point
                if not GripUtils.go_to_multi (x_l= l_x,y_l= l_y,z_l= l_z,roll_l= l_roll,pitch_l=l_pitch,yaw_l=l_yaw,grip_l=True,frame_l= util.poly_frame,
                                      x_r=r_x,y_r=r_y,z_r=r_z,roll_r=r_roll,pitch_r=r_pitch,yaw_r=r_yaw,grip_r=True,frame_r = util.poly_frame,dur=3.0):
                    print "two arm failure"
                    raw_input()                

            # one arm go to
            elif l_arm_point != None:
                 (x,y,z) = l_arm_point[0]
                 (roll,pitch,yaw) = l_arm_point[1]
                 print "Going to",l_arm_point
                 if not GripUtils.go_to(x=x,y=y,z=z,roll=roll,pitch=pitch,yaw=yaw,grip=True,frame=util.poly_frame,arm='l',dur=3.0):
                     print "left arm failure"
                     raw_input()
            elif r_arm_point != None:                
                (x,y,z) = r_arm_point[0]
                (roll,pitch,yaw) = r_arm_point[1]
                print "Going to",r_arm_point
                if not GripUtils.go_to(x=x,y=y,z=z,roll=roll,pitch=pitch,yaw=yaw,grip=True,frame=util.poly_frame,arm='r',dur=3.0):
                    print "right arm failure"
                    raw_input()
            i+=1
     
        total_scoot = sum(scoots)
        if(color_next == 'blue'):
            # If the next fold/action is 'blue', open grippers                                                                                                                            
            GripUtils.open_grippers()
            self.init_robot_pose()
        
        scoot_back = -(total_scoot) if (total_scoot < 0) else 0
        #return (True,total_scoot)           
        print "Moving base by ",scoot_back
            
        pt.x = scoot_back
        raw_input()
        self.basemover.move_base(pt.x,pt.y)
        
        return (True,0)
        

    def execute_drag(self,gripPts,d=0.1,direction='+y',color_next='blue',gripPts_next=None,endPts_next=None,scoot_prev = 0,scoot = 0):        
        """
        Grabs gripPts and moves back through distance d
        """                                                                                                                                                                                                                                                                                                                 
        print "in execute drag. direction = ",direction                                                                                                                                                                                                                                                                     
        direction = drag_direction(direction,self.robotposition)  
        (l_arm_points,r_arm_points,scoots) = self.compute_xyzrpy_drag(gripPts,d,direction,self.robotposition)
        
        """
        FIXME: Add Ziang's stuff here
        """
        pt = Point2D()
        pt.y = 0

        scoot_init = scoots[0]

        if (scoot_init) != 0:
            print "Moving base by ", scoot_init
            pt.x = scoot_init
            pt.y = 0
            raw_input()
            self.basemover.move_base(pt.x,pt.y)
            
            # grab grip points
            """                                                                                                                                                                                                                                                                                                              
            scoot_init = scoots[0] - scoot_prev                                                                                                                                                                                                                                                                             
            if (scoot_init) != 0:                                                                                                                                                                                                                                                                                           
                print "Moving base by ", scoot_init                                                                                                                                                                                                                                                                                         pt.x = scoot_init                                                                                                                                                                                                                                                                                           
                pt.y = 0                                                                                                                                                                                                                                                                                                    
                raw_input()                                                                                                                                                                                                                                                                                                 
                self.basemover.move_base(pt.x,pt.y)                                                                                                                                                                                                                                                                          
                """

        if None not in (l_arm_points[0],r_arm_points[0]):
            (l_x,l_y,l_z) = l_arm_points[0][0]
            (l_roll,l_pitch,l_yaw) = l_arm_points[0][1]
            ps_l = PointStamped()
            ps_l.point.x = l_x
            ps_l.point.y = l_y
            ps_l.point.z = util.z_offset #l_z
            ps_l.header.frame_id = util.poly_frame
            (r_x,r_y,r_z) = r_arm_points[0][0]
            (r_roll,r_pitch,r_yaw) = r_arm_points[0][1]
            ps_r = PointStamped()
            ps_r.point.x = r_x
            ps_r.point.y = r_y
            ps_r.point.z = util.z_offset #r_z
            ps_r.header.frame_id = util.poly_frame
            
            print "\n\n\nGrabbing start points",l_arm_points[0],r_arm_points[0],"\n\n\n\n"
            
            if not GripUtils.grab_points(point_l=ps_l,roll_l=l_roll,yaw_l=l_yaw,pitch_l=l_pitch,x_offset_l=0,z_offset_l=0.003,approach= True,
                                         point_r=ps_r,roll_r=r_roll,yaw_r=r_yaw,pitch_r=r_pitch,x_offset_r=0,z_offset_r=0.001, dur = 3.0):
                print "Both arms failed to grab startpoints"
                raw_input()
            #  stretch article and move up                                                                                                                                                                                                                                                                               
            if not GripUtils.go_to_relative_multi(x_offset_l=0,y_offset_l=0.02,z_offset_l=0.02,grip_l=True,x_offset_r=0,y_offset_r=-0.02,z_offset_r=0.02,grip_r=True,frame=util.poly_frame, dur = 0.5):
                print "Failure to move up"

        elif (l_arm_points[0] != None):
            (x,y,z) = l_arm_points[0][0]
            z = util.z_offset
            (roll,pitch,yaw) = l_arm_points[0][1]
            print "Grabbing start point",l_arm_points[0]
            if not GripUtils.grab(x = x,y=y,z=z ,arm='l',roll=roll,yaw=yaw,pitch=pitch,approach= True,frame=util.poly_frame):
                print "Left arm failed to grab startpoint"
                raw_input()
                    
            #  move left arm up 
            if not GripUtils.go_to_relative_multi(x_offset_l=0,y_offset_l=0.00,z_offset_l=0.02,grip_l=True,x_offset_r=0,y_offset_r=0,z_offset_r=0,grip_r=False,frame=util.poly_frame, dur = 0.5):
                print "Failure to move up"

        elif (r_arm_points[0] != None):
            (x,y,z) = r_arm_points[0][0]
            (roll,pitch,yaw) = r_arm_points[0][1]
            print "Grabbing start point",r_arm_points[0]
            if not GripUtils.grab(x = x,y=y,z=z ,arm='r',
                                  roll=roll,yaw=yaw,pitch=pitch,approach= True,frame=util.poly_frame):
                print "Right arm failed to grab startpoint"
                raw_input()
                
            #  move right arm up                                                                                                                                                                                                                                                                                        
            if not GripUtils.go_to_relative_multi(x_offset_l=0,y_offset_l=0.00,z_offset_l=0,grip_l=False,x_offset_r=0,y_offset_r=0,z_offset_r=0.02,grip_r=True,frame=util.poly_frame, dur = 0.5):
                print "Failure to move up"

        if True: # Dragging backwards only
            # Move back through distance d + 0.03 for error                                                                                                                                                                                                                                                                 
            OVERSHOOT_AMT = 0 #0.05*d
            pt.x = -abs(d + OVERSHOOT_AMT)
            pt.y = 0
            print "Moving back by ",(d+ OVERSHOOT_AMT)
            raw_input()
            error = self.basemover.move_base(pt.x,pt.y)
            if (error[0] < 0):
                # correct for undershoot with arms
                print "Drag correction by",error[0]
                grip_l = True if l_arm_points[0]!= None else False
                grip_r = True if r_arm_points[0]!=None else False
                if not GripUtils.go_to_relative_multi(x_offset_l=error[0],y_offset_l=0.00,z_offset_l=0,grip_l=grip_l,x_offset_r=error[0],y_offset_r=0,z_offset_r=0,grip_r=grip_r,frame=util.poly_frame, dur = 0.5):
                    print "Failure to drag further"
                

            # if next fold is blue, open grippers                                                                                                                                                                                                                                                                                
        if(color_next == 'blue'):
            GripUtils.open_grippers()
            self.init_robot_pose()
        else:
            (l_arm_points,r_arm_points,scoots) = self.compute_xyzrpy_fold(gripPts_next,endPts_next,self.robotposition,color_next)
            if(l_arm_points[0] == l_arm_points[1] == l_arm_points[2] == None):
                # right arm red fold                                                                                                                                                                                                                                                                                    
                GripUtils.open_gripper(arm = 'l')
                self.init_left_arm()
                z_offset_r = min(0.1,d)
                z_offset_l = 0
            elif(r_arm_points[0] == r_arm_points[1] == r_arm_points[2] == None):
                # left arm red fold                                                                                                                                                                                                                                                                                     
                GripUtils.open_gripper(arm = 'r')
                self.init_right_arm()
                z_offset_r = 0
                z_offset_l = min(0.1,d)
            else:
                z_offset_l = z_offset_r = min(0.05,d)
                    # two arm red fold                                                                                                                                                                                                                                                                                      
            grip_l = True if z_offset_l != 0 else False
            grip_r = True if z_offset_r != 0 else False

        if not GripUtils.go_to_relative_multi(x_offset_l=0,y_offset_l=0,z_offset_l=z_offset_l,grip_l=grip_l,x_offset_r=0,y_offset_r=0,z_offset_r=z_offset_r,grip_r=grip_r,frame=util.poly_frame, dur = 0.5):
            print "Failure to move up"
                #return False                                                                                                                                                                                                                                                                                                
        if color_next == "red":
            print "returning from drag. scoot_prev is",(pt.x + scoot_init), "scoot_init = ",scoot_init
            return (True,(pt.x + (scoot_init)))

        #if direction in ['b','r','l']:                                                                                                                                                                                                                                                                                     
        # return to original pose                                                                                                                                                                                                                                                                                            
        pt.x = -(pt.x) + (-scoots[0])
        pt.y = 0

        #return (True, - (d+OVERSHOOT_AMT))                                                                                                                                                                                                                                                       
        print "Moving front by ", pt.x + (-scoots[0])
        raw_input()
        self.basemover.move_base(pt.x,pt.y)
        return (True,0)    

    def move_cost(self, start_station, end_station):
        return self.costcalculator.station_nav_cost(start_station, end_station)

    def execute_move(self,dest,scoot_prev):
        dest = dest+"_scoot"
        if RECORD_FLAG:
            base_diff = self.costcalculator.get_base_pose(dest, array=True) -\
                self.costcalculator.get_base_pose(self.robotposition, array=True)
            log_action('move', [base_diff.tolist()], [])

        if os.environ['ROBOT_MODE'] == 'sim':
            set_sim_state.set_station('/stations/'+dest, self.listener)
            self.robotposition = dest
            return
        commands.getoutput('rosrun pr2_mechanism_controllers send_periodic_cmd_srv.py laser_tilt_controller linear 2 4.0 0.0')        
        
        print "going to station", dest
        raw_input("hit any key to confirm. Make sure tilt lasers are on and SnapMapICP is dead")
        self.nav_server.go_to_station(dest,init_scoot=scoot_prev)
        self.robotposition = dest
        commands.getoutput('rosrun pr2_mechanism_controllers send_periodic_cmd_srv.py laser_tilt_controller linear 0 0 0.0')
        return (True,0)

    def init_robot_pose(self):
        """
        makes PR2 look down at table and put arms up
        """
        if False: #SIM_FLAG:
            return True
        if RECORD_FLAG:
            log_action('init', [], [])

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
                                        frame_l="torso_lift_link", frame_r="torso_lift_link", dur=2.0):
            return False
        else:
            return True

    def init_left_arm(self):
        height = 0.35
        lateral_amount = 0.65
        forward_amount = 0.3
        if not GripUtils.go_to(   x=forward_amount, y=lateral_amount, z=height,
                                  roll=0, pitch=0, yaw=0, grip=False, arm = 'l',frame = "torso_lift_link",
                                  dur=2.0):
            return False
        else:
            return True

    def init_right_arm(self):
        height = 0.35
        lateral_amount = 0.65
        forward_amount = 0.3
        if not GripUtils.go_to(   x=forward_amount, y=-lateral_amount, z=height,
                                  roll=0, pitch=0, yaw=0, grip=False, arm = 'r',frame = "torso_lift_link",
                                  dur=2.0):
            return False
        else:
            return True

    def print_costs(self):
        print "overhead", self.costcalculator.get_overhead(), "ik time",self.costcalculator.get_ik_time(),\
            'num ik calls', self.costcalculator.pac.num_ik_calls       
        
def drag_direction(direction,robotposition):
    """
    hacky conversion between what FoldingSearch returns and a drag direction relative to robot
    """
    robotposition = robotposition.replace('_scoot', '')

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
