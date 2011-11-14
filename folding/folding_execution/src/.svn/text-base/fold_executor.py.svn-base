#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_execution")
import rospy
import sys
from geometry_msgs.msg import PointStamped,PoseStamped
from folding_msgs.msg import FoldTraj
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
from folding_srvs.srv import *
import StanceUtils,PrimitiveUtils
from pr2_simple_arm_motions import GripUtils
from numpy import *
import tf
import time
import thread

## The FoldExecutor does the fold manipulations. Much of this was coded over the summer, and as such it's fairly old and hackish.
# Note in particular that many stances are hard coded, and GripUtils is ignored in lieu of service calls. This should be fixed.
class FoldExecutor:
    def __init__(self):
        self.queue = []
        self.fold_srv = rospy.Service("execute_fold",ExecuteFold,self.execute_fold)
        self.listener = tf.TransformListener()
        self.last_traj = False
        self.last_arm = "l"
        self.hold_arms_flag = False
        self.hold_arms_lock = thread.allocate_lock()
        self.continually_hold_lock = thread.allocate_lock()
        self.init_stances()
   
   #Initialize the stances (hard-coded poses to go in)
    def init_stances(self):
        my_stances = {}
        
        folding_height_stance = Stance  (set_torso_height=True,torso_height=0.3,arms="n")
        my_stances["folding_height"] = folding_height_stance
        viewing_height_stance = Stance  (set_torso_height=True,torso_height=0.2,arms="n")
        my_stances["viewing_height"] = viewing_height_stance
        folding_stance = Stance(x_left=0.65,y_left=0.3,z_left=0.2,roll_left=pi/2,pitch_left=pi/4,yaw_left=0,grip_left=False,frame_left="torso_lift_link"
                                    ,x_right=0.65,y_right=-0.3,z_right=0.2,roll_right=pi/2,pitch_right=pi/4,yaw_right=0,grip_right=False,frame_right="torso_lift_link"
                                    ,arms="b")                                 
        my_stances["folding"] = folding_stance
        folding_right_stance = Stance(x_left=0.65,y_left=0.3,z_left=0.2,roll_left=pi/2,pitch_left=pi/4,yaw_left=0,grip_left=False,frame_left="torso_lift_link"
                                    ,x_right=0.65,y_right=-0.3,z_right=0.2,roll_right=pi/2,pitch_right=pi/4,yaw_right=0,grip_right=False,frame_right="torso_lift_link"
                                    ,arms="r")
        my_stances["folding_right"] = folding_right_stance  
        folding_left_stance = Stance(x_left=0.65,y_left=0.3,z_left=0.2,roll_left=pi/2,pitch_left=pi/4,yaw_left=0,grip_left=False,frame_left="torso_lift_link"
                                    ,x_right=0.65,y_right=-0.3,z_right=0.2,roll_right=pi/2,pitch_right=pi/4,yaw_right=0,grip_right=False,frame_right="torso_lift_link"
                                    ,arms="l")
        my_stances["folding_left"] = folding_left_stance        
        viewing_stance = Stance (x_left=0.1,y_left=0.7,z_left=0.3,roll_left=pi/2,pitch_left=-pi/4,yaw_left=0,grip_left=False,frame_left='torso_lift_link'
                                ,x_right=0.1,y_right=-0.7,z_right=0.3,roll_right=pi/2,pitch_right=-pi/4,yaw_right=0,grip_right=False,frame_right='torso_lift_link'
                                ,arms="b")
        my_stances["viewing"] = viewing_stance
        viewing_stance_left = Stance (x_left=0.1,y_left=0.7,z_left=0.3,roll_left=pi/2,pitch_left=-pi/4,yaw_left=0,grip_left=False,frame_left='torso_lift_link'
                                ,x_right=0.1,y_right=-0.7,z_right=0.3,roll_right=pi/2,pitch_right=-pi/4,yaw_right=0,grip_right=False,frame_right='torso_lift_link'
                                ,arms="l")
        my_stances["viewing_left"] = viewing_stance_left
        viewing_stance_right = Stance (x_left=0.1,y_left=0.7,z_left=0.3,roll_left=pi/2,pitch_left=-pi/4,yaw_left=0,grip_left=False,frame_left='torso_lift_link'
                                ,x_right=0.1,y_right=-0.7,z_right=0.3,roll_right=pi/2,pitch_right=-pi/4,yaw_right=0,grip_right=False,frame_right='torso_lift_link'
                                ,arms="r")
        my_stances["viewing_right"] = viewing_stance_right
        #Head stances
        my_stances["look_bottom_left"] = Stance(arms="n",set_head_angle=True,head_pan=0.6,head_tilt=1.25)
        my_stances["look_top_left"] = Stance(arms="n",set_head_angle=True,head_pan=0.6,head_tilt=0.9)
        my_stances["look_bottom_right"] = Stance(arms="n",set_head_angle=True,head_pan=-0.6,head_tilt=1.25)
        my_stances["look_top_right"] = Stance(arms="n",set_head_angle=True,head_pan=-0.6,head_tilt=0.9)
        my_stances["look_center"] = Stance(arms="n",set_head_angle=True,head_pan=0.0,head_tilt=1.0)
        StanceUtils.add_stances(my_stances)
        initialize = PrimitiveUtils.make_primitive(stances=["folding_height","viewing"],durs=[5.0,5.0],params=[[],[]])
        PrimitiveUtils.add_primitive(primitive=initialize,name="initialize")
        scan_cloth = PrimitiveUtils.make_primitive(
                                                    stances=["viewing","look_bottom_left","pause","look_top_left","pause","look_top_right","pause","look_bottom_right","pause"],
                                                    durs=[5.0,5.0,25.0,5.0,25.0,5.0,25.0,5.0,25.0],
                                                    params=[[],[],[],[],[],[],[],[],[]]
                                                    )

        PrimitiveUtils.add_primitive(primitive=scan_cloth,name="scan_cloth")
   
    #Execute a fold the service sends our way
    def execute_fold(self,req):

        success = self.handle_fold(req.fold_traj)
        return ExecuteFoldResponse(success)
    
    #Store up queue of future folds if we're overloaded
    def handle_fold(self,fold_traj):
        if len(self.queue) == 0:
            self.execute(fold_traj)
        else:
            self.queue.append(fold_traj)
        return True
        
    # Perform the folding
    def execute(self,fold_traj):
        if len(fold_traj.grip_points) > 2:
            rospy.loginfo("Too many grip points!")
        elif len(fold_traj.grip_points) == 2:
            self.executeBiGrip(fold_traj)
        elif len(fold_traj.grip_points) == 1:
            self.executeSingleGrip(fold_traj)
        else:
            rospy.loginfo("Sent empty fold! What gives?")
        self.check_queue()
    
        
    def sendTarget(self, dur, target1, target2=False,grab=False):
        resp = False
        if grab:
            point = target1.point
            pitch = target1.pitch
            roll = target1.roll
            yaw = target1.yaw
            GripUtils.grab_point(point=point,pitch=pitch,roll=roll,yaw=yaw,arm=target1.arm,approach=False)
            return
        target1.point.header.stamp = rospy.Time.now()
        if not target2:
            try:
                srv = rospy.ServiceProxy("move_one_arm",MoveOneArm)
                resp = srv(MoveOneArmRequest(target=target1,dur=dur))
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
        else:
            target2.point.header.stamp = rospy.Time.now()
            assert target1.arm != target2.arm
            try:
                srv = rospy.ServiceProxy("move_both_arms",MoveBothArms)
                target_left = target_right = []
                if target1.arm == "l":
                    target_left = target1
                    target_right = target2
                else:
                    target_left = target2
                    target_right = target1
                resp = srv(MoveBothArmsRequest(target_left=target_left,target_right=target_right,dur=dur))
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
    
    def rotate(self,x,y,tilt):
        new_y = cos(tilt)*y + sin(tilt)*x
        new_x = -sin(tilt)*y + cos(tilt)*x
        return (new_x,new_y)
    
    def gripPoints  (self,point,arm,tilt,grip,preferred_pitch="DEFAULT",preferred_roll="DEFAULT"
                    ,point2=None,arm2="n",tilt2=0,grip2=0,preferred_pitch2=0,preferred_roll2=0
                    ,red=False,grab=False):
        (target1,mode1) = self.gripAt(point=point,arm=arm,tilt=tilt,grip=grip,preferred_pitch=preferred_pitch,preferred_roll=preferred_roll,red=red)
        target2 = False
        mode2 = ""
        if point2:
            (target2,mode2) = self.gripAt(point=point2,arm=arm2,tilt=tilt2,grip=grip2,preferred_pitch=preferred_pitch2,preferred_roll=preferred_roll2,red=red)
        self.sendTarget(3.0,target1,target2,grab=grab)
        return (target1,target2)
            
        
             
    def gripAt(self,point,arm,tilt,grip,preferred_pitch = "DEFAULT", preferred_roll = "DEFAULT",red=False):
        point.header.stamp = rospy.Time.now()
        self.listener.waitForTransform("torso_lift_link",point.header.frame_id,point.header.stamp,rospy.Duration(4.0))
        torso_point = self.listener.transformPoint("torso_lift_link",point)
        print "Received request to move arm %s to point (%f,%f,%f)"%(arm,torso_point.point.x,torso_point.point.y,torso_point.point.z)
        y_sign = 1
        if arm=="r":
            y_sign *= -1
        pitch = 0
        roll = 0
        x = torso_point.point.x
        
        y = torso_point.point.y * y_sign
        z = torso_point.point.z
        if arm == "l":
            torso_point.point.z -= 0.01
        
        #Make elipse
        PITCH_DISPL = 0.09
        MAX_DIST = 0.23#Was 0.20
        centered_x = x - 0.59 #Was 59
        centered_y = y - 0.2
        centered_z = z

        dist = sqrt((centered_x/1.05)**2 + (centered_y / 1.9)**2)
        (new_x,new_y) = self.rotate(centered_x,centered_y / 2,tilt*-1*y_sign)
        if dist > MAX_DIST + PITCH_DISPL*2/3:
            if new_x > 0.1:
                mode = "unreachable_front"
            elif new_x < -0.1:
                mode = "unreachable_back"
            else:
                mode = "free"
        elif MAX_DIST+PITCH_DISPL*2/3 >= dist > MAX_DIST:
            if new_x > 0.05:
                mode = "outer_front"
            elif new_x < -0.05:
                mode = "outer_back"
            else:
                mode = "free"
        elif MAX_DIST >= dist > MAX_DIST - (PITCH_DISPL*3/2):
            if new_x > 0.0:        
                mode = "bounded_front"
            elif new_x < -0.0:
                mode = "bounded_back"
            else:
                mode = "free"
        else:
            print "LEGITIMATELY FREE"
            mode = "free"
        if mode=="outer_front" or mode=="unreachable_front":
            pitch = pi/4
            roll = pi/2
            
        elif mode=="outer_back":
            pitch = pi/2
            roll = pi/2
            torso_point.point.z -= 0.005
        elif mode=="unreachable_back":
            pitch = 2*pi/3
            roll=pi/2
            torso_point.point.z -= 0.005
        elif mode=="bounded_front":
            pitch = pi/2
            roll = pi/2
            if preferred_pitch != "DEFAULT" and preferred_pitch <= pi/2:
                    pitch = preferred_pitch
            if preferred_roll != "DEFAULT":
                    roll = preferred_roll
        elif mode=="bounded_back":
            pitch = pi/2
            roll = pi/2
            if preferred_pitch != "DEFAULT" and preferred_pitch >= pi/2:
                    pitch = preferred_pitch
            if preferred_roll != "DEFAULT":
                    roll = preferred_roll
                    
        else:
            pitch = pi/2
            roll = pi/2
            if preferred_pitch != "DEFAULT":
                    pitch = preferred_pitch
            if preferred_roll != "DEFAULT":
                    roll = preferred_roll
        
        if z > 0.08:
            if pitch >= pi/2:
                pitch = pi/4
        
        if z > 0.1:
            pitch = 0
        if pitch >= pi/2:
            torso_point.point.z -= 0.025
            
        print "Returned RPY = (%f,%f,%f)"%(roll,pitch,tilt)
        rospy.loginfo("Mode for arm %s: %s"%(arm,mode))
        return (GripTarget(point=torso_point,grip=grip,roll=roll, pitch=pitch,yaw=tilt,arm=arm),mode)
    
            
    def torso_pt(self,pt):
        now = rospy.Time.now()
        pt.header.stamp = now
        self.listener.waitForTransform("torso_lift_link",pt.header.frame_id,now,rospy.Duration(4.0))
        return self.listener.transformPoint("torso_lift_link",pt)
        
    def is_backwards(self,fold_traj):
        grip = self.torso_pt(fold_traj.grip_points[0])
        goal = self.torso_pt(fold_traj.goal_points[0])
        return goal.point.x < grip.point.x
            
    def executeSmooth(self,fold_traj):
        if fold_traj == False:
            return False
        if fold_traj.ignore_smooth:
            return False
        if pi/4 <= abs(fold_traj.tilts[0]) <= 3*pi/4:
            return False
        scale = 1
        if self.is_backwards(fold_traj):
            scale = -1
        StanceUtils.open_gripper("b")
        self.fold_stance("b")
        GRIPPER_LENGTH = 0.195
        center = fold_traj.smooth_center
        left_start = self.torso_pt(center)
        right_start = self.torso_pt(center)
        left_start.point.y += 0.07
        left_start.point.x += scale*0.1
        left_start.point.z += GRIPPER_LENGTH + 0.01
        right_start.point.y -= 0.07
        right_start.point.x += scale*0.1
        right_start.point.z += GRIPPER_LENGTH + 0.01
        left_end = self.torso_pt(fold_traj.smooth_edges[0])
        left_end.point.z += GRIPPER_LENGTH + 0.01
        left_end.point.x += scale*0.08
        right_end = self.torso_pt(fold_traj.smooth_edges[1])
        right_end.point.z += GRIPPER_LENGTH + 0.01
        right_end.point.x += scale*0.08
        left_tilt = 0
        right_tilt = 0
        #Start moving
        target_left =GripTarget(point=left_start,arm="l",yaw=left_tilt,grip=True,pitch=-pi/2,roll=0)
        target_right = GripTarget(point=right_start,arm="r",yaw=right_tilt,grip=True,pitch=-pi/2,roll=0)
        self.sendTarget(5.0,target_left,target_right)
        target_left = GripTarget(point=left_end,arm="l",yaw=left_tilt,grip=True,pitch=-pi/2,roll=0)
        target_right = GripTarget(point=right_end,arm="r",yaw=right_tilt,grip=True,pitch=-pi/2,roll=0)
        self.sendTarget(5.0,target_left,target_right)
        self.fold_stance("b")
        return True
    
    def executeSingleGrip(self, fold_traj):
        approach_point = fold_traj.approach_points[0]
        grip_point = fold_traj.grip_points[0]
        quarter_point = fold_traj.quarter_points[0]
        vertical_point = fold_traj.vertical_points[0]
        goal_point = fold_traj.goal_points[0]
        smooth_center = fold_traj.smooth_center
        smooth_edges = fold_traj.smooth_edges
        first_adjustment = False
        tilt = fold_traj.tilts[0]
        roll = pi/2
        pitch = pi/4
        if fold_traj.red:
            tilt = self.last_traj.tilts[0]%(2*pi)
        rospy.loginfo("Tilt: %s"%tilt)
        
        #We do this so that we don't let go of the vertex during a red fold
        if not fold_traj.red:
            self.executeSmooth(self.last_traj)
            StanceUtils.open_gripper("b")
            self.fold_stance("b")
            arm = self.getArm(fold_traj)
            #Move the unused arm out of the way
            if arm=="l":
                StanceUtils.call_stance("viewing_right",5.0)
            else:
                StanceUtils.call_stance("viewing_left",5.0)
  
            (target,nothing) = self.gripPoints(point=approach_point,arm=arm,tilt=tilt,grip=False,preferred_pitch = pi/4,preferred_roll=roll,red=fold_traj.red)
            pitch = target.pitch
            self.gripPoints(point=grip_point,arm=arm,tilt=tilt,grip=False,preferred_pitch = pi/4,preferred_roll=roll,red=fold_traj.red,grab=True)
            
            StanceUtils.close_gripper(arm)
            
        else:
            arm = self.last_arm
        StanceUtils.close_gripper(arm)
        self.gripPoints(point=vertical_point,grip=True,tilt=tilt,arm=arm,preferred_pitch=pi/2,preferred_roll=roll,red=fold_traj.red)
        self.gripPoints(point=goal_point,grip=True,tilt=tilt,arm=arm,preferred_pitch=3*pi/5,preferred_roll=roll,red=fold_traj.red)
        
        self.last_traj = fold_traj
        self.last_arm = arm
        

    def executeBiGrip(self, fold_traj):
        tilts = fold_traj.tilts
        approach_points = fold_traj.approach_points
        grip_points = fold_traj.grip_points
        quarter_points = fold_traj.quarter_points
        weight_points = fold_traj.weight_points
        vertical_points = fold_traj.vertical_points
        goal_points = fold_traj.goal_points
        
        rolls = [pi/2,pi/2]
        #pitches = [pi/4,pi/4]
        pitches = [pi/4,pi/4]
        if not fold_traj.red:
            self.executeSmooth(self.last_traj)
            
            self.fold_stance("b")
            arms = self.getArms(fold_traj)
            #Initially, both are in "approach" mode
            (target1,target2) = self.gripPoints(point=approach_points[0],grip=False,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/4,preferred_roll=rolls[0],
                point2=approach_points[1],grip2=False,tilt2=tilts[1],arm2=arms[1],preferred_pitch2=pi/4,preferred_roll2=rolls[1])
            #pitches[0] = target1.pitch
            #Do the first pickup and nothing with the other arm
            self.gripPoints(point=grip_points[0],grip=False,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/4,preferred_roll=rolls[0],grab=True)
            StanceUtils.close_gripper(arms[0])
            #Do the second pickup, with the first arm interpolated
            first_dx = vertical_points[0].point.x - grip_points[0].point.x
            first_dy = vertical_points[0].point.y - grip_points[0].point.y
            first_dz = vertical_points[0].point.z - grip_points[0].point.z
            second_dx = vertical_points[1].point.x - grip_points[1].point.x
            second_dy = vertical_points[1].point.y - grip_points[1].point.y
            second_dz = vertical_points[1].point.z - grip_points[1].point.z
            interp_dx = first_dx - second_dx
            interp_dy = first_dy - second_dy
            interp_dz = first_dz - second_dz
            interp_x = grip_points[0].point.x + interp_dx
            interp_y = grip_points[0].point.y + interp_dy
            interp_z = grip_points[0].point.z + interp_dz
            interp_pt = PointStamped()
            interp_pt.point.x = interp_x
            interp_pt.point.y = interp_y
            interp_pt.point.z = interp_z
            interp_pt.header.frame_id = grip_points[0].header.frame_id
            #Go to the second approach point
            (target2,nothing) = self.gripPoints(point=interp_pt,grip=True,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/4,preferred_roll=rolls[0],
                                                point2=approach_points[1],arm2=arms[1],grip2=False,tilt2=tilts[1],preferred_pitch2=pi/4,preferred_roll2=rolls[1])
            #pitches[1] = target2.pitch
            self.gripPoints (point=interp_pt,grip=True,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/4,preferred_roll=rolls[0]
                            ,point2=grip_points[1],grip2=False,tilt2=tilts[1],arm2=arms[1],preferred_pitch2=pi/4,preferred_roll2=rolls[1])
            self.gripPoints(point=grip_points[1],grip=False,tilt=tilts[1],arm=arms[1],preferred_pitch=pi/4,preferred_roll=rolls[1],grab=True)

            StanceUtils.close_gripper(arms[1])
        else:
            arms = self.getArms(fold_traj)
        #Bring both to middle

        self.gripPoints (point=vertical_points[0],grip=True,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/2,preferred_roll=rolls[0]
                        ,point2=vertical_points[1],grip2=True,tilt2=tilts[1],arm2=arms[1],preferred_pitch2=pi/2,preferred_roll2=rolls[1]) #FIX pi/4 -> pi/2

        #Set first one down
        first_dx = goal_points[0].point.x - vertical_points[0].point.x
        first_dy = goal_points[0].point.y - vertical_points[0].point.y
        first_dz = goal_points[0].point.z - vertical_points[0].point.z
        second_dx = goal_points[1].point.x - vertical_points[1].point.x
        second_dy = goal_points[1].point.y - vertical_points[1].point.y
        second_dz = goal_points[1].point.z - vertical_points[1].point.z
        interp_dx = first_dx - second_dx
        interp_dy = first_dy - second_dy
        interp_dz = first_dz - second_dz
        interp_x = goal_points[0].point.x - interp_dx
        interp_y = goal_points[0].point.y - interp_dy
        interp_z = goal_points[0].point.z - interp_dz
        interp_pt = PointStamped()
        interp_pt.point.x = interp_x
        interp_pt.point.y = interp_y
        interp_pt.point.z = interp_z
        interp_pt.header.frame_id = grip_points[0].header.frame_id
        self.gripPoints  (point=interp_pt,grip=True,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/2,preferred_roll=rolls[0]
                         ,point2=goal_points[1],grip2=True,tilt2=tilts[1],arm2=arms[1],preferred_pitch2=pi/2,preferred_roll2=rolls[1])
        #Finally, set last one down
        self.gripPoints(point=goal_points[0],grip=True,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/2,preferred_roll=rolls[0]
                        ,point2=goal_points[1],grip2=True,tilt2=tilts[1],arm2=arms[1],preferred_pitch2=pi/2,preferred_roll2=rolls[1])
        GripUtils.open_grippers()
        goal_points[0].point.z += 0.03
        goal_points[1].point.z += 0.03
        self.gripPoints(point=goal_points[0],grip=False,tilt=tilts[0],arm=arms[0],preferred_pitch=pi/2,preferred_roll=rolls[0]
                        ,point2=goal_points[1],grip2=False,tilt2=tilts[1],arm2=arms[1],preferred_pitch2=pi/2,preferred_roll2=rolls[1])
        self.last_traj = fold_traj
    
        
        
    def getArm(self, fold_traj):
        now = rospy.Time.now()
        fold_traj.grip_points[0].header.stamp = now
        self.listener.waitForTransform("torso_lift_link",fold_traj.grip_points[0].header.frame_id,now,rospy.Duration(10.0))
        grip_pt_torso_link = self.listener.transformPoint("torso_lift_link",fold_traj.grip_points[0])
        if grip_pt_torso_link.point.y > 0:
            return "l"
        else:
            return "r"
    
    def getArms(self, fold_traj):
        now = rospy.Time.now()
        for pt in fold_traj.grip_points:
            pt.header.stamp = now
        self.listener.waitForTransform("torso_lift_link",fold_traj.grip_points[0].header.frame_id, now,rospy.Duration(4.0))
        
        grip_pts_torso_link = [self.listener.transformPoint("torso_lift_link",pt) for pt in fold_traj.grip_points]
        angles = [arctan(pt.point.y/(pt.point.x)) for pt in grip_pts_torso_link]
        if angles[0] > angles[1]:
            return ("l","r")
        else:
            return ("r","l")
    
    def check_queue(self):
        if len(self.queue) > 0:
            self.execute(self.queue.pop())
            
    def fold_stance(self,arm):
        if arm=="b":
            StanceUtils.call_stance("folding",3.0)
        elif arm=="l":
            StanceUtils.call_stance("folding_left",3.0)
        elif arm=="r":
            StanceUtils.call_stance("folding_right",3.0)
            
    def viewing_stance(self,arm):
        StanceUtils.call_stance("viewing",3.0)
    
def main(args):
    rospy.init_node("fold_executor")
    fe = FoldExecutor()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
