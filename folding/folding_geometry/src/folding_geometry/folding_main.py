#!/usr/bin/env python                                                            
                                                                                                               
""" 
FoldMain.py

Author: Karthik Lakshmanan

Description: Main program for Geometry based folding. Interfaces Robot model with GUI and planner
"""
 
import roslib
roslib.load_manifest("folding_geometry")
import rospy
import sys
from geometry_msgs.msg import PointStamped,PoseStamped
from arm_navigation_msgs.msg import Shape as GeometricShape
from folding_msgs.msg import PolyStamped,Point2D,FoldTraj
from folding_geometry.FoldingGUI import *
from shape_window import Geometry2D
import FoldingSearch
from numpy import *
import time
import Robot
import util
import tf

TABLE_FLAG = False
class FoldingMain():
    def __init__(self):
        util.listener = tf.TransformListener()
        self.robot = Robot.Robot()        
        self.gui = FoldingGUI(name="Geometry_Based_Folding")    
        self.mode = "towel"
        self.poly_sub = rospy.Subscriber("input",PolyStamped,self.poly_handler)
        #self.scale_factor = self.x_offset = self.y_offset = self.poly_frame = False
        util.scale_factor = 5.0/0.0254
        self.poly_points = []
        now = time.localtime()
        self.logfile = open('/tmp/folding_%04d-%02d-%02d__%02d-%02d-%02d.log'%(now.tm_year,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec),'w')
        self.poly_sub = rospy.Subscriber("input",PolyStamped,self.poly_handler)
        self.start_time = rospy.Time.now()        
        rospy.loginfo("READY TO GO")

    #Receives a stream of polygon vertices and updates the poly appropriately                                             
    def poly_handler(self,stamped_poly):
        rospy.loginfo("RECEIVED A POINT")        
        #self.robot.arms_test()
        #util.poly_frame = "stations/table_front_scoot" #stamped_poly.header.frame_id        
        #util.z_offset = stamped_poly.z_offset
        #print "z_offset=",util.z_offset                                                                                      
        points = stamped_poly.vertices #[Geometry2D.Point(point.x,point.y) for point in stamped_poly.vertices]                
        vertices = [util.convert_from_world_frame(point) for point in points]
        # the first 6 define the table edge
        if TABLE_FLAG:
            print vertices
            vertices = vertices[3:]
        else:
            self.table_detector(vertices)
            return

        if len(vertices) == 0:
            return

        poly = Geometry2D.Polygon(*vertices)
        self.poly_cache = poly
        cvPoly = CVPolygon(Colors.GREEN,self.gui.front(self.gui.shapes),poly)
        self.gui.clearShapes()        
        self.gui.addCVShape(cvPoly)
        self.handle_automatic_folds(self.gui.getPolys()[0].getShape().vertices())


    # waits for 6 points and sets table
    def table_detector(self,vertices):
        global TABLE_FLAG
        if len(vertices) == 3:
            TABLE_FLAG = True
            self.gui.createTable(vertices)
            self.gui.drawAllTable()
            return True
        return False

    #Waits til it has received enough, then folds the article sketched                                                   
    def handle_automatic_folds(self,vertices):        
        if len(vertices) == 10 and self.mode == "shirt":
            self.start_logging()
            self.gui.foldShirt_v3()
            self.stop_logging()
        elif len(vertices) == 10 and self.mode == "tee":
            self.start_logging()
            self.gui.foldTeeNoSleeve()
            solution = FoldingSearch.FoldingSearch(self.gui,self.robot,self.gui.startpoly)
            self.stop_logging()
        elif len(vertices) == 7 and self.mode == "pants":
            self.start_logging()
            self.gui.foldPants_v2()
            self.stop_logging()
        elif len(vertices) == 4 and self.mode == "towel":
            #self.start_logging()
            #vertices = [util.convert_to_world_frame(vertex) for vertex in vertices]
            #vertices = [(self.robot.convert_to_robot_frame(vertex,"table_front")) for vertex in vertices]            
            [bl,tl,tr,br] = vertices
            #self.robot.arms_test(None,'l')
            #self.robot.arms_test(tl,'l')
            #self.robot.arms_test(tr,'r')
            #self.robot.arms_test(br,'r')
            self.gui.foldTowelThirds()            
            solution = FoldingSearch.FoldingSearch(self.gui,self.robot,self.gui.startpoly)
            self.robot.print_costs()
            print "Brett:: Hit a key to make me fold!"
            raw_input()
            self.execute_actions(solution)
            #print bl,tl,tr,br
            #pt = Geometry2D.Point(130, 400)
            #pt3d = self.gui.convertPts2Dto3D(pt)
            #self.robot.arms_test(pt3d)
            #self.stop_logging()

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

    def execute_actions(self,states):
        """
        now execute the actions returned by the search
        """
        i = 1
        for state in states[1:]:
            action = state.action            
            print "action is ",action
            # transform points to current frame of robot
            #gripPts3d, endPts3d = self.gui.convertGripPts(action.get_gripPoints(), action.get_endPoints())
            if action.get_actionType() in ("drag"):
                gripPts = [self.robot.convert_to_robot_frame(util.convert_to_world_frame(gripPt),self.robot.robotposition) for gripPt in gripPts3d]
            if action.get_actionType() == "fold":
                #endPts = [self.robot.convert_to_robot_frame(util.convert_to_world_frame(endPt),self.robot.robotposition) for endPt in endPts3d]
                color_current = action.get_foldType()
            # find type of next action
            color_next = states[i].action.get_foldType() if (i < len(states) and states[i].action.get_actionType()=="fold") else "blue"
            if action.get_actionType() == "fold":
                SUCCESS = self.robot.execute_fold(gripPts,endPts,color_current,color_next)
            elif action.get_actionType() == "drag":
                SUCCESS = self.robot.execute_drag(gripPts,endPts,action.get_dragDirection(),color_next)

            if not SUCCESS:
                rospy.loginfo("Failure to execute %s",action.get_actionType())
                break
            i+=1 

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


def main(args):
    rospy.init_node("folding_main")
    rospy.sleep(3)    
    FM = FoldingMain()
    raw_input("Folding Main Called")
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
