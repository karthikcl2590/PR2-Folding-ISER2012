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
import signal, sys, time
from FoldingSearch import Action
import logging

TABLE_FLAG = False
EXECUTE_FLAG = False
RECORD_FLAG = True
SIM_FLAG = True

def get_execute_tee_actions():

    ''' Returned from search result '''

    actions = []

    node1 = Action("move",
        [],
        [],
        moveDestination = "table_front_left"\
    )

    node2 = Action("fold",
        gripPoints = [Geometry2D.Point(155.00,306.000)],
        endPoints = [Geometry2D.Point(204.335691, 306.384686)],
        foldType = 'blue',
        foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(178.787457, 416.245950),
            Geometry2D.Point(181.187457, 116.245950)\
        )\
    )

    node3 = Action("fold",
        gripPoints = [Geometry2D.Point(179.00, 291.00), Geometry2D.Point(179.00, 416.000)],
        endPoints = [Geometry2D.Point(220.775899, 291.081062), Geometry2D.Point(220.175903, 416.079622)],
        foldType = "blue",
        foldLine = Geometry2D.DirectedLineSegment(\
            Geometry2D.Point(195.587457, 416.245950),
            Geometry2D.Point(196.067457, 216.245950)\
        )\
    )

    node4 = Action("drag",
        gripPoints = [Geometry2D.Point(205.0000, 295.000),
        Geometry2D.Point(205.0000, 416.0000)],
            endPoints = [],
            dragDirection = "-x",
            dragDistance = 55
     )

    node5  = Action("fold",
        [Geometry2D.Point(230.0, 306.0)],
        [Geometry2D.Point(196.805, 306.845)],
        foldType='blue',
        foldLine = Geometry2D.DirectedLineSegment(\
           Geometry2D.Point(256.987, 115.746),
           Geometry2D.Point(262.387, 417.246)\
        )\
    )

    node6 = Action("fold",
        [Geometry2D.Point(220.0, 291.0), Geometry2D.Point(220.0, 416.0)],
        [Geometry2D.Point(186.574,291.08), Geometry2D.Point(187.174,416.079)],
        foldType = 'blue',
        foldLine = Geometry2D.DirectedLineSegment(\
           Geometry2D.Point(243.107457, 216.245950),\
           Geometry2D.Point(243.587457,416.24595)\
        )\
    )

    actions.append(node1)
    actions.append(node2)
    actions.append(node3)
    actions.append(node4)
    actions.append(node5)
    actions.append(node6)

    #self,polys,dragHistory,availableFolds,completedFolds,g = 0.0, h = 0, actionToHere="None",parent=None,depth=0,\
 #robotPosition = 'table_front"
    states = [FoldingSearch.SearchState(polys = [], dragHistory = [], availableFolds = [], completedFolds  = [], g=0.0, h=0.0,action = a) for a in actions]
    #self.robot.robotposition = "table_front_left"
    start = 0
    return states[start:]

def get_execute_BerkeleyProjectTee_actions_2():
    actions = []

    node1 = Action("move",
                   [],
                   [],
                   moveDestination = "table_left"
                   )

    node2 = Action("fold",
                   gripPoints = [Geometry2D.Point(173.000, 288.000),Geometry2D.Point(183.000,317.0000)],
                   endPoints = [Geometry2D.Point(232.512963,289.370602),Geometry2D.Point(221.188516,317.879493)],
                   foldType = "blue",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(200.192292,400.025087),
            Geometry2D.Point(205.892292,152.525087)
            )
                   )
    
    node3 = Action("fold",
                   gripPoints = [Geometry2D.Point(203.000, 280.000),Geometry2D.Point(203.000,400.000)],
                   endPoints = [Geometry2D.Point(248.838926,280.125015),Geometry2D.Point(248.184385,400.123230)],
                   foldType = "blue",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(225.592292, 400.025087),
            Geometry2D.Point(226.042292,235.025087)
            )
                   )

    node4 = Action("drag",
                   gripPoints = [Geometry2D.Point(225.000,400.000),
                                 Geometry2D.Point(225.0000, 280.0000)],
                   endPoints = [],
                   dragDirection = "-x",
                   dragDistance = 50
    )

    
    node5  = Action("fold",
                    gripPoints = [Geometry2D.Point(273.000,288.000),Geometry2D.Point(263.000,317.000)],        
                    endPoints = [Geometry2D.Point(213.856011,289.362104),Geometry2D.Point(225.180458,317.870996)],
                    foldType='blue',
                    foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(290.292292,152.525087),          
            Geometry2D.Point(295.992292,400.025087), # messed up, shouldnt matter
            )
                    )
    
    node6 = Action("fold",
                   gripPoints = [Geometry2D.Point(243.00, 280.00), Geometry2D.Point(243.00, 400.000)],
                   endPoints = [Geometry2D.Point(197.284585, 280.00), Geometry2D.Point(197.284585, 400.00)],
                   foldType = "blue",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(270.142292,235.025087),
            Geometry2D.Point(270.142292,400.025087)
            )
                   )
    actions.append(node1)
    actions.append(node2)
    actions.append(node3)
    actions.append(node4)
    actions.append(node5)
    actions.append(node6)

    #self,polys,dragHistory,availableFolds,completedFolds,g = 0.0, h = 0, actionToHere="None",parent=None,depth=0,\                                                                                                                       
 #robotPosition = 'table_front'):                                                                                                                                                                                                         
    
    states = [FoldingSearch.SearchState(polys = [], dragHistory = [], availableFolds = [], completedFolds  = [], action = a) for a in actions]
    #self.robot.robotposition = "table_front_left"                                                                                                                                                                                         
    start = 0
    return states[start:]

def get_execute_BerkeleyProjectTee_actions():
    actions = []

    node1 = Action("move",
                   [],
                   [],
                   moveDestination = "table_front_left"
                   )

    node2 = Action("drag",
                   gripPoints = [Geometry2D.Point(188.0000, 320.000),
                                 Geometry2D.Point(214.0000, 416.0000)],
                   endPoints = [],
                   dragDirection = "-x",
                   dragDistance = 70
     )

    node3 = Action("fold",
                   gripPoints = [Geometry2D.Point(118.000, 320.000)],
                   endPoints = [Geometry2D.Point(171.233753, 320.580732)],
                   foldType = "red",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(213.562118, 416.976589),
            Geometry2D.Point(216.262118,169.476589)
            )
                   )

    node4 = Action("fold",
                   gripPoints = [Geometry2D.Point(152.000, 316.00),Geometry2D.Point(152.000,416.000)],
                   endPoints = [Geometry2D.Point(173.584942, 316.070642), Geometry2D.Point(172.930404,416.0685)],
                   foldType = "blue",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(232.46, 416.976589),
            Geometry2D.Point(233.00,251.976589)
            )
                   )


    node5  = Action("fold",
                    gripPoints = [Geometry2D.Point(258.000,319.000)],
                    endPoints = [Geometry2D.Point(212.235648, 320.053967)],
                    foldType='blue',
                    foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(301.66,169.47),
            Geometry2D.Point(307.36,416.976589), # messed up, shouldnt matter                                                                                                           
            )
                    )

    node6 = Action("fold",
                   gripPoints = [Geometry2D.Point(234.00, 316.00), Geometry2D.Point(234.00, 416.000)],
                   endPoints = [Geometry2D.Point(198.26, 316.117), Geometry2D.Point(198.918, 416.114813)],
                   foldType = "blue",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(285.922118,251.976),
            Geometry2D.Point(286.46, 416.976589)
            )
                   )
    actions.append(node1)
    actions.append(node2)
    actions.append(node3)
    actions.append(node4)
    actions.append(node5)
    actions.append(node6)

    states = [FoldingSearch.SearchState(polys = [], dragHistory = [], availableFolds = [], completedFolds  = [], action = a) for a in actions]
    #self.robot.robotposition = "table_front_left"                                                                                                                                     \
                                                                                                                                                                                        
    start = 0
    return states[start:]



def get_execute_BlackWillowTee_actions():
    actions = []

    node1 = Action("move",
                   [],
                   [],
                   moveDestination = "table_front_left"
                   )

    node2 = Action("fold",
                   gripPoints = [Geometry2D.Point(175.00,309.000)],
                   endPoints = [Geometry2D.Point(231.2, 309.67)],
                   foldType = 'blue',
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(201.736610, 423.656176),
            Geometry2D.Point(204.96, 153.656)
            )
    )

    node3 = Action("fold",
                   gripPoints = [Geometry2D.Point(203.00, 309.00), Geometry2D.Point(203.00, 423.000)],
                   endPoints = [Geometry2D.Point(246.444365, 309.155676), Geometry2D.Point(245.627376, 423.152748)],
                   foldType = "blue",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(224.311610, 423.656176),
            Geometry2D.Point(224.956610, 243.656176)
            )
                   )

    node4 = Action("move",
                   [],
                   [],
                   moveDestination = "table_front_right"
                   )

    node5  = Action("fold",
                    gripPoints = [Geometry2D.Point(338.0, 309.0)],
                    endPoints = [Geometry2D.Point(283.528102, 310.261135)],
                    foldType='blue',
                    foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(307.1616, 154.03),
            Geometry2D.Point(307.1616, 154.03), # messed up, shouldnt matter                                                                                                                                        
            )
                    )

    node6 = Action("fold",
                   gripPoints = [Geometry2D.Point(311.00, 308.00), Geometry2D.Point(310.00, 423.000)],
                   endPoints = [Geometry2D.Point(265.794932, 308.161985), Geometry2D.Point(267.619, 423.151865)],
                   foldType = "blue",
                   foldLine = Geometry2D.DirectedLineSegment(
            Geometry2D.Point(288.166, 243.656176),
            Geometry2D.Point(288.811610, 423.656176)
            )
                   )
    actions.append(node1)
    actions.append(node2)
    actions.append(node3)
    actions.append(node4)
    actions.append(node5)
    actions.append(node6)

    states = [FoldingSearch.SearchState(polys = [], dragHistory = [], availableFolds = [], completedFolds  = [], action = a) for a in actions]                                                                                                                                                                                                                    
    start = 0
    return states[start:]

    

class FoldingMain():
    def __init__(self):
        util.listener = tf.TransformListener()
        rospy.sleep(3)
        self.robot = Robot.Robot()        
        #self.robot.robotposition = "table_front_left"
        self.gui = FoldingGUI(name="Geometry_Based_Folding")    
        self.mode = util.mode
        self.table_preset()
        self.poly_sub = rospy.Subscriber("input",PolyStamped,self.poly_handler)
        #self.scale_factor = self.x_offset = self.y_offset = self.poly_frame = False
        util.scale_factor = 5.0/0.0254
        self.poly_points = []
        now = time.localtime()
        self.logfile = open('/tmp/folding_%04d-%02d-%02d__%02d-%02d-%02d.log'%(now.tm_year,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec),'w')
        self.poly_sub = rospy.Subscriber("input",PolyStamped,self.poly_handler)
        self.start_time = rospy.Time.now()        
        rospy.loginfo("READY TO GO")
        #article_ind = -1
        #while not (article_ind > 0 and article_ind < 6):
        #    article_ind = raw_input('Enter # of article:\n\
        #        (1) hand towel\n\
        #        (2) big towel\n\
        #        (3) pants\n\
        #        (4) t-shirt\n\
        #        (5) long-sleeve shirt\n\
        #        ')
        #    article_ind = int(article_ind)
        #self.article_ind = article_ind
        #TODO Update towels to be different
        modes = ['towel', 'big_towel', 'pants', 'tee', 'shirt']
        self.mode = util.mode
        self.article_ind = modes.index(self.mode)
        self.makePolyFns = [self.gui.makeSmallTowel, self.gui.makeBigTowel, self.gui.makePants,\
        self.gui.makeShirt, self.gui.makeLongSleeveShirt];
        #self.mode = ['towel', 'towel', 'pants', 'tee', 'shirt'][self.article_ind-1]

    """
    # test version of poly handler
    def poly_handler(self,stamped_poly):
        if util.BUSY == True:
            return
        rospy.loginfo("RECEIVED A POINT")
        points = stamped_poly.vertices #[Geometry2D.Point(point.x,point.y) for point in stamped_poly.vertices]                                                                                        
        vertices = [util.convert_from_world_frame(point) for point in points]
        #vertices = [util.convert_from_world_frame(point) for point in points]
        poly = Geometry2D.Polygon(*vertices)
        self.poly_cache = poly
        cvPoly = CVPolygon(Colors.GREEN,self.gui.front(self.gui.shapes),poly)
        self.gui.clearShapes()
        self.gui.addCVShape(cvPoly)
        vertices = self.gui.getPolys()[0].getShape().vertices()
        print "#vertices = ",len(vertices)
        if len(vertices) >= 2:
            self.robot.arms_test(vertices[0],vertices[1])
            """
        
    
    #Receives a stream of polygon vertices and updates the poly appropriately                                             
    def poly_handler(self,stamped_poly):
        if util.BUSY == True:
            return

        rospy.loginfo("RECEIVED A POINT")        
        #self.robot.arms_test()
        points = stamped_poly.vertices #[Geometry2D.Point(point.x,point.y) for point in stamped_poly.vertices]                
        vertices = [util.convert_from_world_frame(point) for point in points]

        """
        for pt in vertices:
            print pt
        if SIM_FLAG:
            tbl  =Geometry2D.Point(151.979, 405.130237)
            tbf = Geometry2D.Point(224.808244, 488.925433)
            tbr = Geometry2D.Point(350.632802, 468.785788)
            self.table_detector([tbl,tbf,tbr])
            bl = Geometry2D.Point(200,470)
            poly = Geometry2D.Polygon(*self.makePolyFns[self.article_ind](bl)) #(*vertices)
            #poly = Geometry2D.Polygon(*self.gui.makePants(vertices[0]))
            self.poly_cache = poly
            cvPoly = CVPolygon(Colors.GREEN,self.gui.front(self.gui.shapes),poly)
            self.gui.clearShapes()        
            self.gui.addCVShape(cvPoly)
            self.handle_automatic_folds(self.gui.getPolys()[0].getShape().vertices())
            return
        # Edited for simulation

        # the first 6 define the table edge
        
        if TABLE_FLAG:
            #print vertices
            vertices = vertices[3:]
        else:
            self.table_detector(vertices)
            return
            
        if len(vertices) == 0:
            return
        """
        #self.robot.arms_test()
        #ppoly = Geometry2D.Polygon(*vertices[0])
        #poly = Geometry2D.Polygon(*self.gui.makeBerkeleyProjectTee(vertices[0])) 
        poly = Geometry2D.Polygon(*self.gui.makeBigTowel(vertices[0])) 
        #poly = Geometry2D.Polygon(*self.gui.makeBlackWillowTee(vertices[0]))
        #poly = Geometry2D.Polygon(*self.gui.makeSmallRedTowel(vertices[0]))
	#poly = Geometry2D.Polygon(*self.gui.makePants(vertices[0]))
        #poly = Geometry2D.Polygon(*self.gui.makeShirt(vertices[0]))
        self.poly_cache = poly
        cvPoly = CVPolygon(Colors.GREEN,self.gui.front(self.gui.shapes),poly)
        self.gui.clearShapes()        
        self.gui.addCVShape(cvPoly)
        self.handle_automatic_folds(self.gui.getPolys()[0].getShape().vertices())
        

    # waits for 6 points and sets table
    def table_detector(self,vertices):
        global TABLE_FLAG
        #print "table_detector, numvertices=",vertices
        if len(vertices) == 3:
            print "table vertices"
            for vertex in vertices:
                print vertex.x(),vertex.y()
            TABLE_FLAG = True        
            self.gui.createTable(vertices)
            self.gui.drawAllTables()
            return True
        return False


    def table_preset(self):        
        left = Geometry2D.Point3d(Geometry2D.Point(158.56737838,307.742449897),0,None)
        center = Geometry2D.Point3d(Geometry2D.Point(246.252446881,436.567891089),0,None)
        right = Geometry2D.Point3d(Geometry2D.Point(348.709236861,353.454649123),0,None)
        vertices = [left,center,right]
        self.gui.createTable(vertices)
        self.gui.drawAllTables()
        return True

    #Waits til it has received enough, then folds the article sketched                                                   
    def handle_automatic_folds(self,vertices):        
        if len(vertices) == 10 and self.mode == "shirt":
            self.start_logging()
            self.gui.foldShirt_v3()
            solution = FoldingSearch.FoldingSearch(self.gui,self.robot,self.gui.startpoly)
            self.robot.print_costs()
            self.stop_logging()
            self.stop_logging()
        elif len(vertices) == 10 and self.mode == "tee":
            if EXECUTE_FLAG:
                print "calling execute_tee_actions"
                actions = get_execute_BerkeleyProjectTee_actions_2()
                self.execute_actions(actions)
                sys.exit(0)
            self.start_logging()
            self.gui.foldTeeNoSleeve()
            solution = FoldingSearch.FoldingSearch(self.gui,self.robot,self.gui.startpoly)            
            self.robot.print_costs()            
            print "Brett:: Hit a key to make me fold!"
            raw_input()
            self.execute_actions(solution)
            #actions = get_execute_tee_actions()
            #self.execute_actions(actions)
            return
            self.start_logging()
            self.gui.foldTeeNoSleeve()
            
            solution = FoldingSearch.FoldingSearch(self.gui,self.robot,self.gui.startpoly)
            self.robot.print_costs()
            self.stop_logging()
        elif len(vertices) == 7 and self.mode == "pants":
            self.start_logging()
            self.gui.foldPants_v2()
            solution = FoldingSearch.FoldingSearch(self.gui,self.robot,self.gui.startpoly)
            self.robot.print_costs()
            print "Brett:: Hit a key to make me fold!"
            raw_input()
            self.stop_logging()
        elif len(vertices) == 4 and self.mode == "towel" or self.mode == "big_towel":
            util.BUSY = True
            #self.start_logging()
            """
            # arms test stuff            
            gripPts = vertices[0:2]
            endPts = vertices[2:]
            gripPts,endPts = self.gui.convertGripPts(gripPts,endPts)
            #gripPts = [self.robot.convert_to_robot_frame(util.convert_to_world_frame(gripPt),self.robot.robotposition) for gripPt in gripPts]
            #endPts = [self.robot.convert_to_robot_frame(util.convert_to_world_frame(endPt),self.robot.robotposition) for endPt in endPts]
            
            #vertices = [util.convert_to_world_frame(vertex) for vertex in vertices]                    
            #vertices = [(self.robot.convert_to_robot_frame(vertex,"table_front")) for vertex in vertices]                        
            #self.robot.arms_test(None,'l')
            self.robot.arms_test(endPts[1],endPts[0])
            return
            """
            # ---------------------------------
            self.gui.foldTowelThirds()            
            solution = FoldingSearch.FoldingSearch(self.gui,self.robot,self.gui.startpoly)
            self.robot.print_costs()
            print "Brett:: Hit a key to make me fold!"
            raw_input()
            self.execute_actions(solution)
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
        if not EXECUTE_FLAG or self.mode != 'tee':
            states=states[1:]
        for state in states:

            action = state.action
            print "\n\n\n\n\naction is ",action

            # transform points to current frame of robot
            gripPts3d, endPts3d = self.gui.convertGripPts(action.get_gripPoints(), action.get_endPoints())
            if action.get_actionType() in ("drag"):
                gripPts3d = [self.robot.convert_to_robot_frame(util.convert_to_world_frame(gripPt),self.robot.robotposition) for gripPt in gripPts3d]
                d = action.get_dragDistance()/util.scale_factor
            if action.get_actionType() == "fold":
                #endPts = [self.robot.convert_to_robot_frame(util.convert_to_world_frame(endPt),self.robot.robotposition) for endPt in endPts3d]
                color_current = action.get_foldType()
                # find type of next action
            if (i < len(states) and states[i].action.get_actionType()=="fold"):                
                color_next = states[i].action.get_foldType()
                gripPts_next = states[i].action.get_gripPoints()
                endPts_next = states[i].action.get_endPoints()
                gripPts_next,endPts_next = self.gui.convertGripPts(gripPts_next,endPts_next)
            else:
                color_next = "blue"
                gripPts_next = None
                endPts_next = None
    
            print "color_next",color_next

            if action.get_actionType() == "fold":
                SUCCESS = self.robot.execute_fold(gripPts3d,endPts3d,color_current,color_next)
            elif action.get_actionType() == "drag":
                SUCCESS = self.robot.execute_drag(gripPts3d,d,action.get_dragDirection(),color_next, gripPts_next, endPts_next)
            elif action.get_actionType() == "move":
                SUCCESS = self.robot.execute_move(action.get_moveDestination())

            if not SUCCESS:
                rospy.loginfo("Failure to execute %s",action.get_actionType())
            raw_input(" hit a key for next ACTION")
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


def sigint_handler(signal, frame):
    profiler.dump_stats('foldprof')

def main(args):
    rospy.init_node("folding_main")
    rospy.sleep(3)    
    FM = FoldingMain()
    raw_input("Folding Main Called")
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        #profiler = cProfile.Profile()
        #signal.signal(signal.SIGINT, sigint_handler)
        #cProfile.run('main(args)', 'Profileprof')
        main(args)
    except rospy.ROSInterruptException: pass

'''
class Action():
    def __init__(self,actionType, gripPoints, endPoints,moveDestination = "None", dragDirection = 'None', dragDistance = 'None', foldType = 'None', foldLine = 'N\
one'):
        self.actionType = actionType
        self.gripPoints = gripPoints
        self.endPoints = endPoints
        self.moveDestination = moveDestination
        self.dragDirection = dragDirection
        self.dragDistance = dragDistance
        self.foldType = foldType
        self.foldLine = foldLine
'''

                                                                         

        

                                                                                                                                                                                     
