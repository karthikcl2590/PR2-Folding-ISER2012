#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_geometry")
from shape_window.ShapeWindow import *
from shape_window import ShapeWindowUtils
from shape_window import Geometry2D
import sys
import time
import math
import rospy
from numpy import *

#   The FoldingGUI is a ShapeWindow which, in addition to drawing shapes, allows you to fold them. The two types of folds you can execute are "Red Folds" and "Blue Folds".
# A blue fold is the intuitive one. You draw a line segment which bisects the polygon into two parts. The part to the left of the line segment (relative to it's direction) is mirrored about the segment, and the
# portion is placed on top.
# A red fold is identical to a blue fold, except it must be executed on the last-folded portion of cloth. This is an exception to the general rule that the robot may not grasp a particular layer of the cloth. As the
# portion has just been folded, it is still in the robot's grippers, and may be grasped even if there is a layer underneath,.

FLAG_sim = False
robotPositions = {"+x":(430,250),"-x":(50,250),"+y":(250,550)}
tableCorners = {"bl":(150,450),"tl":(150,50),"br":(350,450),"tr":(350,50)}
class FoldingGUI(ShapeWindow):
    
    def initExtended(self):
        self.robotPosition = '+y'
        self.currentSearchState = None
        self.table_edge = Geometry2D.LineSegment(Geometry2D.Point(0,450),Geometry2D.Point(1500,450))                
        self.max_reach_y = 200
        self.fold_callback = False
        self.unfreezeMouse()
        self.setGripSize(10)
        self.simFlag = False
        self.wideGripFlag = False
        self.drawingModeFlag = False
        self.numGrippers = 2
        self.blueStart = Geometry2D.Point(0,0)
        self.blueEnd = Geometry2D.Point(0,0)
        self.redStart = Geometry2D.Point(0,0)
        self.redEnd = Geometry2D.Point(0,0)
        self.midStart = Geometry2D.Point(0,0)
        self.midEnd = Geometry2D.Point(0,0)
        self.mode = "towel"
        self.lastFolded = []
        self.lastFoldline = []
        self.lastState = []
        self.setGripperLimit(False)
        self.addQueue = []
        self.removeQueue = []
        self.setSnapRange(15)
        self.initOverlay()
        self.gravityRobustness = 0
        self.dragStart = Geometry2D.Point(0,0)
        self.dragEnd =  Geometry2D.Point(0,0)
        self.dragDistance = 0
        self.dragDirection = ''
        self.objectDefined = False
        self.foldTree = []        
        self.readytoFold = False
        self.startpoly = None
        thread.start_new_thread(self.check_definition,())
        
    def check_definition(self):
        while(not self.isClosed()):
            if self.objectDefined:
                try:
                    self.handle_folds(self.getPolys()[0].getShape().vertices())
                except AttributeError: 
                    print sys.exc_info()[0] ,sys.exc_value
                self.objectDefined = False
                time.sleep(0.01)

    def initOverlay(self):
        gripSlider = CVSlider(origin=Geometry2D.Point(150,950),valMin=0,valMax=200,valueGetter=self.getGripSize,valueSetter=self.setGripSize,sliderWidth=150)
        self.addOverlay(gripSlider)
        wideGripButton = CVOnOffButton(text="WG?",bottomLeft=Geometry2D.Point(50,950),valueGetter=self.wideGrip,valueToggle=self.toggleWideGrip)
        self.addOverlay(wideGripButton)
        drawPolyButton = CVOnOffButton(text="Draw",bottomLeft=Geometry2D.Point(400,950),valueGetter=self.drawingMode,valueToggle=self.toggleDrawingMode)
        self.addOverlay(drawPolyButton)
        clearShapesButton = CVButton(text="CLEAR",bottomLeft=Geometry2D.Point(50,900), onClick=self.clearShapes)
        self.addOverlay(clearShapesButton)
        curLabel = CVLabel(text="Current Shape", bottomLeft=Geometry2D.Point(150,50))
        self.addOverlay(curLabel)
        gripVisualizer = CVVisualizer(origin=Geometry2D.Point(450,50),valueGetter=self.getGripSize,color=Colors.BLUE,displayCondition = self.wideGrip)
        self.addOverlay(gripVisualizer)        
        start = Geometry2D.Point(0,self.max_reach_y)
        end   = Geometry2D.Point(1500,self.max_reach_y)
        """tr1line = CVLineSegment(color=Colors.GREY,height=100,shape=Geometry2D.LineSegment(start,end))
        self.addOverlay(tr1line)        
        table_edge_line = CVLineSegment(color=Colors.GREY,height=100,shape=self.table_edge)
        self.addOverlay(table_edge_line) """
        self.drawTable(0)
        self.drawTable(500)
        self.drawTable(1000)
        startSec1 = Geometry2D.Point(500,0)
        endSec1 = Geometry2D.Point(500,1000)
        sec1 = CVLineSegment(color=Colors.YELLOW, height = 100, shape=Geometry2D.LineSegment(startSec1, endSec1))
        self.addOverlay(sec1)
        startSec2 = Geometry2D.Point(1000,0)
        endSec2 = Geometry2D.Point(1000,1000)
        sec2 = CVLineSegment(color=Colors.YELLOW, height = 100, shape=Geometry2D.LineSegment(startSec2, endSec2))
        self.addOverlay(sec2)
        altLabel = CVLabel(text="Available Actions", bottomLeft=Geometry2D.Point(650,50))
        self.addOverlay(altLabel)
        nextLabel = CVLabel(text="Next Shape", bottomLeft=Geometry2D.Point(1170,50))
        self.addOverlay(nextLabel)
        self.drawRobot(self.robotPosition)


    def drawRobot(self,robotPosition, offset = 0):        
        coords = robotPositions[robotPosition]
        #print coords
        #self.robotPosition = robotPosition
        if robotPosition == "+y":  
            vertices = [Geometry2D.Point(coords[0]-20,coords[1]+40),Geometry2D.Point(coords[0],coords[1]),Geometry2D.Point(coords[0]+20,coords[1]+40)]        
            reachable_y = Geometry2D.LineSegment(Geometry2D.Point(coords[0]-100,coords[1]-self.max_reach_y),Geometry2D.Point(coords[0]+100,coords[1]-self.max_reach_y))            
        elif robotPosition == "+x":
            vertices = [Geometry2D.Point(coords[0]+40,coords[1]+20),Geometry2D.Point(coords[0],coords[1]),Geometry2D.Point(coords[0]+40,coords[1]-20)]
            reachable_y = Geometry2D.LineSegment(Geometry2D.Point(coords[0]-self.max_reach_y,coords[1]+100),Geometry2D.Point(coords[0]-self.max_reach_y,coords[1]-100))
        elif robotPosition == "-x":
            vertices = [Geometry2D.Point(coords[0]-40,coords[1]+20),Geometry2D.Point(coords[0],coords[1]),Geometry2D.Point(coords[0]-40,coords[1]-20)]
            reachable_y = Geometry2D.LineSegment(Geometry2D.Point(coords[0]+self.max_reach_y,coords[1]+100),Geometry2D.Point(coords[0]+self.max_reach_y,coords[1]-100))

        robot = Geometry2D.Polygon(vertices[0],vertices[1],vertices[2])
        if(offset == 0):
            self.addTempCVShape(CVPolygon(color = Colors.YELLOW,shape=robot,height=100))
            self.addPropCVShape(CVPolygon(color  = Colors.YELLOW, shape=robot, height=100))
            self.addTempCVShape(CVLineSegment(color = Colors.GREEN,shape = reachable_y,height=100))
            self.addPropCVShape(CVLineSegment(color = Colors.GREEN,shape = reachable_y,height=100))
        else:
            self.addSelectedCVShape(CVPolygon(color = Colors.YELLOW, shape = robot, height = 100))
            self.addSelectedCVShape(CVLineSegment(color = Colors.GREEN,shape = reachable_y,height= 100))
        
    #    self.addOverlay(CVPolygon(color = Colors.YELLOW,shape=robot,height=100))  
       
    def mouseFreeze(self):
        return self.mouse_frozen
    
    def freezeMouse(self):
        self.mouse_frozen = True
    
    def unfreezeMouse(self):
        self.mouse_frozen = False
    
    def onMouse(self,event,x,y,flags,param):        
        if self.mouseFreeze():
            return
        if self.drawingMode():
            return self.polyDraw(event,x,y,flags,param)
        else:
            return self.foldDrawer(event,x,y,flags,param)
            
    def snapPoints(self):
        snap_points = []
        for cv_poly in self.getPolys():
            for side in cv_poly.getShape().sides():
                snap_points.append(side.start())
                snap_points.append(side.center())
                snap_points.append(side.end())
        return snap_points
        
    def shapeListeners(self):
        return False

    def isFoldOutsideTable(self, foldline):
        directions = ['-x' , '+x' , '+y']
        axisX = Geometry2D.LineSegment(Geometry2D.Point(0.0,0), Geometry2D.Point(1.0,0))
        axisY = Geometry2D.LineSegment(Geometry2D.Point(0.0, 0), Geometry2D.Point(0,1.0))
        angle = math.fabs(Geometry2D.angleBetweenLines(foldline, axisX))
        print "foldline", foldline,"angle", angle
        #raw_input()
        if((angle == nan) or ((angle > 1.50) and (angle < 1.70))):
            directions = ['+x', '-x']
        else:
            directions = ['+y']
        for direc in directions:
            tableEdge = self.getClosestTableEdge(direc)
            if direc == '-x':
                if tableEdge.isRightOf(foldline.start()) and tableEdge.isRightOf(foldline.end()):
                    return (True, direc)
            if direc == '+y':
                if tableEdge.isRightOf(foldline.start()) and tableEdge.isRightOf(foldline.end()):
                    return (True,direc)
            if direc == "+x":
                if tableEdge.isLeftOf(foldline.start()) and tableEdge.isLeftOf(foldline.end()):
                    return (True,direc)
        if('+y' in directions):
            return (False,'+y')
        else:
            tableEdgeXplus = self.getClosestTableEdge('+x')
            tableEdgeXminus = self.getClosestTableEdge('-x')
            print "Distance off", math.fabs(tableEdgeXplus.start().x() - (foldline.start().x())), math.fabs(tableEdgeXminus.start().x() - foldline.start().x())

            if(math.fabs(tableEdgeXplus.start().x() - (foldline.start().x())) >= math.fabs(tableEdgeXminus.start().x() - foldline.start().x())):
                return (False,'-x')
            else:
                return (False,'+x')

    def getClosestTableEdge(self,direction = '-x'):
        if direction == '-x':
            start = tableCorners["bl"]
            end = tableCorners["tl"]
        elif direction == '+x':
            start = tableCorners["tr"]
            end = tableCorners["br"]
        elif direction  =='+y':
            start = tableCorners["br"]
            end = tableCorners["bl"]
        return Geometry2D.DirectedLineSegment(Geometry2D.Point(start[0], start[1]), Geometry2D.Point(end[0], end[1]))
    
    def getOppositeTableEdge(self,robotPosition):
        """
        Given a robot position, returns the table edge opposite to the robot (line seg from right to left from robot's perspective)
        """        
        if robotPosition == "-x":
            start = tableCorners["br"]
            end = tableCorners["tr"]
        elif robotPosition == "+x":
            start = tableCorners["tl"]
            end = tableCorners["bl"]
        elif robotPosition == "+y":
            start = tableCorners["tr"]
            end = tableCorners["tl"]                
        return Geometry2D.DirectedLineSegment(Geometry2D.Point(start[0],start[1]),Geometry2D.Point(end[0],end[1]))

    def getHighlighted(self,x,y):
        mousedOver = ShapeWindow.getHighlighted(self,x,y)
        if len(mousedOver) == 0:
            return []
        else:
            return [cvShape for cvShape in mousedOver if cvShape.getHeight() == max(mousedOver,key = lambda x: x.getHeight()).getHeight()]
        
    def adjust_foldline(self):
        if not self.fold_callback:
            return False
        else:
            return True


    def polyDraw(self,event,x,y,flags,param):
        vert = self.polyDrawer(event,x,y,flags,param)
        if event == cv.CV_EVENT_RBUTTONUP:
            self.objectDefined = True
#self.handle_folds(self.getPolys()[0].getShape().vertices())

    def foldDrawer(self,event,x,y,flags,param):
        (newx,newy) = self.snapToPoint(x,y)
        pt = Geometry2D.Point(newx,newy)        
        self.highlightPt(pt)
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.blueStart = pt
        elif event == cv.CV_EVENT_RBUTTONDOWN:
            self.redStart = pt
        elif event == cv.CV_EVENT_LBUTTONUP:
            self.blueEnd = pt
            self.executeBlueFold()        
        elif event == cv.CV_EVENT_RBUTTONUP:
            self.redEnd = pt
            self.executeRedFold()
        elif cv.CV_EVENT_FLAG_LBUTTON == flags-32:
            line = Geometry2D.DirectedLineSegment(self.blueStart,pt)
            self.addTempCVShape(CVDirectedLineSegment(cv.RGB(175,175,255),10,line,2))
        elif cv.CV_EVENT_FLAG_RBUTTON == flags-32:
            line = Geometry2D.DirectedLineSegment(self.redStart,pt)
            self.addTempCVShape(CVDirectedLineSegment(cv.RGB(255,175,175),10,line,2))
        elif event == cv.CV_EVENT_MBUTTONDOWN:
            self.midStart = pt
        elif event == cv.CV_EVENT_MBUTTONUP:
            self.midEnd = pt
            dx = self.midEnd.x() - self.midStart.x()
            dy = self.midEnd.y() - self.midStart.y()
            for shape in [cvshape.getShape() for cvshape in self.getShapes()]:
                shape.translate(dx,dy)
    
    def executeDragFold(self):
        self.lastState = []
        for poly in self.getPolys():
            print "Poly is %s"%(poly)
            self.lastState.append(poly.dupl())
        foldline = Geometry2D.DirectedLineSegment(self.dragStart,self.dragEnd)
        self.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,0,255),self.front(),foldline))
        print "Is bluefoldLegal", self.legalBlueFold(foldline)
        if self.legalBlueFold(foldline):
            (activeVerts,gripPts,Endpts) = self.foldAll(self.getPolys(),foldline,True)
            #add can_folf->try_drag sequence
            for v in activeVerts:
                self.highlightPt(v)
            if self.wideGrip():
                for g in gripPts:
                    self.drawGripper(g)
            print "Num grippers required: %d"%len(gripPts)
            if not self.adjust_foldline():
                return
            adjusted_foldline = self.echoFold(foldline,gripPts,False,'hang')
            #adjusted_foldline = foldline
            print "INITIAL FOLDLINE WAS: %s"%foldline
            print "RECEIVED ADJUSTED FOLDLINE: %s"%adjusted_foldline
            if self.legalBlueFold(adjusted_foldline):
                self.clearShapes()
                for poly in self.lastState:
                    self.addCVShape(poly)
                #Now redo the folding but don't echo it                                                                                    
                self.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,255,255),self.front(),foldline))
                (activeVerts,gripPts,endPts) = self.foldAll(self.getPolys(),adjusted_foldline,True)
                #If something invalid happens, ignore it                                                                                   
                if len(gripPts)==0:
                    self.clearShapes()
                    for poly in self.lastState:
                        self.addCVShape(poly)
                        (activeVerts,gripPts,endPts) = self.foldAll(self.getPolys(),foldline)
            else:
                print "RECEIVED ILLEGAL ADJUSTED FOLDLINE"
        else:
            print "Illegal fold"
        #raw_input("hit any key to continue")


    def executeBlueFold(self):
        self.lastState = []
        rospy.loginfo("\n\n NEW FOLD \n");
        for poly in self.getPolys():
            #rospy.loginfo("Poly %s",poly)
            rospy.loginfo("shape %s",poly.shape)
            #rospy.loginfo("sides %s",poly.shape.sides())
            self.lastState.append(poly.dupl())                                        
        #for poly in self.proposed:
            #print "Proposed poly is",poly
        foldline = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
        print "Their Blue fold line", foldline
        self.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,0,255),self.front(),foldline))
        if self.legalBlueFold(foldline):
            (activeVerts,gripPts,endPts) = self.foldAll(self.getPolys(),foldline,False)
            print "num_polys before call to canfold",len(self.lastState)
            (canFold,unreachablePts) = self.can_fold(gripPts,endPts)            
            if not canFold: # cannot fold as is because of reachability constraints
                print "cannot fold! checking drag"
                canDrag = self.try_drag(self.lastState,gripPts,endPts,unreachablePts)
                if not canDrag: 
                    print "Cannot fold or drag. Shoot me"
                    return
                else:
                    print "can drag! executing drag fold"
                    self.clearShapes()
                    for poly in self.lastState:
                        self.addCVShape(poly)
                    self.executeDragFold()
            
            for v in activeVerts:
                self.highlightPt(v)
            if self.wideGrip():
                for g in gripPts:
                    self.drawGripper(g)
            print "Num grippers required: %d"%len(gripPts)
            if not self.adjust_foldline():
                return
            adjusted_foldline = self.echoFold(foldline,gripPts,False,'nothang')
            adjusted_foldline = foldline
            print "INITIAL FOLDLINE WAS: %s"%foldline
            print "RECEIVED ADJUSTED FOLDLINE: %s"%adjusted_foldline
            if self.legalBlueFold(adjusted_foldline):
                self.clearShapes()
                for poly in self.lastState:
                    self.addCVShape(poly)
                #Now redo the folding but don't echo it
                self.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,255,255),self.front(),foldline))
                (activeVerts,gripPts,endPts) = self.foldAll(self.getPolys(),adjusted_foldline,False)
                #If something invalid happens, ignore it
                if len(gripPts)==0:
                    self.clearShapes()
                    for poly in self.lastState:
                        self.addCVShape(poly)
                        (activeVerts,gripPts) = self.foldAll(self.getPolys(),foldline)
            else:
                print "RECEIVED ILLEGAL ADJUSTED FOLDLINE"
        else:
            print "Illegal fold"
            
        #raw_input("hit any key to continue")
            
    def legalDragFold(self,foldline,polys,checkHang = False):
        return True
        hangingPolys = [cvshape for cvshape in polys if cvshape.isHang()]
        if len(hangingPolys) > 0:
            print "hanging > 0"
            for hangingPoly in hangingPolys:
                bisected = Geometry2D.bisectPoly(hangingPoly, foldline)
                #if not False in bisected:         
                """
                dragAxis = Geometry2D.LineSegment(Geometry2D.Point(0.0,0), Geometry2D.Point(1.0,0))
            elif self.dragDirection =='-y' or self.dragDirection == '+y':
                dragAxis = Geometry2D.LineSegment(Geometry2D.Point(0.0, 0), Geometry2D.Point(0,1.0)) 
           angle = math.fabs(Geometry2D.angleBetweenLines(foldline, dragAxis)) "
            if(angle == 0 or angle == 180):
            
                    #if self.isFoldOutsideTable(foldline):
                     #   return False
               
                if(foldline.start().
                if (foldline.start().x() > self.dragStart.x()) and (foldline.end().x() > self.dragStart.x()):
                        return False
                elif(direction == '+x'):
                    if (foldline.start().x() < self.dragStart.x()) and (foldline.end().x() < self.dragStart.x()):
                        return False
                elif(direction == '-y'):
                    if(foldline.start().y() < self.dragStart.y()) and (foldline.end().y() < self.dragStart.y()):
                        return False
                elif(direction == '+y'):
                    if(foldline.start().y() > self.dragStart.y()) and (foldline.end().y() > self.dragStart.y()):
                        return False
            return True
            """
        if checkHang and self.isAnyPolyWithinHang(foldline,polys):
            print "Poly within Fold Line"
            return False
        return True
        
    def legalBlueFold(self,foldline,polys):
        if not self.legalDragFold(foldline,polys):
            print "Drag Fold Illegal"
            return False
        print "Legal Blue Fold Check"
        for shape in [cvshape.getShape() for cvshape in polys]:
            if shape.containsExclusive(foldline.start()) or shape.containsExclusive(foldline.end()):
                print "Illegal shape is",shape
                #raw_input()
                return False 
        return True
                   
    def executeRedFold(self):        
        foldline = Geometry2D.DirectedLineSegment(self.redStart,self.redEnd)
        self.addTempCVShape(CVDirectedLineSegment(cv.RGB(255,0,0),self.front(),foldline))
        if self.legalRedFold(foldline):
            (activeVerts,gripPts) = self.foldAll(self.lastFolded,foldline)
            for v in activeVerts:
                self.highlightPt(v)
            if self.wideGrip():
                for g in gripPts:
                    self.drawGripper(g)
            print "Num grippers required: %d"%len(gripPts)
            self.echoFold(foldline,gripPts,True)
        else:
            print "Illegal fold"
            
    def legalRedFold(self,foldline):
        if self.currentSearchState == None:
            lastState = self
        else:
            lastState = self.currentSearchState
        if len(lastState.lastFolded) == 0:
            return False            
        for line in lastState.lastFoldline:
            if line.start() and line.end() and foldline.isRightOf(line.start()) or foldline.isRightOf(line.end()):
                print "Line is right of the last foldline"
                return False
        for shape in [cvshape.getShape() for cvshape in lastState.lastFolded]:
            if shape.containsExclusive(foldline.start()) or shape.containsExclusive(foldline.end()):
                return False
        return True
    

    def foldAll(self,polys,foldline,isHang,SearchNode = None,d = 0, direction = "+y"):        
        self.flushQueue()
        print "Current polys in Fold"
        for poly in polys:
            print poly
        print "End current polys"
        [toFold,toNotFold] = self.getFoldedRegion(polys,foldline)
        print "End of new Polys"
        print "Poly to be folded"
        for poly in toFold:
            print poly
        #raw_input()    
        
#        height = (mergeAdjacentFold(toFold))
        print "poly not to be folded"
        for poly in toNotFold:
            print poly
        print "POLY ENDED"
        self.toFold = toFold
        self.toNotFold = toNotFold
        self.lastFolded = []
        self.lastFoldline = []
        activeVerts = []
        activeEndPts = []
        (outside, direc) = self.isFoldOutsideTable(foldline)
        polyHanging = self.isAnyPolyHangingInDirection(polys,direc)
        mirrorAxis = Geometry2D.DirectedLineSegment(Geometry2D.mirrorPt(self.getClosestTableEdge(direc).start(), foldline), Geometry2D.mirrorPt(self.getClosestTableEdge(direc).end(), foldline))
        print "Poly Hanging", polyHanging, "Direc" , direc, foldline
        for poly in sorted(polys,key=lambda p: p.getHeight(), reverse=True):
            if isHang:
                print "toFold:I am here",
                for poly in toFold:
                    print poly
                #print "isHang so do drag Fold"
                
                newActive = self.drag(poly,foldline,toFold,direction,d, SearchNode, pHang = polyHanging)
                newEnd = []
            else:

                (newActive,newEnd) = self.fold(poly,foldline,toFold,SearchNode, pHang = polyHanging)                            
            if newActive==False and isHang:
                self.flushQueue()
                print "Invalid Drag fold executed"
                raw_input()
                return (activeVerts, [], activeEndPts)
            #if newActive == [] and newEnd == [] and not isHang:
             #   self.flushQueue()
              #  print "Returning because of empty queues"
               # return (activeVerts,[],activeEndPts)
            for v in newActive:
                if not v in activeVerts:
                    activeVerts.append(v)
            for v in newEnd:
                if not v in activeEndPts:
                    activeEndPts.append(v)
        gripPoints = self.gripPoints(activeVerts)
        if not self.gripperLimit or len(gripPoints) <= self.gripperLimit:
            print "length of queue", self.addQueue
            if(self.allPolyHang(list(self.addQueue))):
                self.flushQueue()
                gripPoints = []
                print "All polys hanging"
            else:
                self.executeQueue(SearchNode, mirrorAxis, direc)
        else:
            self.flushQueue()
            print "Error: requires %d grippers"%len(gripPoints)
            #raw_input()
            gripPoints = []
        return (activeVerts, gripPoints,activeEndPts)
            
    
    def allPolyHang(self, polys):
        for poly in polys:
            if not poly.isHang():
                return False
        return True

    def can_fold(self,gripPts,endPts,robotPosition):
        """
        determines if robot can reach all points required to complete the fold
        """        
        can_reach_all = True
        unreachable = []
        #for pt in endPts:
            #print "endPoint",pt
        for point in gripPts + endPts:            
            if(not self.can_reach(point,robotPosition)):
                can_reach_all = False
                unreachable.append(point)
        return (can_reach_all,unreachable)



    def isFoldPerpendicularToHang(self,poly, foldline):
        if not poly.isHang():
            return False
        else:
            print poly
            hangDirec = poly.getHangDirection()
            hangLine = self.getClosestTableEdge(hangDirec)
            angle = math.fabs(Geometry2D.angleBetweenLines(foldline, hangLine))
            print "hangline", hangLine, "foldline", foldline, "angle", angle, math.isnan
            if(math.isnan(angle) or angle > 1.50):
                print "return True"
                return True
            else:
                bisected = Geometry2D.bisectPoly(poly.getShape(), foldline)
                if not (False in bisected):
                    return False
                else:
                    return True
        return False

    def isAnyPolyHangingInDirection(self,polys,direction):
        for poly in polys:
            if poly.isHang() and poly.getHangDirection() == direction:
                return True
        return False
                

    def fold(self, poly, foldline, toFold,SearchNode = None, pHang = False):        
        active = []
        endPts = []
        height = poly.getHeight()
        color = poly.getColor()
        shape = poly.getShape()
        drawh= self.front()
        halves = Geometry2D.bisectPoly(shape,foldline)
        (outside, direc) = self.isFoldOutsideTable(foldline)
        tableEdge = self.getClosestTableEdge(direc)
        #print "poly",poly,"halves",halves[0],halves[1]
        #if False in halves:
            #print "Foldline has false half", foldline
            #for p in halves:
            #    if p!= False:
            #        print p
        inters = [x for x in [Geometry2D.intersect(side,foldline) for side in shape.sides()] if x]        
        if len(inters) >= 2:
            lines = [Geometry2D.LineSegment(x,y) for x in inters for y in inters if x != y]
            ln = max(lines,key=lambda x: x.length())
            if not ln in self.lastFoldline:
                self.lastFoldline.append(ln)
        [a,b,c] = foldline.standardForm()
        for p in halves:
            if p != False:
                if len([folded for folded in toFold if folded.containsExclusive( p.randPt())]) > 0:            
                #if len([x for x in toFold if x.contains(p.randPt())]) > 0:
                #if foldline.isRightOf(p.center()):
                    print poly,  self.isFoldPerpendicularToHang(poly, foldline)
                    #raw_input("Current Poly")
                    for pt in p.vertices():
                        print "checking pt",pt
                        if self.isActive(pt,poly,foldline, SearchNode.get_polys()) and not self.isFoldPerpendicularToHang(poly, foldline):
                            if(pHang and poly.isHang()) or not pHang:
                                active.append(pt)
                                print "Point added",pt
                            # find endpts of the fold
                                endPts.append(Geometry2D.mirrorPt(pt,foldline))
                            
                    drawp = Geometry2D.mirrorPoly(p,foldline)
                    #drawh = self.front()
                    if poly.isHang() and not self.isFoldPerpendicularToHang(poly,foldline):    #check if mirroredPoly is still hanging
                        if(self.isPolyHanging(drawp, poly.getHangDirection())):
                            print "Illegal Fold, poly in Hang", foldline, poly
                            raw_input()
                            return ([],[])
                        else:
                            bisected = Geometry2D.bisectPoly(drawp, self.getClosestTableEdge(poly.getHangDirection()))
                    #if not poly.isHang():
                     #   bisected = []
                    #if not self.legalDragFold(tableEdge, SearchNode.get_polys(), True):                                                                                           
                        #   return False                                                                                                                                              
                            for b in bisected:
                                print b
                                if b ==False:
                                    continue
                                else:
                                   # raw_input("Unhanging the Poly")
                                    if not self.isPolyHanging(b, direc):
                                        drawc = Colors.complementCV(Colors.darkenCV(color,0.02))
                                        cvpoly = CVPolygon(drawc,drawh,b)
                                        cvpoly.setHang(False)
                                        cvpoly.setPrevHang(True)
                                        self.lastFolded.append(cvpoly)
                                        self.queueAddShape(cvpoly)
                                        print cvpoly
                                    #    raw_input("Here is the unhung Poly")
                                    else:
                                        drawp = b
                                        drawc = Colors.complementCV(color)
                                        cvpoly = CVPolygon(drawc,drawh,drawp)
                                        cvpoly.setHang(poly.isHang(), poly.getHangDirection())
                                        self.queueAddShape(cvpoly)
                                        self.lastFolded.append(cvpoly)
                                        if SearchNode != None:
                                            SearchNode.lastFolded.append(cvpoly)
                    else:
                        drawc = Colors.complementCV(color)
                        cvpoly = CVPolygon(drawc,drawh,drawp)
                        cvpoly.setHang(poly.isHang(), poly.getHangDirection())
                        self.queueAddShape(cvpoly)
                        self.lastFolded.append(cvpoly)
                        if SearchNode !=None:
                            SearchNode.lastFolded.append(cvpoly)
                        
                            
                else:
                    drawp = p
                    drawh = height
                    drawc = color
                    cvpoly = CVPolygon(drawc,drawh,drawp)
                    cvpoly.setHang(poly.isHang(), poly.getHangDirection())
                    self.queueAddShape(cvpoly)
                #print "CVPOLY added"
        self.queueRemoveShape(poly)
        return (active,endPts)

    def can_reach(self,point,robotPosition):
        """
        hardcoded - see if robot can reach this point
        """
        (x,y) = (point.x(),point.y())
        (robot_x,robot_y) = robotPositions[robotPosition]
        robot_max_reach = self.max_reach_y
        can_reach = True
        if(robotPosition == '+y'):
            if (y <= robot_y - robot_max_reach):
                can_reach = False
        elif(robotPosition == '+x'):
            if (x <= robot_x - robot_max_reach):
                can_reach = False
        elif(robotPosition == '-x'):
            if(x >= robot_x + robot_max_reach):
                can_reach = False
        
        return can_reach

    def try_drag(self,polys,gripPts,endPts,unreachablePts,robotPosition):
        """                                                                                                                                                         
        Determine direction and extent of drag needed to perform the fold.                                                                                 
        If dragging is not legal, return False
        If dragging is legal, return True
        """
        # robot can only drag towards itself
        direction = robotPosition
        
        print "in try_drag, num polys = ",len(polys)
        if not self.can_drag(polys,gripPts,direction):
            return False

        min_d_to_drag = 0
        for pt in unreachablePts:
            (x,y) = (pt.x(),pt.y())
            d = 10
            temp_d = 10
            while(not self.can_reach(Geometry2D.Point(pt.x(),pt.y()+temp_d))):
                temp_d += d
            if temp_d > min_d_to_drag:
                min_d_to_drag = temp_d                
                # since table edge is the cloth edge, hangline is min_d_to_drag up from the table edge
                [table_start,table_end] = self.table_edge.pts()
                self.dragEnd = Geometry2D.Point(table_start.x(), table_start.y() - min_d_to_drag)
                self.dragStart = Geometry2D.Point(table_end.x(), table_end.y() - min_d_to_drag)
                self.dragDistance = min_d_to_drag
                self.dragDirection = direction                                
                
        return True

    def drag(self, poly, foldline, toFold,direction, distance, SearchNode = None, pHang = False):
        active = []
        height = poly.getHeight()
        color = poly.getColor()
        shape = poly.getShape()
        halves = Geometry2D.bisectPoly(shape,foldline)
        inters = [x for x in [Geometry2D.intersect(side,foldline) for side in shape.sides()] if x]
        if len(inters) >= 2:
            lines = [Geometry2D.LineSegment(x,y) for x in inters for y in inters if x != y]
            ln = max(lines,key=lambda x: x.length())
            if not ln in self.lastFoldline:
                self.lastFoldline.append(ln)
        [a,b,c] = foldline.standardForm()
        for p in halves:
            #print "Polygon in fold",p,foldline,toFold                                                                                          
            #print "randpt",p.randPt()                                                                                                          

            if p != False:
                if len([folded for folded in toFold if folded.containsExclusive(p.randPt())]) > 0:
                #if len([x for x in toFold if x.contains(p.randPt())]) > 0:
                #if foldline.isRightOf(p.center()):
                    for pt in p.vertices():
                        if self.isActive(pt,poly,foldline,SearchNode.get_polys()) and not self.isFoldPerpendicularToHang(poly, foldline):
                            if(pHang and poly.isHang()) or not pHang:
                                print "Active Pt Added", pt
                                active.append(pt)
                    drawp = Geometry2D.movePoly(p,direction,distance)
                    drawh = height
                    drawc = color
                    tableEdge = self.getClosestTableEdge(direction)
                    bisected = Geometry2D.bisectPoly(drawp, tableEdge)
                
                        #if not self.legalDragFold(tableEdge, SearchNode.get_polys(), True):
                         #   return False
                    for b in bisected:
                        if b!=False:
                            print "b is",b
                           # raw_input("BISECT POLY")
                            if self.isPolyHanging(b, direction):
                                drawc = Colors.lightenCV(color, 0.02)
                                cvpoly = CVPolygon(drawc,drawh,b)
                                cvpoly.setHang(True, direction)
                                self.lastFolded.append(cvpoly)
                                self.queueAddshape(cvpoly)
                            else:
                                drawp = b
                                drawh = height
                                drawc = color #Colors.lightenCV(color,50)
                                cvpoly = CVPolygon(drawc,drawh,drawp)
                                cvpoly.setHang(False)
                                self.lastFolded.append(cvpoly)
                else:
                    print "CURRENT FOLDLINE" , foldline
                    for pt in p.vertices():
                        print "Pt being checked", pt
                        if self.isActive(pt,poly,foldline,SearchNode.get_polys()) and not self.isFoldPerpendicularToHang(poly, foldline):
                            if(pHang and poly.isHang()) or not pHang:
                                active.append(pt)
                                print "Pt added", pt
                            print "point active but not added"
                    print active
                    drawp = Geometry2D.movePoly(p,direction,distance)
                    drawh = height
                    drawc = color
                    tableEdge = self.getClosestTableEdge(direction)
                    print "TableEdge for movePoly", tableEdge
                    bisected = Geometry2D.bisectPoly(drawp, tableEdge)
                    print bisected
                    #if not self.legalDragFold(tableEdge, SearchNode.get_polys(), True):
                            #return False
                    for b in bisected:
                        if b!=False:
                            print "b is" , b
#                            raw_input("BISECTING POLY")
                            if self.isPolyHanging(b, direction):
                                drawc = Colors.lightenCV(color,20)
                                cvpoly = CVPolygon(drawc,drawh,b)
                                cvpoly.setHang(True, direction)
                                self.lastFolded.append(cvpoly)
                                self.queueAddShape(cvpoly)
                                print b
                            else:
                                drawp = b
                                print b
 #                               raw_input("ENd of Bisect")
                                drawh = height
                                drawc = color
                                cvpoly = CVPolygon(drawc,drawh,drawp)
                                cvpoly.setHang(poly.isHang(), poly.getHangDirection())
                                self.queueAddShape(cvpoly)                
                #print "CVPOLY ADDED TO QUEUE FROM DRAG",cvpoly.shape
        self.queueRemoveShape(poly)
        print "Active verts", active
        return active


    def can_drag(self,gripPts,direction):
        print "can_drag, gripPts",
        for pt in gripPts:
            print pt
        if len(gripPts) <= 0:
            return False
        if len(gripPts) > self.numGrippers:            
            return False
        for pt in gripPts:
            print "can_drag checking pt",pt
            if not self.can_reach(pt,direction): # direction = robotPosition :)
                print "cannot reach, returning false"
                return False
        print "returning true"
        return True

    """
    def can_drag(self,polys,gripPts,direction='+y'):
        
        Tries to drag article along direction 'direction'
        direction: -x,+x,-y,+y in global coordinates
        gripPts: points to grip while dragging. May not always be the same as grip points for folding!

        Returns: If successful - True
                 If unsuccessful - False
                
        # check if the max width of the upperstring of all the polygons is less than epsilon + distance between the grip points
        epsilon = 10
        print "in can_drag"
        # find max upperstring width
        upperstring_maxwidth = 0
        for poly in polys:
            upperstring = self.upper_string(poly.shape,direction)        
            upperstring_width = abs(upperstring[0].y() - upperstring[-1].y())
            if(upperstring_width >= upperstring_maxwidth):
                upperstring_maxwidth = upperstring_width
        --- Visualize String ---
        for i in range(len(upperstring)-1):
            start = upperstring[i]
            end = string[i+1]
            string_element = CVLineSegment(color=Colors.RED,height=100,shape=Geometry2D.LineSegment(start,end))
            self.addOverlay(string_element)                           
              
        if(direction in ['-x','+x']):
            gripWidth = abs(gripPts[0].y() - gripPts[1].y())
        else:
            gripWidth = abs(gripPts[0].x() - gripPts[1].x())
    
        if upperstring_maxwidth <= epsilon + gripWidth:
            return True
        else:
            return False                      
            
    """
    def upper_string(self,poly,direction = '+y'):
        """
        direction: -x,+x,-y,+y in global coordinates        
        Given a polygon and direction of drag, returns vertices that make up the string of type string_type
        If there is more than 1 upper string, returns the one that contains the highest vertex
        """
        #print "in upper string"
        upper_string = []
        sides = poly.sides()
        vertices = poly.vertices()
        # extract all x/y vertices
        vertices_coords = [(vertex.x(),vertex.y()) for vertex in poly.vertices()]        
        # extract the relevant vertex (highest/lowest x/y). This vertex is guaranteed to be part of the upper_string
        fn = argmax if direction in ['-x','-y'] else argmin # highest vs lowest
        dim = 0 if direction in ['-x','+x'] else 1           # x vs y
        seg_dir = -1 if direction in ['+x','+y'] else 1           # direction of line_seg to check if vertex is part of upper_string
        highest_vertex_index = fn([x for(x,y) in vertices_coords])
        highest_vertex = vertices[highest_vertex_index]        
        upper_string.append(highest_vertex)        
        # extend upperstring in the "successor" direction
        complete = False
        while(not complete):
            vertex = poly.neighboring_vertices(upper_string[-1])[1]         
            # if vertex is already in upper_string, break
            if vertex in upper_string:
                break
            # raycast to check if this point is above all points in the polygon 
            end_pt = [vertex.x(),vertex.y()]        
            end_pt[dim] += seg_dir*10000
            line_seg =  Geometry2D.DirectedLineSegment(vertex,Geometry2D.Point(end_pt[0],end_pt[1]))
            for side in sides:
                if (vertex not in side.pts()) and Geometry2D.intersect(line_seg,side):
                    complete = True
                    break            
            if(not complete):
                upper_string.append(vertex)                
        # extend upperstring in the "predecessor" direction
        complete = False
        while(not complete):
            vertex = poly.neighboring_vertices(upper_string[0])[0]            
            if vertex in upper_string:
                break
            # raycast to check if this point is above all points in the polygon                                                                                                     
            end_pt = [vertex.x(),vertex.y()]
            end_pt[dim] += seg_dir*10000
            line_seg =  Geometry2D.DirectedLineSegment(vertex,Geometry2D.Point(end_pt[0],end_pt[1]))
            for side in sides:
                if (vertex not in side.pts()) and Geometry2D.intersect(line_seg,side): 
                    complete = True
                    break
            if(not complete):
                upper_string.insert(0,vertex)            
        return upper_string
    
    def move(self,direction,robotPosition):        
        if robotPosition == '+y':
            if direction == "l":
                robotPosition = "-x"
            elif direction == "r":
                robotPosition = "+x"

        elif self.robotPosition == "-x":
            if direction == "r":
                robotPosition = "+y"

        elif self.robotPosition == "+x":
            if direction == "l":
                robotPosition = "+y"
        

    def front(self):
        shapeFront = ShapeWindow.front(self)
        if len(self.addQueue) == 0:
            return shapeFront
        else:
            return max(shapeFront,max([cvShape.getHeight() for cvShape in self.addQueue])+1)

    def queueAddShape(self,cvpoly):
        self.addQueue.append(cvpoly)
        
    def queueRemoveShape(self,cvpoly):
        self.removeQueue.append(cvpoly)
    
    def flushQueue(self):
        for el in list(self.addQueue):
            self.addQueue.remove(el)
        for el in list(self.removeQueue):
            self.removeQueue.remove(el)
    
    def executeQueue(self,SearchNode = None, mirrorAxis = None, direc = None):
        if SearchNode!= None:
            #print "execute queue, before append",len(SearchNode.polys)
            newPolys = self.mergeAdjacentPoly(list(self.addQueue), mirrorAxis)
            for el in newPolys:
                print "executeQueue adding",el.dupl()
                SearchNode.polys.append(el.dupl())                
#                raw_input("POly Added to searchNode")
            self.flushQueue()
            return
        
        for el in self.removeQueue:
            self.removeCVShape(el)
        self.clearProposed()
        for el in self.addQueue:
            self.addPropCVShape(el.dupl())
            self.addCVShape(el)
        self.flushQueue()
        
# determine if polygon lies exlusively in the hang part of the fold.
    def isAnyPolyWithinHang(self,foldline,direction,cvpolys):
        print "FoldLine",foldline
        for cvs in [poly.getShape() for poly in cvpolys if not poly.isHang()]:     
            for vert in cvs.vertices():
                if(direc == '-x'):
                    if(foldline.isLeftOf(vert)):
                        return False
                elif(direc == '+x'):
                    if(foldline.isRightOf(vert)):
                        return False
                elif foldline.isRightOf(vert):
                    print "vert is",vert
                    return False
                else: 
                    continue
        return True

    def isPolyHanging(self, poly, direction):
        tableEdge  = self.getClosestTableEdge(direction)
        for vert in poly.vertices():
            if (direction == '-x' or direction == '+y') and tableEdge.isLeftOf(vert):
                print "Table Edge:" ,tableEdge, vert
                return False
            elif((direction == '+x') and tableEdge.isLeftOf(vert)):
                print "Table Edge +x", tableEdge, vert
                #raw_input()
                return False
        #print "before returning true",poly
        #raw_input()
        return True
            
            

    def activeVertices(self,foldline,cvpolys):
        activeVert = []
        for cvs in cvpolys:
                height = cvs.getHeight()
                verts = cvs.getShape().convexVertices()
                tempActive = []
                for v in verts:
                    if self.isActive(v,cvs,foldline) and not v in activeVert:
                        tempActive.append(v)
                activeVert.extend(tempActive)
        concave = []
        for cvs in self.getPolys():
            for vert in cvs.getShape().concaveVertices():
                concave.append(vert)
        return [vert for vert in activeVert if len([vert2 for vert2 in concave if Geometry2D.distance(vert,vert2) < 1])==0]
        
    def isActive(self,vertex,cvShape,foldline, polys):
        if foldline.isLeftOf(vertex) or foldline.contains(vertex):
            print "is left of ", vertex
            return False
        elif self.isCovered(vertex,cvShape,polys):
            print "is covered", vertex
            return False
        elif self.isSupported(vertex,cvShape,foldline):
            print "isSupported", vertex
            return False
        concave = []
        for cvs in polys:
            for vert in cvs.getShape().concaveVertices():
                concave.append(vert)
        if len([vert2 for vert2 in concave if Geometry2D.distance(vertex,vert2) < 2])>0:
            print "len of concave"
            return False
        else:
            return True
    
    def isSupported(self,vertex,cvShape,foldline):
        displ = Geometry2D.ptLineDisplacement(vertex,foldline)
        #Flip so displacement is going from the line to the vertex. Represents the negative gravity vector.
        displ.flip()
        if cvShape.getShape().contains(displ.extrapolate(1.01)):
            return True
        elif self.gravityRobustness > 0:
            #Tweak in both directions
            theta = self.gravityRobustness
            l = displ.length()
            newln = Geometry2D.LineSegment(displ.start(),foldline.end())
            displ.setStart(newln.extrapolate(l*tan(theta)/newln.length()))
            if cvShape.getShape().contains(displ.extrapolate(1.01)):
                return True
            displ.setStart(newln.extrapolate(-1*l*tan(theta)/newln.length()))
            if cvShape.getShape().contains(displ.extrapolate(1.01)):
                return True
        if not cvShape.getShape().isConvexPt(vertex):
            print "Not convex vertex", vertex
            return True
        return False
        
    def setGripperLimit(self,val):
        self.gripperLimit = val
        
        
    def gripPoints(self,activeVerts):
        if(self.wideGrip()):
            return self.optimizeGripPts(activeVerts)
        else:
            return list(activeVerts)

    def optimizeGripPts(self,activeVerts):
        toGrip = list(activeVerts)
        if len(toGrip) <= 1:
            return toGrip
        gripped = []
        gripPts = []
        (xRange,yRange) = Geometry2D.getBoundingBox(toGrip)
        possiblePts = [Geometry2D.Point(x,y) for x in xRange for y in yRange]
        while(len(toGrip) > 0):
            bestPt = max(possiblePts, key = lambda pt: self.gripScore(pt,toGrip))
            covered = self.coveredBy(bestPt,toGrip)
            gripped.extend(covered)
            for pt in covered:
                toGrip.remove(pt)
            gripPts.append(bestPt)
        return gripPts
                    
    def gripScore(self,pt,toGrip):
        covered = self.coveredBy(pt,toGrip)
        mainScore = len(covered)
        if len(covered) == 0:
            return 0
        norm_factor = 1.0 / (self.getGripSize()*len(toGrip))
        distScore = -1 * max([Geometry2D.distance(pt,covered_pt) for covered_pt in covered])*norm_factor
        return mainScore+distScore

    def coveredBy(self,gripPt,pts):
        r = self.getGripSize()
        return [pt for pt in pts if Geometry2D.distance(gripPt,pt) <= r]
    
    def getGripSize(self):
        return self.gripSize
    
    def setGripSize(self,r):
        self.gripSize = r
        
    def getNumGrippers(self):
        return self.numGrippers
    
    def setNumGrippers(self, numGrippers):
        self.numGrippers = numGrippers
        
    def wideGrip(self):
        return self.wideGripFlag
    
    def toggleWideGrip(self):
        self.wideGripFlag = not self.wideGripFlag
    
    def drawingMode(self):
        return self.drawingModeFlag
    
    def toggleDrawingMode(self):
        self.drawingModeFlag = not self.drawingModeFlag
        
    def drawGripper(self,pt):
        circ = Geometry2D.Circle(pt,self.getGripSize())
        gripper = CVCircle(Colors.GREY,self.tempFront(),circ,False)
        self.addTempCVShape(gripper)
        
    def getAccessible(self):
        acc = []
        temp = []
        for shape in self.getPolys():
            for l in shape.getShape().sides():
                temp.append((l,shape.getHeight(),shape.getShape()))    
        lineHeightShapeTuples = temp
        for (line,height,shape) in lineHeightShapeTuples:
            inters = [Geometry2D.intersect(line,line2) for (line2,height2,shape2) in lineHeightShapeTuples if line2 != line and height2 > height]
            bisectPts = [x for x in inters if x != False]
            for seg in Geometry2D.bisectLineByPts(line,bisectPts):
                if not self.isCovered(seg.center(),shape):
                    acc.append(seg)
        return acc

    def getSilhouette(self,polys):
        sil = []
        segs = ShapeWindowUtils.flatten([poly.getShape().sides() for poly in polys])
        while len(segs) > 0:
            seg = segs.pop()
            inters = [(Geometry2D.intersect(seg,seg2),seg2) for seg2 in segs]
            bisectPts = [(x,ln) for (x,ln) in inters if x != False and x != seg.start() and x != seg.end() and x != ln.start() and x != ln.end()]
            if len(bisectPts) == 0:
                if self.isOnSilOf(seg,polys):
                    sil.append(seg)
            else:
                for (x,ln) in bisectPts:
                    segs.remove(ln)
                    seg1 = Geometry2D.LineSegment(ln.start(),x)
                    seg2 = Geometry2D.LineSegment(x,ln.end())
                    for s in [seg1,seg2]:
                        if s.length() > 0:
                            segs.append(s)
                for newseg in Geometry2D.bisectLineByPts(seg,[x for (x,ln) in bisectPts]):
                    if self.isOnSilOf(newseg,polys) and newseg.length() > 0:
                        sil.append(newseg)
        collapsedSil = self.collapse(sil)
        return max(Geometry2D.getConnectedComponents(collapsedSil), key = lambda comp: Geometry2D.getBoundingArea(comp))
    
    def collapse(self,lines):
        output = []
        while len(lines) > 0:
            seg = lines[0]
            overlap = [seg2 for seg2 in lines if seg.overlaps(seg2)]
            for ln in overlap:
                lines.remove(ln)
            newln = Geometry2D.mergeLines(overlap)
            if newln.length() > 0:
                output.append(newln)
        return output
    
    def isOnSil(self,pt,shape):
        for shape2 in [cvShape.getShape() for cvShape in self.getShapes()]:
            if shape2 != shape and shape2.containsExclusive(pt):
                return False
        return True

    def isOnSilOf(self,seg,shapes):
        for shape in shapes:
            if shape.getShape().containsExclusive(seg.center()):
                return False
        return True

    def getFoldedRegion(self,cvpolys,foldline):
        foldline.expand(0.001)
        print "length of cvpoly",len(cvpolys), "foldline",foldline.start(), foldline.end()
        polys = [cvpoly.getShape() for cvpoly in cvpolys]
        toFold = []
        toNotFold = []
        newpolys = []
        rmpolys = []
        for poly in polys:
            halves = Geometry2D.bisectPoly(poly,foldline)
            #print poly,foldline, halves
            if not (False in halves):
                rmpolys.append(poly)
                for half in halves:
                    #if foldline.isRightOf(half.randPt()):
                    #connectedLines = [side for side in half.sides() if foldline.contains(side.end()) and not foldline == side]
                    #if foldline.isRightOf(connectedLines[0].start()):
                    if foldline.isRightOf(half.randPt()):
                        newpolys.append(half)
                    else:
                        toNotFold.append(half)
                        #print half
        for poly in rmpolys:
            polys.remove(poly)
        while len(newpolys) > 0:
            toFold.extend(newpolys)
            for newpoly in newpolys:
                if newpoly in polys:
                    polys.remove(newpoly)
            consecutive = []
            for newpoly in newpolys:
                consecutive.extend([poly for poly in polys if poly.adjacentTo(newpoly)])
            newpolys = consecutive
        toNotFold.extend(polys)
        return [toFold,toNotFold]


    def checkMirrorAxis(self,poly, mirrorAxis):
        for seg in poly.getShape().sides():
            if(mirrorAxis.overlaps(seg)):
                return True
        return False

    def getAdjacentVert(self,vert,vertices):
        adjacentVert = []
        for vertex in vertices:
            if(vertex == vert):
                continue
            if(vertex.x() == vert.x()):
                adjacentVert.append(vertex)
            if(vertex.y() == vert.y()):
                adjacentVert.append(vertex)
        return adjacentVert

    def mergeAdjacentPoly(self, polys, mirrorAxis):
        print " merge called"
        for poly in polys:
            print poly
        newPolys = []
        alreadyChecked = []
        for poly1 in polys:
            isadded = False
            #print "Start Poly One", poly1
            for poly2 in polys:
               # print "Checking Poly 2", poly2
                if poly1 == poly2:
                    print "Poly equal"
                    continue
                elif math.fabs(poly1.getHeight() - poly2.getHeight()) > 1:
                    print "Height unequal"
                    continue
                elif (poly1.isPrevHang() or poly2.isPrevHang()) and poly1.getShape().adjacentTo(poly2.getShape()) and self.checkMirrorAxis(poly1, mirrorAxis) and self.checkMirrorAxis(poly2,mirrorAxis) and not (poly1.isHang()) and not poly2.isHang():
                    print "Shape adjacent", alreadyChecked                       
                    if not (((poly1, poly2) in alreadyChecked) or ((poly2, poly1) in alreadyChecked)):

                        print "Checkig for poly vertices"
                        vertices = list(poly1.getShape().vertices())
                        for vert in poly2.getShape().vertices():
                            if (not vert in vertices):
                                vertices.append(vert)
                            else:
                                vertices.remove(vert)
                        arrangeVert = []
                        for vert in vertices:
                            if(arrangeVert == []):
                                arrangeVert.append(vert)
                                continue
                            else:
                                for vertAdj in self.getAdjacentVert(arrangeVert[-1], vertices):
                                    if not vertAdj in arrangeVert:
                                        arrangeVert.append(vertAdj)
                                        break
                        print arrangeVert
                        poly = Geometry2D.Polygon(*arrangeVert)
                        height  = min(poly1.getHeight(), poly2.getHeight())
                        cvpoly = CVPolygon(poly1.getDrawColor(),height,poly)
                        newPolys.append(cvpoly)
                        print "Added cvpoly" , cvpoly
                        alreadyChecked.append((poly1,poly2))
                        isadded = True
                    else:
                        isadded = True
            if not isadded:
                if poly1 not in newPolys:
                    newPolys.append(poly1)
        return newPolys
                                
    
    def setFoldCallback(self,callback):
        self.fold_callback = callback
    
    def echoFold(self,foldline,activeVerts,red,hang):
        if self.fold_callback:
            #thread.start_new_thread(self.fold_callback,(foldline,activeVerts,red))
            return self.fold_callback(foldline,activeVerts,red,hang)
            print "Finished echoing"

### Draw simulation #####


    def drawSimulationFolds(self,searchNodes):
        for i,searchNode in enumerate(searchNodes):
            for poly in searchNode.get_polys():
                self.addPropCVShape(poly)
            for availFolds in searchNode.get_availableFolds():
                foldline = Geometry2D.DirectedLineSegment(availFolds.start.dupl(),availFolds.end.dupl());
                dx, dy = searchNode.getDragDistance()
                foldline.translate(dx,dy)
#print "Foldline is",foldline 21
                self.addPropCVShape(CVDirectedLineSegment(cv.RGB(0,0,255),self.front(),foldline))
            if (i == (len(searchNodes)-1)):
                print "search finished"
                return
            else:
                print "Length of selected polys",len(searchNodes[i+1].get_polys())
                for poly in searchNodes[i+1].get_polys():
                    self.addSelectedCVShape(poly)
                self.drawRobot(searchNodes[i+1].robotPosition,1000)
                raw_input("Enter key")
                self.clearShapes();
                for poly in searchNodes[i+1].get_polys():
                    self.addCVShape(poly)
                self.robotPosition = searchNodes[i+1].robotPosition
                self.drawRobot(searchNodes[i+1].robotPosition)

##### Added for Simulation purposes:Originally in poly_gui_bridge.py################ 
    def handle_folds(self,vertices):
#        self.freezeMouse()
        for v in vertices:
            print v
        print "length of vert", len(vertices)
        if len(vertices) == 10 and self.mode == "shirt":
            self.start_logging()
            self.foldShirt_v3()
            self.stop_logging()
        elif len(vertices) == 10 and self.mode == "tee":
           # self.start_logging()
            self.foldTeeNoSleeve()
           # self.stop_logging()
        elif len(vertices) == 7 and self.mode == "pants":
            self.start_logging()
            self.foldPants_v2()
            self.stop_logging()
        elif len(vertices) == 4 and self.mode == "towel":            
            
            #self.start_logging()
            self.foldTowelThirds()    
            #self.stop_logging()
        
            
    def start_logging(self):
        self.start_time = rospy.Time.now()
        msg = "Starting to execute fold of type %s"%self.mode
        rospy.loginfo(msg)
        self.logfile.write("%s\n"%msg)
        print "Exiting log"

    def stop_logging(self):
        self.end_time = rospy.Time.now()
        dur = self.end_time - self.start_time
        msg = "Finished %s. Duration: %d.%d seconds"%(self.mode,dur.secs,dur.nsecs)
        rospy.loginfo(msg)
        self.logfile.write("%s\n"%msg)


###                            ###
#    Draw functions (Hard-Coded)#
###                            ###


    def drawTable(self, xoffset):
        
        bl = Geometry2D.Point(xoffset + 150, self.table_edge.start().y())
        tl = Geometry2D.Point(xoffset + 150, self.table_edge.start().y() - 400)
        tr = Geometry2D.Point(xoffset + 350, self.table_edge.start().y() - 400)
        br = Geometry2D.Point(xoffset + 350, self.table_edge.start().y())
        
        s1 = CVLineSegment(color=Colors.GREY, height = 100, shape=Geometry2D.LineSegment(bl, tl))
        s2 = CVLineSegment(color=Colors.GREY, height = 100, shape=Geometry2D.LineSegment(tl, tr))
        s3 = CVLineSegment(color=Colors.GREY, height = 100, shape=Geometry2D.LineSegment(tr, br))
        s4 = CVLineSegment(color=Colors.GREY, height = 100, shape=Geometry2D.LineSegment(br, bl))
        self.addOverlay(s1)
        self.addOverlay(s2)
        self.addOverlay(s3)
        self.addOverlay(s4)
    
    def translateFolds(self,fold, direction, distance):
        """
        Translates all remaining folds in accordance with current placement of cloth after drag
        """
        for child in fold.getChildren():
            if child.isPerformed():
                translateFolds(fold.getChildren())
            else:
                child.translate(self, direction, distance)
                translateFolds(child,fold, direction, distance)
                
    def makeRectangle(self,bottomLeft):
        bl = Geometry2D.Point(189, 366)
        tl = Geometry2D.Point(bl.x(), bl.y()-50)
        tr = Geometry2D.Point(bl.x() + 100, bl.y() - 50)
        br = Geometry2D.Point(bl.x() + 100, bl.y())
        return [bl, tl, tr, br]

    def makeShirt(self,bottomLeft):
        bl = Geometry2D.Point(189,400)
        la = Geometry2D.Point(bl.x(),bl.y() - 100)
        lsb = Geometry2D.Point(la.x() - 20, la.y() + 10)
        lst = Geometry2D.Point(lsb.x() - 10, lsb.y() - 40)
        ls = Geometry2D.Point(bl.x(), bl.y() - 150)
        rs = Geometry2D.Point(bl.x() + 100, bl.y() - 150)
        rst = Geometry2D.Point(rs.x() + 30, rs.y() + 20)
        rsb = Geometry2D.Point(rs.x() + 20, rs.y() + 60)
        ra = Geometry2D.Point(rs.x(), rs.y()  + 50)
        br = Geometry2D.Point(rs.x(),bl.y())
        return [bl, la, lsb, lst, ls, rs, rst, rsb, ra, br]
        



###                              ###
#   FOLD PRIMITIVES (Hard-Coded)   #
###                              ###

    def foldTee(self):
         [bottom_left,left_armpit,left_sleeve_bottom,left_sleeve_top,left_shoulder,right_shoulder,right_sleeve_top,right_sleeve_bottom,right_armpit,bottom_right] = self.getPolys()[0].getShape().vertices()
         bottom = Geometry2D.DirectedLineSegment(bottom_left,bottom_right)
         bottom.expand(0.01)
         top_left = Geometry2D.DirectedLineSegment(left_armpit,bottom.start()).extrapolate(-1.0)
         top_right = Geometry2D.DirectedLineSegment(right_armpit,bottom.end()).extrapolate(-1.0)
         self.gravityRobustness = pi/3
         sleeve_len = max(Geometry2D.distance(left_sleeve_bottom,left_sleeve_top),Geometry2D.distance(left_armpit,left_shoulder))
         self.wideGripFlag = True
         self.setGripSize(1.4*sleeve_len/2)
         
         #Sleeve 1
         seg = Geometry2D.LineSegment(left_shoulder,bottom_left)
         pt_l = left_armpit 
         blueEnd = top_left
         #self.blueEnd = left_shoulder
         #self.blueStart = left_armpit
         blueStart = Geometry2D.DirectedLineSegment(left_sleeve_bottom,left_armpit).extrapolate(0.95)
         newseg = Geometry2D.LineSegment(blueStart,blueEnd)
         newseg.expand(1.0)
         self.blueStart = newseg.start()
         self.blueEnd = newseg.end()

         firstfold = Fold(newseg.start(), newseg.end(), 'b')
         ###self.executeBlueFold()
         ###time.sleep(2.5)
         
         
         #Thirds
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(1.0/4.0)
         self.blueEnd = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(1.0/4.0)
         left_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         ###self.executeBlueFold()
         secondfold = Fold(self.blueStart.dupl(), self.blueEnd.dupl(), 'b')
         firstfold.addChild(secondfold)
         
         #Sleeve 2
         sleeve_len = max(Geometry2D.distance(right_sleeve_bottom,right_sleeve_top),Geometry2D.distance(right_armpit,right_shoulder))
         self.setGripSize(1.1*sleeve_len/2)
         seg = Geometry2D.LineSegment(right_shoulder,bottom_right)
         pt_r = right_armpit
         blueStart = top_right
         #self.blueStart = right_shoulder
         #self.blueEnd = right_armpit
         blueEnd = Geometry2D.DirectedLineSegment(right_sleeve_bottom,right_armpit).extrapolate(0.95)
         newseg = Geometry2D.LineSegment(blueStart,blueEnd)
         newseg.expand(1.0)
         self.blueStart = newseg.start()
         self.blueEnd = newseg.end()
         
         thirdfold = Fold(newseg.start(), newseg.end(), 'b')
 
###self.executeBlueFold()
         ###time.sleep(2.5)
         #Thirds
         self.blueEnd = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(3.0/4.0)
         self.blueStart = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(3.0/4.0)
         right_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         ####self.executeBlueFold()
#         time.sleep(2.5)
         #Finally, in half
         fourth = Fold(self.blueStart.dupl(), self.blueEnd.dupl())

         thirdfold.addChild(fourth)
         
         #self.setGripSize(1.05*Geometry2D.distance(left_shoulder,right_shoulder)/2)
         top = Geometry2D.LineSegment(left_shoulder,right_shoulder)
         top.expand(0.5)
         left_third_adj = Geometry2D.LineSegment(left_third.start(),Geometry2D.intersect(left_third,top))
         right_third_adj = Geometry2D.LineSegment(right_third.end(),Geometry2D.intersect(right_third,top))
         blueEnd = left_third_adj.center()
         blueStart = right_third_adj.center()
         fold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
         #self.setGripSize(fold.length()*1.2/2)
         self.setGripSize(fold.length()*0.9/2)
         fold.expand(1.5)
         self.blueEnd = fold.start()
         self.blueStart = fold.end()
         self.executeBlueFold()
         time.sleep(2.5)
         
    def foldTeeNoSleeve(self):
         [bottom_left,left_armpit,left_sleeve_bottom,left_sleeve_top,left_shoulder,right_shoulder,right_sleeve_top,right_sleeve_bottom,right_armpit,bottom_right] = self.getPolys()[0].getShape().vertices()
         bottom = Geometry2D.DirectedLineSegment(bottom_left,bottom_right)
         bottom.expand(0.01)
         top_left = Geometry2D.DirectedLineSegment(left_armpit,bottom.start()).extrapolate(-1.0)
         top_right = Geometry2D.DirectedLineSegment(right_armpit,bottom.end()).extrapolate(-1.0)
         self.gravityRobustness = pi/3
         sleeve_len = max(Geometry2D.distance(left_sleeve_bottom,left_sleeve_top),Geometry2D.distance(left_armpit,left_shoulder))
         self.wideGripFlag = True
         self.setGripSize(1.4*sleeve_len/2)
         
         self.FoldTree = []
         #Sleeve 1
         seg = Geometry2D.LineSegment(left_shoulder,bottom_left)
         pt_l = left_armpit 
         blueEnd = top_left
         #self.blueEnd = left_shoulder
         #self.blueStart = left_armpit
         blueStart = Geometry2D.DirectedLineSegment(left_sleeve_bottom,left_armpit).extrapolate(0.95)
         newseg = Geometry2D.LineSegment(blueStart,blueEnd)
         newseg.expand(1.0)
         self.blueStart = newseg.start()
         self.blueEnd = newseg.end()
         #self.executeBlueFold()
         #time.sleep(2.5)
         firstFold = Fold(self.blueStart, self.blueEnd, 'b')
         
         #Thirds
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(1.0/4.0 - 0.05)
         self.blueEnd = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(1.0/4.0 - 0.05)
         left_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
#         self.executeBlueFold() ME
         secondFold = Fold(self.blueStart, self.blueEnd, 'b')
         
         #Sleeve 2
         sleeve_len = max(Geometry2D.distance(right_sleeve_bottom,right_sleeve_top),Geometry2D.distance(right_armpit,right_shoulder))
         self.setGripSize(1.1*sleeve_len/2)
         seg = Geometry2D.LineSegment(right_shoulder,bottom_right)
         pt_r = right_armpit
         blueStart = top_right
         #self.blueStart = right_shoulder
         #self.blueEnd = right_armpit
         blueEnd = Geometry2D.DirectedLineSegment(right_sleeve_bottom,right_armpit).extrapolate(0.95)
         newseg = Geometry2D.LineSegment(blueStart,blueEnd)
         newseg.expand(1.0)
         self.blueStart = newseg.start()
         self.blueEnd = newseg.end()
         thirdFold = Fold(self.blueStart, self.blueEnd,'b')
         #self.executeBlueFold()
         #time.sleep(2.5)
         #Thirds
         self.blueEnd = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(3.0/4.0 + 0.05)
         self.blueStart = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(3.0/4.0 + 0.05)
         right_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
#         self.executeBlueFold() ME
         fourthFold = Fold(self.blueStart, self.blueEnd, 'b')

         #time.sleep(2.5)
         
#Finally, in half
         
         
         #self.setGripSize(1.05*Geometry2D.distance(left_shoulder,right_shoulder)/2)
         top = Geometry2D.LineSegment(left_shoulder,right_shoulder)
         top.expand(0.5)
         left_third_adj = Geometry2D.LineSegment(left_third.start(),Geometry2D.intersect(left_third,top))
         right_third_adj = Geometry2D.LineSegment(right_third.end(),Geometry2D.intersect(right_third,top))
         blueEnd = left_third_adj.center()
         blueStart = right_third_adj.center()
         fold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
         #fifthFold = Fold(self.blueStart, self.blueEnd, 'b')
         #self.setGripSize(fold.length()*1.3/2)
         self.setGripSize(fold.length()*0.8/2)
         fold.expand(1.5)
         self.blueEnd = fold.start()
         self.blueStart = fold.end()
         sixthFold = Fold(fold.start(), fold.end(), 'b')
#         self.executeBlueFold() ME
#         time.sleep(2.5)
         firstFold.addChild(secondFold)
         secondFold.addChild(thirdFold)
         thirdFold.addChild(fourthFold)
         fourthFold.addChild(sixthFold)
         #fifthFold.addChild(sixthFold)
         self.foldTree = [firstFold]
         self.startpoly = self.getPolys()[0]
         self.readytoFold = True

         
    def foldTee_v2(self):
         [bottom_left,left_armpit,left_sleeve_bottom,left_sleeve_top,left_neck,right_neck,right_sleeve_top,right_sleeve_bottom,right_armpit,bottom_right] = self.getPolys()[0].getShape().vertices()
         self.gravityRobustness = pi/3
         sleeve_len = Geometry2D.distance(left_sleeve_bottom,left_sleeve_top)
         self.wideGripFlag = True
         self.setGripSize(1.05*sleeve_len/2)
         
         #Sleeve 1
         self.left_side = Geometry2D.DirectedLineSegment(bottom_left,left_armpit).extrapolate
         self.blueEnd = Geometry2D.DirectedLineSegment(bottom_left,left_armpit).extrapolate(2.0)
         self.blueStart = left_armpit
         self.executeBlueFold()
         time.sleep(2.5)
         
         
         #Thirds
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(1.0/4.0)
         self.blueEnd = left_neck
         left_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         
         #Sleeve 2
         sleeve_len = Geometry2D.distance(right_sleeve_bottom,right_sleeve_top)
         self.setGripSize(1.05*sleeve_len/2)
         
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_right,right_armpit).extrapolate(2.0)
         self.blueEnd = right_armpit
         self.executeBlueFold()
         time.sleep(2.5)
         #Thirds
         self.blueEnd = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(3.0/4.0)
         self.blueStart = right_neck
         right_third = Geometry2D.DirectedLineSegment(self.blueEnd,self.blueStart)
         self.executeBlueFold()
         time.sleep(2.5)
         #Finally, in half
         
         #self.setGripSize(1.05*Geometry2D.distance(left_shoulder,right_shoulder)/2)
         self.blueStart = left_third.center()
         self.blueEnd = right_third.center()
         
         self.executeBlueFold()
         time.sleep(2.5)
         
    
    def foldShirt_v1(self):
        self.freezeMouse()
        #First, identify the different parts of the shirt. For now assume a simple ordering
        [bottom_left,left_armpit,left_sleeve_bottom,left_sleeve_top,left_shoulder,right_shoulder,right_sleeve_top,right_sleeve_bottom,right_armpit,bottom_right] = self.getPolys()[0].getShape().vertices()
        self.wideGripFlag = True
        sleeve_height = max(Geometry2D.distance(left_sleeve_bottom,left_sleeve_top),Geometry2D.distance(right_sleeve_bottom,right_sleeve_top))
        self.setGripSize(1.05*sleeve_height/2)
        self.blueStart = left_armpit
        self.blueEnd = left_shoulder
        self.executeBlueFold()
        print "Finished Executing"
        time.sleep(2.5)
        new_left_sleeve_bottom = Geometry2D.mirrorPt(left_sleeve_bottom,Geometry2D.LineSegment(self.blueStart,self.blueEnd))
        new_left_sleeve_top = Geometry2D.mirrorPt(left_sleeve_top,Geometry2D.LineSegment(self.blueStart,self.blueEnd))
        dx = new_left_sleeve_top.x() - left_shoulder.x()
        dy = new_left_sleeve_top.y() - left_shoulder.y()
        if dy == 0:
            theta = math.pi/2
        else:
            theta = math.atan(dx/dy)
        sleeve_length = Geometry2D.distance(left_sleeve_top,left_shoulder)
        self.redStart = left_shoulder
        self.redEnd = Geometry2D.intersect  (Geometry2D.Line(left_armpit,new_left_sleeve_bottom)
                                            ,Geometry2D.Line(
                                                                    left_shoulder,
                                                                    Geometry2D.Point(left_shoulder.x() + sleeve_length*math.tan(theta/2),left_shoulder.y()+sleeve_length)))
        self.executeRedFold()
        time.sleep(2.5)
        self.blueEnd = right_armpit
        self.blueStart = right_shoulder
        self.executeBlueFold()
        time.sleep(2.5)
        new_right_sleeve_bottom = Geometry2D.mirrorPt(right_sleeve_bottom,Geometry2D.LineSegment(self.blueStart,self.blueEnd))
        new_right_sleeve_top = Geometry2D.mirrorPt(right_sleeve_top,Geometry2D.LineSegment(self.blueStart,self.blueEnd))
        dx = new_right_sleeve_top.x() - right_shoulder.x()
        dy = new_right_sleeve_top.y() - right_shoulder.y()
        if dy == 0:
            theta = math.pi/2
        else:
            theta = math.atan(dx/dy)
        sleeve_length = Geometry2D.distance(right_sleeve_top,right_shoulder)
        self.redEnd = right_shoulder
        self.redStart = Geometry2D.intersect  (Geometry2D.Line(right_armpit,new_right_sleeve_bottom)
                                            ,Geometry2D.Line(
                                                                    right_shoulder,
                                                                    Geometry2D.Point(right_shoulder.x() - sleeve_length*math.tan(theta/2),right_shoulder.y()+sleeve_length)))
        self.executeRedFold()
        self.wideGripFlag = False
        time.sleep(2.5)
        self.blueStart = Geometry2D.LineSegment(bottom_right,bottom_left).extrapolate(0.5)
        self.blueEnd = Geometry2D.LineSegment(right_shoulder,left_shoulder).extrapolate(0.5)
        self.executeBlueFold()
        time.sleep(2.5)
        self.blueEnd = right_armpit
        self.blueStart = left_armpit
        self.executeBlueFold()
        time.sleep(2.5)
        self.blueEnd = Geometry2D.LineSegment(bottom_left,left_armpit).extrapolate(0.5)
        self.blueStart = Geometry2D.LineSegment(bottom_right,right_armpit).extrapolate(0.5)
        self.executeBlueFold()
        self.unfreezeMouse()
        return
        
    def foldShirt_v2(self):
        [bottom_left,left_armpit,left_sleeve_bottom,left_sleeve_top,left_shoulder,right_shoulder,right_sleeve_top,right_sleeve_bottom,right_armpit,bottom_right] = self.getPolys()[0].getShape().vertices()
        self.wideGripFlag = True
        self.gravityRobustness = pi/8
        sleeve_height = Geometry2D.distance(left_sleeve_bottom,left_sleeve_top)
        self.setGripSize(1.05*sleeve_height/2)
        #First, fold both sleeves straight in
        dx = left_armpit.x() - left_sleeve_bottom.x()
        dy = left_sleeve_bottom.y() - left_armpit.y()
        
        theta1 = Geometry2D.safeArctan(dx,dy)
        dx = right_armpit.x() - left_armpit.x()
        dy = left_armpit.y() - right_armpit.y()
        theta2 = Geometry2D.safeArctan(dy,dx)
        theta = theta1 + pi/2 + theta2
        theta3 = theta/2 - theta1
        self.blueStart = Geometry2D.LineSegment(left_armpit,left_sleeve_bottom).extrapolate(0.005)
        self.blueEnd = Geometry2D.Point(left_armpit.x() - left_armpit.y()*tan(theta3),0)
        self.executeBlueFold()
        time.sleep(2.5)
        redFold = Geometry2D.DirectedLineSegment(right_shoulder,right_armpit)
        redFold.expand(2.0)  
        self.redStart = redFold.start()
        self.redEnd = redFold.end()
        self.executeRedFold()
        time.sleep(2.5)
        sleeve_height = Geometry2D.distance(right_sleeve_bottom,right_sleeve_top)
        self.setGripSize(1.05*sleeve_height/2)
        dx = right_sleeve_bottom.x() - right_armpit.x()
        dy = right_sleeve_bottom.y() - right_armpit.y()
        
        theta1 = Geometry2D.safeArctan(dx,dy)
        dx = -1* (right_armpit.x() - left_armpit.x())
        dy = -1*(left_armpit.y() - right_armpit.y())
        theta2 = Geometry2D.safeArctan(dy,dx)
        theta = theta1 + pi/2 + theta2
        theta3 = theta/2 - theta1
        self.blueEnd = Geometry2D.LineSegment(right_armpit,right_sleeve_bottom).extrapolate(0.02)
        self.blueStart = Geometry2D.Point(right_armpit.x() + right_armpit.y()*tan(theta3),0)
        self.executeBlueFold()
        time.sleep(2.5)
        redFold = Geometry2D.DirectedLineSegment(left_armpit,left_shoulder)
        redFold.expand(2.0)  
        self.redStart = redFold.start()
        self.redEnd = redFold.end()
        self.executeRedFold()
        time.sleep(2.5)
        #Fold in thirds
        blueFold = Geometry2D.DirectedLineSegment(Geometry2D.LineSegment(bottom_left,bottom_right).extrapolate(1.0/3.0),Geometry2D.LineSegment(left_shoulder,right_shoulder).extrapolate(1.0/3.0))
        blueFold.expand(2.0)
        self.blueStart = blueFold.start()
        self.blueEnd = blueFold.end()
        self.executeBlueFold()
        time.sleep(2.5)
        blueFold = Geometry2D.DirectedLineSegment(Geometry2D.LineSegment(left_shoulder,right_shoulder).extrapolate(2.0/3.0),Geometry2D.LineSegment(bottom_left,bottom_right).extrapolate(2.0/3.0))
        blueFold.expand(2.0)
        self.blueEnd = blueFold.end()
        self.blueStart = blueFold.start()
        self.executeBlueFold()
        time.sleep(2.5)
        finalFold = Geometry2D.DirectedLineSegment(Geometry2D.LineSegment(bottom_right,right_shoulder).center(),Geometry2D.LineSegment(bottom_left,left_shoulder).center())
        finalFold.expand(2.0)
        self.blueStart = finalFold.start()
        self.blueEnd = finalFold.end()
        self.executeBlueFold()
        self.gravityRobustness = 0
        self.wideGripFlag = False

#VERSION 3 of SWEATER FOLDING
    def foldShirt_v3(self):
         [bottom_left,left_armpit, left_sleeve_bottom,left_sleeve_top,left_shoulder,right_shoulder, right_sleeve_top,right_sleeve_bottom,right_armpit, bottom_right] = self.getPolys()[0].getShape().vertices()
         bottom = Geometry2D.DirectedLineSegment(bottom_left,bottom_right)
         bottom.expand(0.01)
         top_left = Geometry2D.DirectedLineSegment(left_armpit,bottom.start()).extrapolate(-1.0)
         top_right = Geometry2D.DirectedLineSegment(right_armpit,bottom.end()).extrapolate(-1.0)
         self.gravityRobustness = pi/3
         sleeve_len = max(Geometry2D.distance(left_sleeve_bottom,left_sleeve_top),Geometry2D.distance(left_armpit,left_shoulder))
         self.wideGripFlag = True
         self.setGripSize(1.5*sleeve_len/2)
         
         #Sleeve 1
         dx = left_sleeve_top.x() - left_sleeve_bottom.x()
         dy = left_sleeve_top.y() - left_sleeve_bottom.y()
         blueStart = Geometry2D.LineSegment(left_sleeve_bottom,left_armpit).center()
         blueEnd = Geometry2D.Point(blueStart.x()+dx,blueStart.y()+dy)
         seg = Geometry2D.LineSegment(blueStart,blueEnd)
         seg.expand(1.0)#was 0.5
         self.blueStart = seg.start()
         self.blueEnd = seg.end()
         self.executeBlueFold()
         time.sleep(2.5)
         seg = Geometry2D.LineSegment(left_shoulder,bottom_left)
         pt_l = left_armpit 
         self.blueEnd = top_left
         #self.blueEnd = left_shoulder
         self.blueStart = left_armpit
         self.executeBlueFold()
         time.sleep(2.5)
         
         
         #Thirds
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(1.0/4.0)
         self.blueEnd = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(1.0/4.0)
         left_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         
         #Sleeve 2
         sleeve_len = max(Geometry2D.distance(right_sleeve_bottom,right_sleeve_top),Geometry2D.distance(right_armpit,right_shoulder))
         self.setGripSize(1.5*sleeve_len/2)
         dx = right_sleeve_top.x() - right_sleeve_bottom.x()
         dy = right_sleeve_top.y() - right_sleeve_bottom.y()
         blueStart = Geometry2D.LineSegment(right_sleeve_bottom,right_armpit).center()
         blueEnd = Geometry2D.Point(blueStart.x()+dx,blueStart.y()+dy)
         seg = Geometry2D.LineSegment(blueStart,blueEnd)
         seg.expand(1.0)#Was 0.5
         self.blueStart = seg.end()
         self.blueEnd = seg.start()
         self.executeBlueFold()
         seg = Geometry2D.LineSegment(right_shoulder,bottom_right)
         pt_r = right_armpit
         self.blueStart = top_right
         #self.blueStart = right_shoulder
         self.blueEnd = right_armpit
         self.executeBlueFold()
         time.sleep(2.5)
         #Thirds
         self.blueEnd = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(3.0/4.0)
         self.blueStart = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(3.0/4.0)
         right_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         time.sleep(2.5)
         #Finally, in half
         
         #self.setGripSize(1.05*Geometry2D.distance(left_shoulder,right_shoulder)/2)
         top = Geometry2D.LineSegment(left_shoulder,right_shoulder)
         top.expand(0.5)
         left_third_adj = Geometry2D.LineSegment(left_third.start(),Geometry2D.intersect(left_third,top))
         right_third_adj = Geometry2D.LineSegment(right_third.end(),Geometry2D.intersect(right_third,top))
         blueEnd = left_third_adj.center()
         blueStart = right_third_adj.center()
         fold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
         self.setGripSize(fold.length()*0.85/2)
         fold.expand(0.5)
         self.blueStart = fold.end()
         self.blueEnd = fold.start()
         #self.blueStart = fold.end()
         #self.blueEnd = fold.start()
         self.executeBlueFold()
         time.sleep(2.5)
         
#VERSION 4 of SWEATER FOLDING
    def foldShirt_v4(self):
         [bottom_left,left_armpit, left_sleeve_bottom,left_sleeve_top,left_shoulder,right_shoulder, right_sleeve_top,right_sleeve_bottom,right_armpit, bottom_right] = self.getPolys()[0].getShape().vertices()
         bottom = Geometry2D.DirectedLineSegment(bottom_left,bottom_right)
         bottom.expand(0.01)
         top_left = Geometry2D.DirectedLineSegment(left_armpit,bottom.start()).extrapolate(-1.0)
         top_right = Geometry2D.DirectedLineSegment(right_armpit,bottom.end()).extrapolate(-1.0)
         self.gravityRobustness = pi/3
         sleeve_len = max(Geometry2D.distance(left_sleeve_bottom,left_sleeve_top),Geometry2D.distance(left_armpit,left_shoulder))
         self.wideGripFlag = True
         self.setGripSize(1.5*sleeve_len/2)
         
         #Sleeve 1
         time.sleep(2.5)
         seg = Geometry2D.LineSegment(left_shoulder,bottom_left)
         pt_l = left_armpit 
         self.blueEnd = top_left
         #self.blueEnd = left_shoulder
         self.blueStart = left_armpit
         lastfold = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         time.sleep(2.5)
         new_left_sleeve_top = Geometry2D.mirrorPt(left_sleeve_top,lastfold)
         new_left_sleeve_bottom = Geometry2D.mirrorPt(left_sleeve_bottom,lastfold)
         real_top_left = Geometry2D.intersect(Geometry2D.Line(left_sleeve_top,left_shoulder),lastfold)
        
         
         #Other approach
         redStart = Geometry2D.Point((new_left_sleeve_top.x() + real_top_left.x())/2,(new_left_sleeve_top.y() + real_top_left.y())/2)
         self.highlightPt(self.redStart)
         sleeve_top = Geometry2D.DirectedLineSegment(real_top_left,new_left_sleeve_top)
         sleeve_bottom = Geometry2D.DirectedLineSegment(left_armpit,new_left_sleeve_bottom)
         sleeve = min((sleeve_top,sleeve_bottom),key=lambda n: n.length())
         
         redStart = sleeve.center()
         redEnd = Geometry2D.Point(redStart.x() - sleeve.dy(),redStart.y() + sleeve.dx())
         fold = Geometry2D.DirectedLineSegment(redStart,redEnd)
         fold.expand(5.0)
         self.redStart = fold.start()
         self.redEnd = fold.end()
         self.executeRedFold()
         time.sleep(2.5)
         #Thirds
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(1.0/4.0)
         self.blueEnd = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(1.0/4.0)
         left_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         
         #Sleeve 2
         seg = Geometry2D.LineSegment(right_shoulder,bottom_right)
         pt_r = right_armpit
         self.blueStart = top_right
         #self.blueStart = right_shoulder
         self.blueEnd = right_armpit
         lastfold = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         time.sleep(2.5)
         new_right_sleeve_top = Geometry2D.mirrorPt(right_sleeve_top,lastfold)
         new_right_sleeve_bottom = Geometry2D.mirrorPt(right_sleeve_bottom,lastfold)
         real_top_right = Geometry2D.intersect(Geometry2D.Line(right_sleeve_top,right_shoulder),lastfold) 
         
         #Other approach
         redStart = Geometry2D.Point((new_right_sleeve_top.x() + real_top_right.x())/2,(new_right_sleeve_top.y() + real_top_right.y())/2)
         self.highlightPt(self.redStart)
         sleeve_top = Geometry2D.DirectedLineSegment(real_top_right,new_right_sleeve_top)
         sleeve_bottom = Geometry2D.DirectedLineSegment(right_armpit,new_right_sleeve_bottom)
         sleeve = min((sleeve_top,sleeve_bottom),key=lambda n: n.length())
         #sleeve = sleeve_top
         redStart = sleeve.center()
         redEnd = Geometry2D.Point(redStart.x() - sleeve.dy(),redStart.y() + sleeve.dx())
         fold = Geometry2D.DirectedLineSegment(redStart,redEnd)
         fold.expand(5.0)
         self.redStart = fold.start()
         self.redEnd = fold.end()
         self.executeRedFold()
         time.sleep(2.5)
         #Thirds
         self.blueEnd = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(3.0/4.0)
         self.blueStart = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(3.0/4.0)
         right_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         time.sleep(2.5)
         #Finally, in half
         
         #self.setGripSize(1.05*Geometry2D.distance(left_shoulder,right_shoulder)/2)
         top = Geometry2D.LineSegment(left_shoulder,right_shoulder)
         top.expand(0.5)
         left_third_adj = Geometry2D.LineSegment(left_third.start(),Geometry2D.intersect(left_third,top))
         right_third_adj = Geometry2D.LineSegment(right_third.end(),Geometry2D.intersect(right_third,top))
         blueEnd = left_third_adj.center()
         blueStart = right_third_adj.center()
         fold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
         self.setGripSize(fold.length()*1.2/2)
         fold.expand(0.5)
         self.blueStart = fold.start()
         self.blueEnd = fold.end()
         self.executeBlueFold()
         time.sleep(2.5)
        
    def foldPants(self):
        [left_leg_right,left_leg_left,top_left,top_right,right_leg_right,right_leg_left,crotch] = self.getPolys()[0].getShape().vertices()
        self.wideGripFlag = True
        self.setGripSize(2.0)
        l1 = Geometry2D.distance(top_right,right_leg_right)
        l2 = Geometry2D.distance(crotch,right_leg_left)
        blueFold = Geometry2D.DirectedLineSegment(Geometry2D.LineSegment(right_leg_right,top_right).extrapolate(0.5),Geometry2D.LineSegment(right_leg_left,crotch).extrapolate(0.5*l1/l2))
        blueFold.expand(0.01)
        self.blueStart = blueFold.start()
        self.blueEnd = blueFold.end()
        self.executeBlueFold()
        time.sleep(2.5)
        top_ctr = Geometry2D.LineSegment(top_left,top_right).center()
        
        l1 = Geometry2D.distance(top_left,left_leg_left)
        l2 = Geometry2D.distance(crotch,left_leg_right)
        blueFold = Geometry2D.DirectedLineSegment(Geometry2D.LineSegment(left_leg_left,top_left).extrapolate(0.5),Geometry2D.LineSegment(left_leg_right,crotch).extrapolate(0.5*l1/l2))
        blueFold.expand(0.01)
        self.blueStart = blueFold.end()
        self.blueEnd = blueFold.start()
        self.executeBlueFold()
        time.sleep(2.5)
        
        blueFold = Geometry2D.DirectedLineSegment(crotch,top_ctr)
        blueFold.expand(1.0)
        self.blueStart = blueFold.start()
        self.blueEnd = blueFold.end()
        self.executeBlueFold()
        time.sleep(2.5)
        self.wideGripFlag = False
        
    def foldPants_v2(self):
        [left_leg_right,left_leg_left,top_left,top_right,right_leg_right,right_leg_left,crotch] = self.getPolys()[0].getShape().vertices()
        self.gravityRobustness = pi/8
        top_ctr = Geometry2D.LineSegment(top_left,top_right).center()
        bottom_ctr = Geometry2D.LineSegment(left_leg_right,right_leg_left).center()
        leg_width = max(Geometry2D.distance(left_leg_right,left_leg_left),Geometry2D.distance(right_leg_right,right_leg_left))
        waist_width = Geometry2D.distance(top_left,top_right)
        self.blueStart = bottom_ctr
        self.blueEnd = top_ctr
        self.executeBlueFold()
        time.sleep(2.5)
        self.wideGripFlag = True
        self.gripSize = 1.25*leg_width/2
        #l1 = max(Geometry2D.distance(top_left,left_leg_left),Geometry2D.distance(top_right,right_leg_right))
        #l2 = max(Geometry2D.distance(crotch,left_leg_right),Geometry2D.distance(crotch,right_leg_left))
        #blueFold = Geometry2D.DirectedLineSegment(Geometry2D.LineSegment(right_leg_right,top_right).extrapolate(0.5),Geometry2D.LineSegment(right_leg_left,crotch).extrapolate(0.5*l1/l2))
        top_ln = Geometry2D.LineSegment(top_right,right_leg_right)
        bottom_ln = Geometry2D.LineSegment(top_ctr,right_leg_left)
        blueStart = top_ln.center()
  
        blueEnd = bottom_ln.center()
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        blueFold.expand(1.0)
        self.blueStart = blueFold.start()
        self.blueEnd = blueFold.end()
        self.executeBlueFold()
        time.sleep(2.5)
       
        self.wideGripFlag = False
        self.gravityRobustness = 0
        
    def foldTowel(self):
        [bl,tl,tr,br] = self.getPolys()[0].getShape().vertices()
        height = max(Geometry2D.distance(bl,tl),Geometry2D.distance(br,tr))
        width = max(Geometry2D.distance(tl,tr),Geometry2D.distance(bl,br))
                
        #Fold in half
        self.blueEnd = Geometry2D.LineSegment(bl,tl).center()
        self.blueStart = Geometry2D.LineSegment(br,tr).center()
        self.executeBlueFold()
        time.sleep(2.5)
        self.wideGripFlag = True
        self.setGripSize(1.05*height/4)
        #Fold in half again
        blueStart = Geometry2D.LineSegment(self.blueStart,self.blueEnd).center()
        blueEnd = Geometry2D.LineSegment(tl,tr).center()
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        blueFold.expand(2.0)
        self.blueEnd = blueFold.start()
        self.blueStart = blueFold.end()
        self.executeBlueFold()
        time.sleep(2.5)
        self.wideGripFlag = False
     
    def foldTowelThirds(self):
        print "in fold towel thirds"
        [bl,tl,tr,br] = self.getPolys()[0].getShape().vertices()        
        height = max(Geometry2D.distance(bl,tl),Geometry2D.distance(br,tr))
        width = max(Geometry2D.distance(tl,tr),Geometry2D.distance(bl,br))
        table_start = Geometry2D.Point(bl.x() - 10,bl.y())
        table_end = Geometry2D.Point(br.x() + 10, br.y())        
                
        """ New Fold Sequence using Tree of Folds """
        self.FoldTree = []
         #Fold in half
        l_ctr = Geometry2D.LineSegment(bl,tl).center()
        r_ctr = Geometry2D.LineSegment(br,tr).center()
        blueEnd = Geometry2D.LineSegment(bl,tl).center()
        blueStart = Geometry2D.LineSegment(br,tr).center()
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        blueFold.expand(0.05)
        #print "Blue Start %s"%(self.blueStart)
        #print "Blue End %s"%(self.blueEnd)

        firstFold = Fold(blueFold.start(), blueFold.end(),'b')
        
        #Second Fold in half again
        blueStart = Geometry2D.LineSegment(l_ctr,r_ctr).extrapolate(2/3.0 + 0.05)
        blueEnd = Geometry2D.LineSegment(tl,tr).extrapolate(2/3.0 + 0.05)
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        blueFold.expand(2.0)
        blueStart = blueFold.end()
        blueEnd = blueFold.start()
        secondFold = Fold(blueStart, blueEnd, 'b')

        blueStart = Geometry2D.LineSegment(l_ctr,r_ctr).extrapolate(1/3.0 - 0.05)
        blueEnd = Geometry2D.LineSegment(tl,tr).extrapolate(1/3.0 - 0.05)
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        blueFold.expand(2.0)
        blueStart = blueFold.start()
        blueEnd = blueFold.end()
        thirdFold = Fold(blueStart, blueEnd, 'b')

        secondFold.addChild(thirdFold)
        firstFold.addChild(secondFold)
        self.foldTree = [firstFold]        
        self.startpoly = self.getPolys()[0]
        self.readytoFold = True
        #-------------- Old definition ---------------------- #
        """
        #Fold in half
        l_ctr = Geometry2D.LineSegment(bl,tl).center()
        r_ctr = Geometry2D.LineSegment(br,tr).center()
        blueEnd = Geometry2D.LineSegment(bl,tl).center()
        blueStart = Geometry2D.LineSegment(br,tr).center()
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        #blueFold.expand(0.05)
        self.blueStart = blueFold.start()
        self.blueEnd = blueFold.end()
        print "Blue Start %s"%(self.blueStart)
        print "Blue End %s"%(self.blueEnd)      
                
        self.executeBlueFold()
        time.sleep(2.5)
        self.wideGripFlag = True;
        self.setGripSize(1.05*height/3.5)
        #Fold in half again
        blueStart = Geometry2D.LineSegment(l_ctr,r_ctr).extrapolate(2/3.0 + 0.05)
        blueEnd = Geometry2D.LineSegment(tl,tr).extrapolate(2/3.0 + 0.05)
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        blueFold.expand(2.0)
        self.blueStart = blueFold.end()
        self.blueEnd = blueFold.start()
        self.executeBlueFold()
        time.sleep(2.5)
        blueStart = Geometry2D.LineSegment(l_ctr,r_ctr).extrapolate(1/3.0 - 0.05)
        blueEnd = Geometry2D.LineSegment(tl,tr).extrapolate(1/3.0 - 0.05)
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        blueFold.expand(2.0)
        self.blueStart = blueFold.start()
        self.blueEnd = blueFold.end()
        self.executeBlueFold()
        time.sleep(2.5)
        self.wideGripFlag = False 
        """
 
class Fold:
    def __init__(self, startPoint, endPoint , foldtype):
        self.start = startPoint
        self.end = endPoint
        self.type = foldtype
        self.performed = False
        self.children = []
        self.parents = []
    
    def getType(self):
        return self.type

    def setStart(self, startPoint):
        self.start = startPoint

    def setEnd(self, endPoint):
        self.end = endPoint

    def addParent(self,parent):
        self.parents.append(parent)
     

    def getParents(self):
        return self.parents

    def translate(self, dx,dy):        
        self.start.translate(dx, dy)
        self.end.translate(dx, dy)

    def getchildren(self):
        return self.children

    def addChild(self,childFold):
        self.children.append(childFold)
        childFold.addParent(self)

    def getstart(self):
        return self.start

    def getend(self):
        return self.end    
    
    def setType(self, foldtype):
        self.type = foldtype

    def isPerformed(self):
        return self.performed

    def setPerformed(self):
        self.performed = True

    def dupl(self):
        start = self.start.dupl()
        end = self.end.dupl()
        foldtype = self.type
        return Fold(start,end,foldtype)


if __name__ == '__main__':
    #global FLAG_sim
    rospy.init_node("poly_gui_bridge")
    gui = FoldingGUI(name="FoldingGUIApoorva")
    gui.simFlag = True
    gui.setGripperLimit(2)
    rospy.spin()
