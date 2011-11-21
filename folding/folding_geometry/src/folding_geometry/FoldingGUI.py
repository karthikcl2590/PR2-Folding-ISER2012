#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_geometry")
from shape_window.ShapeWindow import *
from shape_window import ShapeWindowUtils
from shape_window import Geometry2D
import sys
import time
import math
from numpy import *

#   The FoldingGUI is a ShapeWindow which, in addition to drawing shapes, allows you to fold them. The two types of folds you can execute are "Red Folds" and "Blue Folds".
# A blue fold is the intuitive one. You draw a line segment which bisects the polygon into two parts. The part to the left of the line segment (relative to it's direction) is mirrored about the segment, and the
# portion is placed on top.
# A red fold is identical to a blue fold, except it must be executed on the last-folded portion of cloth. This is an exception to the general rule that the robot may not grasp a particular layer of the cloth. As the
# portion has just been folded, it is still in the robot's grippers, and may be grasped even if there is a layer underneath,.
class FoldingGUI(ShapeWindow):
    
    def initExtended(self):
        self.fold_callback = False
        self.unfreezeMouse()
        self.setGripSize(10)
        self.wideGripFlag = False
        self.drawingModeFlag = False
        self.numGrippers = 2
        self.blueStart = Geometry2D.Point(0,0)
        self.blueEnd = Geometry2D.Point(0,0)
        self.redStart = Geometry2D.Point(0,0)
        self.redEnd = Geometry2D.Point(0,0)
        self.midStart = Geometry2D.Point(0,0)
        self.midEnd = Geometry2D.Point(0,0)
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

    
    def initOverlay(self):
        gripSlider = CVSlider(origin=Geometry2D.Point(150,450),valMin=0,valMax=200,valueGetter=self.getGripSize,valueSetter=self.setGripSize,sliderWidth=150)
        self.addOverlay(gripSlider)
        wideGripButton = CVOnOffButton(text="WG?",bottomLeft=Geometry2D.Point(50,450),valueGetter=self.wideGrip,valueToggle=self.toggleWideGrip)
        self.addOverlay(wideGripButton)
        drawPolyButton = CVOnOffButton(text="Draw",bottomLeft=Geometry2D.Point(400,450),valueGetter=self.drawingMode,valueToggle=self.toggleDrawingMode)
        self.addOverlay(drawPolyButton)
        clearShapesButton = CVButton(text="CLEAR",bottomLeft=Geometry2D.Point(50,400), onClick=self.clearShapes)
        self.addOverlay(clearShapesButton)
        #topViewLabel = CVButton(text="Top View", bottomLeft=Geometry2D.Point(100,50))
        #self.addOverlay(topViewLabel)
        gripVisualizer = CVVisualizer(origin=Geometry2D.Point(450,50),valueGetter=self.getGripSize,color=Colors.BLUE,displayCondition = self.wideGrip)
        self.addOverlay(gripVisualizer)
                
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
            return self.polyDrawer(event,x,y,flags,param)
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
        if self.legalBlueFold(foldline):
            (activeVerts,gripPts) = self.foldAll(self.getPolys(),foldline,True)
            for v in activeVerts:
                self.highlightPt(v)
            if self.wideGrip():
                for g in gripPts:
                    self.drawGripper(g)
            print "Num grippers required: %d"%len(gripPts)
            if not self.adjust_foldline():
                return
            adjusted_foldline = self.echoFold(foldline,gripPts,False)
            print "INITIAL FOLDLINE WAS: %s"%foldline
            print "RECEIVED ADJUSTED FOLDLINE: %s"%adjusted_foldline
            if self.legalBlueFold(adjusted_foldline):
                self.clearShapes()
                for poly in self.lastState:
                    self.addCVShape(poly)
                #Now redo the folding but don't echo it                                                                                    
                self.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,255,255),self.front(),foldline))
                (activeVerts,gripPts) = self.foldAll(self.getPolys(),adjusted_foldline,True)
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


    def executeBlueFold(self):
        self.lastState = []
        print "New Blue Fold"
        for poly in self.getPolys():
            self.lastState.append(poly.dupl())
            print "Poly is %s"%(poly)
        foldline = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
        self.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,0,255),self.front(),foldline))
        if self.legalBlueFold(foldline):
            (activeVerts,gripPts) = self.foldAll(self.getPolys(),foldline,False)
            for v in activeVerts:
                self.highlightPt(v)
            if self.wideGrip():
                for g in gripPts:
                    self.drawGripper(g)
            print "Num grippers required: %d"%len(gripPts)
            if not self.adjust_foldline():
                return
            adjusted_foldline = self.echoFold(foldline,gripPts,False)
            print "INITIAL FOLDLINE WAS: %s"%foldline
            print "RECEIVED ADJUSTED FOLDLINE: %s"%adjusted_foldline
            if self.legalBlueFold(adjusted_foldline):
                self.clearShapes()
                for poly in self.lastState:
                    self.addCVShape(poly)
                #Now redo the folding but don't echo it
                self.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,255,255),self.front(),foldline))
                (activeVerts,gripPts) = self.foldAll(self.getPolys(),adjusted_foldline,False)
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
        
    def legalDragFold(self,foldline):
        hanging = [cvshape for cvshape in self.getPolys() if cvshape.isHang()]
        if len(hanging) > 0:
            dragAxis = NULL
            if self.dragDirection == '-x' or self.dragDirection == '+x':
                dragAxis = Geometry2D.LineSegment(Geometry2D.Point(0.0,0), Geometry2D.Point(1.0,0))
            elif self.dragDirection =='-y' or self.dragDirection == '+y':
                dragAxis = Geometry2D.LineSegment(Geometry2D.Point(0.0, 0), Geometry2D.Point(0,1.0)) 
            angle = math.fabs(Geometry2D.angleBetweenLines(foldline, dragaxis)) 
            if(angle == 0 or angle == 180):
                if(direction == '-x'):
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
            return True;
        elif self.isAnyPolyWithinHang(foldline,self.getPolys()):
            return False
        return True
        
    def legalBlueFold(self,foldline):
        if not self.legalDragFold(foldline):
            return False
        for shape in [cvshape.getShape() for cvshape in self.getPolys()]:
            if shape.containsExclusive(foldline.start()) or shape.containsExclusive(foldline.end()):
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
        if len(self.lastFolded) == 0:
            return False
        for line in self.lastFoldline:
            if line.start() and line.end() and foldline.isRightOf(line.start()) or foldline.isRightOf(line.end()):
                print "Line is right of the last foldline"
                return False
        for shape in [cvshape.getShape() for cvshape in self.lastFolded]:
            if shape.containsExclusive(foldline.start()) or shape.containsExclusive(foldline.end()):
                return False
        return True
    

    def foldAll(self,polys,foldline,isHang):
        self.flushQueue()
        [toFold,toNotFold] = self.getFoldedRegion(polys,foldline)
        self.toFold = toFold
        self.toNotFold = toNotFold
        self.lastFolded = []
        self.lastFoldline = []
        activeVerts = []
        for poly in sorted(polys,key=lambda p: p.getHeight(), reverse=True):
            if isHang:
                #newActive = self.fold(poly,foldline,toFold)
                newActive = self.drag(poly,foldline,toFold,self.dragDirection,self.dragDistance)
            else:
                newActive = self.fold(poly,foldline,toFold)
            for v in newActive:
                if not v in activeVerts:
                    activeVerts.append(v)
        gripPoints = self.gripPoints(activeVerts)
        if not self.gripperLimit or len(gripPoints) <= self.gripperLimit:
            self.executeQueue()
        else:
            self.flushQueue()
            print "Error: requires %d grippers"%len(gripPoints)
        return (activeVerts, gripPoints)
                
    def fold(self, poly, foldline, toFold):
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
            print "Ploygon in fold"#,p,foldline,toFold
            #print "randpt",p.randPt()

            if p != False:
                if len([folded for folded in toFold if folded.containsExclusive(p.randPt())]) > 0:
                #if len([x for x in toFold if x.contains(p.randPt())]) > 0:
                #if foldline.isRightOf(p.center()):
                    for pt in p.vertices():
                        if self.isActive(pt,poly,foldline) and not poly.isHang():
                            active.append(pt)
                    drawp = Geometry2D.mirrorPoly(p,foldline)
                    drawh = self.front()
                    drawc = Colors.complementCV(color)
                    cvpoly = CVPolygon(drawc,drawh,drawp)
                    cvpoly.setHang(poly.isHang())
                    self.lastFolded.append(cvpoly)
                else:
                    drawp = p
                    drawh = height
                    drawc = color
                    cvpoly = CVPolygon(drawc,drawh,drawp)
                    cvpoly.setHang(poly.isHang())
                self.queueAddShape(cvpoly)
        self.queueRemoveShape(poly)
        return active



    def drag(self, poly, foldline, toFold,direction, distance):
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
            print "Ploygon in fold"#,p,foldline,toFold                                                                                          
            #print "randpt",p.randPt()                                                                                                          

            if p != False:
                if len([folded for folded in toFold if folded.containsExclusive(p.randPt())]) > 0:
                #if len([x for x in toFold if x.contains(p.randPt())]) > 0:
                #if foldline.isRightOf(p.center()):
                    for pt in p.vertices():
                        if self.isActive(pt,poly,foldline):
                            active.append(pt)
                    drawp = Geometry2D.movePoly(p,'+y',50)
                    drawh = height
                    drawc = color
                    cvpoly = CVPolygon(drawc,drawh,drawp)
                    cvpoly.setHang(True)
                    self.lastFolded.append(cvpoly)
                else:
                    drawp = Geometry2D.movePoly(p,'+y',50)
                    drawh = height
                    drawc = color
                    cvpoly = CVPolygon(drawc,drawh,drawp)
                    cvpoly.setHang(poly.isHang())
                self.queueAddShape(cvpoly)
        self.queueRemoveShape(poly)
        return active

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
    
    def executeQueue(self):
        for el in self.removeQueue:
            self.removeCVShape(el)
        for el in self.addQueue:
            self.addCVShape(el)
        self.flushQueue()
        
# determine if polygon lies exlusively in the hang part of the fold.
    def isAnyPolyWithinHang(self,foldline,cvpolys):
        for cvs in [poly.getShape() for poly in cvpolys]:     
            for vert in cvs.vertices():
                if foldline.isRightOf(vert) or foldline.contains(vert):
                    continue
                else: 
                    return False
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
        
    def isActive(self,vertex,cvShape,foldline):
        if foldline.isLeftOf(vertex) or foldline.contains(vertex):
            return False
        elif self.isCovered(vertex,cvShape):
            return False
        elif self.isSupported(vertex,cvShape,foldline):
            return False
        concave = []
        for cvs in self.getPolys():
            for vert in cvs.getShape().concaveVertices():
                concave.append(vert)
        if len([vert2 for vert2 in concave if Geometry2D.distance(vertex,vert2) < 2])>0:
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
        polys = [cvpoly.getShape() for cvpoly in cvpolys]
        toFold = []
        toNotFold = []
        newpolys = []
        rmpolys = []
        for poly in polys:
            halves = Geometry2D.bisectPoly(poly,foldline)
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
    
    def setFoldCallback(self,callback):
        self.fold_callback = callback
    
    def echoFold(self,foldline,activeVerts,red):
        if self.fold_callback:
            #thread.start_new_thread(self.fold_callback,(foldline,activeVerts,red))
            return self.fold_callback(foldline,activeVerts,red)
            print "Finished echoing"



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
         self.executeBlueFold()
         time.sleep(2.5)
         
         
         #Thirds
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(1.0/4.0)
         self.blueEnd = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(1.0/4.0)
         left_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         
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
         
         
         #Thirds
         self.blueStart = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(1.0/4.0 - 0.05)
         self.blueEnd = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(1.0/4.0 - 0.05)
         left_third = Geometry2D.DirectedLineSegment(self.blueStart,self.blueEnd)
         self.executeBlueFold()
         
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
         #self.executeBlueFold()
         #time.sleep(2.5)
         #Thirds
         self.blueEnd = Geometry2D.DirectedLineSegment(bottom_left,bottom_right).extrapolate(3.0/4.0 + 0.05)
         self.blueStart = Geometry2D.DirectedLineSegment(top_left,top_right).extrapolate(3.0/4.0 + 0.05)
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
         #self.setGripSize(fold.length()*1.3/2)
         self.setGripSize(fold.length()*0.8/2)
         fold.expand(1.5)
         self.blueEnd = fold.start()
         self.blueStart = fold.end()
         self.executeBlueFold()
         time.sleep(2.5)
         
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
        [bl,tl,tr,br] = self.getPolys()[0].getShape().vertices()
        height = max(Geometry2D.distance(bl,tl),Geometry2D.distance(br,tr))
        width = max(Geometry2D.distance(tl,tr),Geometry2D.distance(bl,br))
        
        
        #Fold in half
        l_ctr = Geometry2D.LineSegment(bl,tl).center()
        r_ctr = Geometry2D.LineSegment(br,tr).center()
        blueEnd = Geometry2D.LineSegment(bl,tl).center()
        blueStart = Geometry2D.LineSegment(br,tr).center()
        blueFold = Geometry2D.DirectedLineSegment(blueStart,blueEnd)
        #blueFold.expand(0.05)
        self.blueStart = blueFold.start()
        self.blueEnd = blueFold.end()
        self.dragStart = blueFold.start()
        self.dragEnd = blueFold.end()
        self.dragDistance = bl.y() - l_ctr.y()
        self.dragDirection = '+y'
        print "Blue Start %s"%(self.blueStart)
        print "Blue End %s"%(self.blueEnd)
        self.executeDragFold()
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
