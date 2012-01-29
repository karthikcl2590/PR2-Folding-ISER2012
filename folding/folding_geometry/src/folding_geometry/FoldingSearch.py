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
import util
import FoldingGUI
from numpy import *

"""
Implements search utilities for folding
"""
gui = None
robot = None

"""
 define robot actions 

table_front , table_left, table_right, table_front_left, table_front_right

"""
actions_move = ["table_front","table_left", "table_right", "table_front_left", "table_front_right"]
robot_position_XY = { "table_front": "+y" , "table_left": "-x",  "table_right":"+x", "table_front_left": "-x", "table_front_right": "+x"}

""" 
hardcoded action costs
"""
move_cost = 10.0
drag_cost = 4.0
fold_cost = 1.0
fold_sequence = []


class Action():
    def __init__(self,actionType, gripPoints, endPoints,moveDestination = "None", dragDirection = 'None', dragDistance = 'None', foldType = 'None', foldLine = 'None'):
        self.actionType = actionType
        self.gripPoints = gripPoints
        self.endPoints = endPoints
        self.moveDestination = moveDestination
        self.dragDirection = dragDirection
        self.dragDistance = dragDistance
        self.foldType = foldType
        self.foldLine = foldLine

    def get_actionType(self):
        return self.actionType
    
    def get_gripPoints(self):
        return self.gripPoints
        

    def get_endPoints(self):
        return self.endPoints

#Move specific functions
    def get_moveDestination(self):
        return self.moveDestination

#Drag specific functions

    def get_dragDirection(self):
        return self.dragDirection
    
    def get_dragDistance(self):
        return self.dragDistance
   
#Fold Specific functions
    def get_foldType(self):
        return self.foldType

    def get_foldLine(self):
        return self.foldLine

    def __str__(self):
        print self.actionType, self.get_moveDestination()
        if (self.get_actionType() == 'fold'):
            return "Action Performed:" + str(self.get_actionType())+" FoldType:"+str(self.get_foldType())+" FoldLine: "+ str(self.foldLine)
        elif (self.get_actionType() == 'move'):
            return "Action Performed:" + self.get_actionType()+" MoveDestination: " + self.get_moveDestination()
        elif (self.get_actionType() == 'drag'):
            return "Action Performed:" + str(self.get_actionType())+ "Drag Distance:" + str(self.get_dragDistance())+ "Drag Direction: " +str(self.get_dragDirection())

    

class SearchState():
    
    def __init__(self,polys=[],robotPosition='table_front',g=0.0,h = 0, actionToHere="None",parent=None,depth=0,
                 dragHistory = [],availableFolds=[], completedFolds = []):  
        self.robotPosition = robotPosition # can be "table_front","table_left", "table_right", "table_front_left", "table_front_left"
        self.polys = polys
        self.children = []
        self.depth = depth  # useful for depth limited search?
        self.g = g
        self.h = h
        self.action = actionToHere            
        self.parent = parent
        self.lastFolded = []
        self.availableFolds = availableFolds
        self.completedFolds = completedFolds
        self.dragHistory = dragHistory # stores history of drags as (direction,distance)
        
    def get_g(self):
        return self.g

    def get_cost(self):
        return self.g() + self.h()

    def get_h(self):
        return self.h

    def get_depth(self):
        return self.depth
    
    def get_polys(self):
        return self.polys

    def get_children(self):
        return self.children

    def get_availableFolds(self):
        return self.availableFolds

    def get_completedFolds(self):
        return self.completedFolds

    def robotPositionXY(self):
        return robot_position_XY[self.robotPosition]
        
    def makeChildren(self):
        # perform all moves
        current_actions_move = list(actions_move)
        current_actions_move.remove(self.robotPosition)
        for pos in []:# current_actions_move:
            newPosition = pos
            print "Move Cost" ,robot.move_cost(self.robotPosition+"_scoot",newPosition+"_scoot")
            action = Action('move',gripPoints =  [],endPoints = [], moveDestination = pos) 
            self.children.append(SearchState(polys = self.get_polys(),robotPosition=newPosition,g=self.get_g()+robot.move_cost(self.robotPosition+"_scoot",newPosition+"_scoot"), h=getHeuristic(self),actionToHere =action,
                                  parent=self,depth = self.get_depth()+1,availableFolds = list(self.availableFolds), completedFolds= list(self.completedFolds), dragHistory = list(self.dragHistory)))
            
                                             #Calculate net drag distance
        drag_x = 0
        drag_y = 0
    
        for dragDirection,distance in self.dragHistory:
            if(dragDirection == "+x"):
                drag_x += distance
            elif(dragDirection == "-x"):
                drag_x -= distance
            elif(dragDirection == "+y"):
                drag_y += distance
            elif(dragDirection == "-y"):
                drag_y -=distance
               
        print " drag x,y ", drag_x, drag_y
        # perform all available folds that are feasible                
        for fold in self.availableFolds:
            # translate fold wrt Drag History
            print "Original Fold", fold.getstart(), fold.getend()
            newFold = fold.dupl()
            newFold.translate(drag_x, drag_y)
            print "Trying fold",newFold.getstart(),newFold.getend()
            child, gripPts, endPts = simulateFold(self,newFold)   
            #print "simulate Fold on poly"
            #for poly in self.get_polys():
            #    print poly            
            if child:                
                child.action = Action(actionType = "fold",foldLine = fold,gripPoints = gripPts,endPoints = endPts, foldType  = "blue")
                child.availableFolds, child.completedFolds = updateAvailableFolds(fold,list(self.availableFolds),list(self.completedFolds))
                child.g = child.g 
                child.h = getHeuristic(child)
                child.dragHistory = list(self.dragHistory)
                self.children.append(child)
                
        # perform drags    
        """
               for d in range(10,50,10):            
               child = simulateDrag(self,d)
            if child:
                self.children.append(child)
        """
        
        for direction in self.getPossibleDragDirections(self.robotPositionXY()):
            d = 20
            while(d < 100):
                child = simulateDrag(self,d,direction)
                if child:            
                    self.children.append(child)

                    print "======= Drag Cost" , child.get_g()
                d = d + 20 
        return self.children

    def getPossibleDragDirections(self, robotPosition):
        
        return [robotPosition]
        dragList = []

        if robotPosition == '-x':
            dragList.append('-x')
        elif robotPosition == '+x':
            dragList.remove('+x')
        elif robotPosition  == '+y':
            dragList.remove('+y')
        return dragList
                    

    def getDragDistance(self):
        drag_x = 0
        drag_y = 0
        for drag,distance in self.dragHistory:
            if(self.robotPositionXY() == "+x"):
                drag_x += distance
            elif(self.robotPositionXY() == "-x"):
                drag_x -= distance
            elif(self.robotPositionXY() == "+y"):
                drag_y += distance
        return (drag_x, drag_y)

    def __eq__(self,state):
        if(self.robotPosition != state.robotPosition):
            return False
        elif len(self.polys)!=len(state.polys):
            return False
        else:            
            for poly in self.polys:
                if poly not in state.polys:
                    return False
        return True                

    def __str__(self):
        return 'len_polys: '+ str(len(self.get_polys()))+' robotposition: '+self.robotPosition+ 'actiontohere: '+ str(self.action)


def simulateFold(parent,Fold, isHeuristic = False):
    if (Fold.getType() == 'r'):
        return simulateRedFold(parent,Fold, isHeuristic)
    elif(Fold.getType() == 'b'):
        return simulateBlueFold(parent,Fold, isHeuristic)
    else:
        return False,[],[]
                                    
def simulateRedFold(parent,Fold, isHeuristic):    
    
    return False,[],[]


def simulateBlueFold(parentNode,Fold, isHeuristic):

    #raw_input("+++++++++++++++++++++++++++++ Adding a blue fold ++++++++++++++++++++++++++++")
    (start,end,foldtype) = (Fold.getstart(),Fold.getend(),Fold.getType())
    foldline = Geometry2D.DirectedLineSegment(start,end)
    #gui.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,0,255),gui.front(parentNode.get_polys()),foldline))
    if gui.legalBlueFold(foldline,parentNode.get_polys()):
        child = SearchState(polys = [],robotPosition = parentNode.robotPosition, g = parentNode.get_g()+fold_cost,actionToHere = ("fold",Fold),parent=parentNode,depth = parentNode.get_depth()+1)  
        (activeVerts,gripPts,endPts) = gui.foldAll(parentNode.get_polys(),foldline,dragAction = False,SearchNode = child)

        if(len(gripPts) == 0):
            return False,[],[]
        if isHeuristic:
            return child, gripPts, endPts
        foldColor = 'blue'
        if(parentNode.action != "None" and parentNode.action.get_actionType() ==  'drag'):
                isRedFold = True
                for gripPt in gripPts:
                    if not gripPt in parentNode.action.get_endPoints():
                        isRedFold = False
                if isRedFold:
                    foldColor = 'red'

        gripPts3D,endPts3D = gui.convertGripPts(gripPts, endPts) 
        print "gripPts for simulate Fold"
        for pt in zip(gripPts,gripPts3D):
            print pt[0],pt[1]
        #raw_input("Adding a Blue Fold")

        (canFold,costFold) = robot.feasible_fold(gripPts3D,endPts3D,parentNode.robotPosition, foldColor) #gui.can_fold(gripPts,endPts,parentNode.robotPosition) # feasible_fold(self,gripPts3D,endPts3D,parentNode.robotposition):
        
        if not canFold: # cannot fold as is because of reachability constraints             
            print "cannot reach folds"
            return False,[],[]
        else:
            child.g = costFold + parentNode.get_g()
            print "Cost of Fold" , costFold, child.get_g()
            for pt in activeVerts:
                print pt
#child.availableFolds = updateAvailableFolds(Fold,list(parentNode.availableFolds))            
            return child, gripPts, endPts
    else:
        return False,[],[]

def simulateDrag(parentNode, distance, direction):
    #print "direction", direction
    
    #raw_input("==================== Adding a Drag Fold ======================")
    foldline = gui.getOppositeTableEdgeForDrag(direction)

#def simulateDrag(parentNode,distance):
    ###Code added for drag in all directions###

    # use opposite table edge as foldline, as friction vector always points towards it
    #foldline = gui.getOppositeTableEdge(parentNode.robotPosition)
    (start,end) = (foldline.start(),foldline.end())
    if True:#gui.legalDragFold(foldline, parentNode.get_polys(), False):

        child = SearchState(polys = [],robotPosition = parentNode.robotPosition, g = parentNode.get_g()+drag_cost + distance/50,
                           actionToHere = Action(actionType = "drag",dragDirection = direction,dragDistance = distance, gripPoints = [], endPoints = []),parent=parentNode,depth = parentNode.get_depth()+1)
        (activeVerts,gripPts,endPts) = gui.foldAll(parentNode.get_polys(),foldline,dragAction = True,SearchNode = child,d = distance, direction = direction)

        if(len(gripPts) ==0):
            return False
        #print "gripPts for simulate Drag"
        gripPts3D,endPts3D = gui.convertGripPts(gripPts, endPts)
        #for pt in zip(gripPts,gripPts3D):
        #    print pt[0],pt[1]
        #raw_input("Adding a Drag Fold")

        if len(child.get_polys()) == 0:
            return False
        
        
        (canDrag, costDrag) = robot.feasible_drag(gripPts3D,distance,direction,parentNode.robotPosition)  #gui.can_drag(gripPts,direction=parentNode.robotPosition)
        if canDrag:        
            #print "Cost of Drag", costDrag
        #add current drag to parent drag history
            child.g = costDrag+parentNode.get_g()
            child.h = getHeuristic(child)
            child.action.gripPoints  = gripPts
            child.action.endPoints = endPts
            child.dragHistory = list(parentNode.dragHistory)
            child.dragHistory.append((direction, distance))
            child.availableFolds = parentNode.availableFolds
            child.completedFolds  = parentNode.completedFolds

        #if(drag_x <= 0):
            return child
        else:
            return False
    return False


### deprecated
def move(parent,direction):
    """
    takes a parent and a direction of motion, returns a new robot position
    """
    if parent.robotPosition == '+y':
        if direction == "l":
            robotPosition = "-x"
        elif direction == "r":
            robotPosition = "+x"            
    elif parent.robotPosition == "-x":
        if direction == "r":
            robotPosition = "+y"                
    elif parent.robotPosition == "+x":
        if direction == "l":
            robotPosition = "+y"
    return robotPosition


            
def getHeuristic(currentNode):    
    allFoldList = list(fold_sequence)
    completedFoldList = list(currentNode.get_completedFolds())
    for fold in completedFoldList:
        allFoldList.remove(fold)
    return sum(fold.getCost() for fold in allFoldList)

def setHeuristic(searchNode):
    print "start", searchNode
    allFoldList = list(fold_sequence)
    print allFoldList
    
    searchNode = searchNode
    for fold in allFoldList:
        child, gripPts, endPts = simulateFold(searchNode,fold, True)
        print "Child in set heuristic" , child, gripPts, endPts
        if child:
            maxDistance =float(max(Geometry2D.ptMagnitude(Geometry2D.ptDiff(pt1, pt2)) for pt1, pt2 in zip(gripPts, endPts)))
            print "maxDistance", maxDistance
            fold.setCost(((maxDistance/5)*0.0254)/0.25)
            searchNode = child
        else:
            fold.setCost(float("inf"))
            return False
    return True 
     

def addParentsOfFold(fold, foldList, completedFoldList):
    newList = []
    for parent in fold.getParents():
        if parent in foldList or parent in completedFoldList:
            continue
        else: 
            newListTemp = addParentsOfFold(parent,foldList, completedFoldList)
            newListTemp.extend(newList.append(parent))
            newList = newListTemp
    newList.append(fold)
    return newList
    

def createFoldList(availableFoldList, completedFoldList):
    foldList = list(availableFoldList)
    for fold in foldList:
        for child in fold.getChildren():
            if not child in foldList and not child in completedFoldList:
                foldList.append(addParentsOfFold(child, foldList, completedFoldList))
            
    return foldList

def updateAvailableFolds(fold,availableFoldList,completedFoldList):
    """                                                                                                                      
    Adds child nodes to fringe after current fold is performed                                                               
    """
    availableFoldList.remove(fold)
    completedFoldList.append(fold)
    for child in fold.getChildren():    
        for parent in child.getParents():
            if parent in completedFoldList:
                availableFoldList.append(child)
    return availableFoldList, completedFoldList

def goalTest(Node):
    if(len(Node.availableFolds) == 0):
        print "goaltest succeeded"
        return True

def FoldingSearch(mygui,myrobot,startpoly):
    """
    implement a uniform cost search
    """
    global gui,robot, fold_sequence
    gui = mygui
    robot = myrobot
    print "FoldingSearch called"
    print "Timing Started"
    #t = rospy.get_time() 
    # create the Fold Tree
    foldTree = gui.foldTree
    fold_sequence = gui.foldSequence
    start_availableFolds = []
    completedFolds = []
    totalNodesAdded = 0
    totalNodesExpanded = 0
    for fold in foldTree:
        start_availableFolds.append(fold)
    alreadySeen = []
    searchQueue = util.PriorityQueue()
    searchQueue.push(SearchState(polys = [gui.startpoly],robotPosition='table_front',availableFolds = start_availableFolds),0)

    print "Start Poly" , gui.startpoly
    setHeuristic(searchQueue[0])
    #raw_input()
    for fold in fold_sequence:
        print fold, fold.getCost()
    #print "searchQueue",searchQueue.heap
    t = rospy.get_time()
    
    while not searchQueue.isEmpty():
        #print searchQueue.isEmpty()
        """
        print "\nQUEUE ELEMENTS"
        for el in searchQueue.heap:
            for poly in el[1].polys:
                print poly
        """        
        
        currentState = searchQueue.pop()
        if (currentState.get_completedFolds() > 0):
            for poly in currentState.get_polys():
                if (len(poly.getShape().vertices()) <= 2):
                    print "Error Too Few vertices in currentState", poly, currentState
        print "Current State : Folds Completed" , len(currentState.get_completedFolds()), "nodes expanded",totalNodesExpanded, "nodes added", totalNodesAdded
        #print "popping currentState",currentState,"g ",currentState.get_g()
        #raw_input()
        if goalTest(currentState):
            break

        if currentState.get_g() == float("infinity"):
            continue

        if not currentState in alreadySeen:
            totalNodesExpanded+=1
            #print "CURRENTSTATE", currentState , type(currentState)
            alreadySeen.append(currentState)            
                       
            for child in currentState.makeChildren():
                for poly in child.get_polys():
                    if (len(poly.getShape().vertices()) <= 2):
                        print "Error Too Few vertices in childState", poly, currentState, child
                        raw_input("WTH IS ONE LINE")
                        #continue

                totalNodesAdded+=1
                searchQueue.push(child,child.get_g()+child.get_h())    
                #print "pushing child",child
                
                #for poly in child.get_polys():
                 #   if (len(poly.getShape().vertices()) <= 2):
                  #      print "Error Too Few vertices in childState", poly, currentState, child
                        #raw_input("WTH IS ONE LINE")

                if len(child.get_polys()) == 0:
                    raw_input("WTH IS 0") 
                #for poly in child.get_polys():
                #    print poly
        
        """
        print "searchqueue,"
        for state in searchQueue.heap:
            print state[1],
        print "\nalreadyseen"
        for state in alreadySeen:
            print state,
        print "\n"
        """
    print "Time End" , rospy.get_time() - t, "tf time", util.TfTime
    rospy.loginfo("Time end %f tf time %f", rospy.get_time() - t, util.TfTime)
    print "Nodes Expanded",totalNodesExpanded,"Nodes Pushed" , totalNodesAdded
    
    # track back to find actions
    if(len(currentState.availableFolds) > 0):
        print "Search failed!"
        sys.exit(1)
        return
    state = currentState
    actions = []
    costs = []
    states = []
    while(state.parent != None):
        #If a fold is performed immediately following a drag and gripPoints are same as endPoints, we make it a redFold.
        if(state.parent.action !='None'):
            if(state.parent.action.get_actionType() ==  'drag' and state.action.get_actionType() == 'fold'):
                isRedFold = True
                for gripPt in state.action.get_gripPoints():
                    if not gripPt in state.parent.action.get_endPoints():
                        isRedFold = False
                if isRedFold:
                    state.action.foldType = 'red'
            
        actions.append(state.action)
        states.append(state)
        costs.append(state.get_g())
        state = state.parent
        #print state.get_g(), state.action
    states.append(state)
    actions.reverse()
    states.reverse()
    print "\n\nFinished folding. actions = "
    for action,cost in zip(actions,costs):
        print action,cost
    gui.drawSimulationFolds(states)
    return states

def startFolding():   
    r = rospy.Rate(1)
    while not gui.readytoFold and not rospy.is_shutdown(): # spin until ready to fold
        r.sleep()
    print "starting folding"
    FoldingSearch(gui.startpoly)    

if __name__ == '__main__':    
    global gui
    rospy.init_node("FoldingSearch")    
    gui = FoldingGUI.FoldingGUI(name="FoldingGUI")
    gui.simFlag = True
    gui.setGripperLimit(2)
    startFolding()
    rospy.spin()

