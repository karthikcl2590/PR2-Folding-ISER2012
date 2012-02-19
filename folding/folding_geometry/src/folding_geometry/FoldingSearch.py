#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_geometry")
from shape_window.ShapeWindow import *
from shape_window import ShapeWindowUtils
from shape_window import Geometry2D
import sys, cProfile
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
actions_move = ["table_front"]# "table_front_left", "table_left"]#, "table_front_right"]#, "table_left","table_front_right","table_right"]
robot_position_XY = { "table_front": "+y" , "table_left": "-x",  "table_right":"+x", "table_front_left": "-x", "table_front_right": "+x"}

""" 
hardcoded action costs
"""
move_cost = 10.0
drag_cost = 4.0
fold_cost = 1.0
fold_sequence = []

DEBUG = False
DEBUG_CHILDREN = False
PROFILE = False

class Action():
    def __init__(self,actionType, gripPoints, endPoints,moveDestination = None, dragDirection = None, dragDistance = None, foldType = None, foldLine = None):
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

    # Move specific functions
    def get_moveDestination(self):
        return self.moveDestination

    # Drag specific functions

    def get_dragDirection(self):
        return self.dragDirection
    
    def get_dragDistance(self):
        return self.dragDistance
   
    # Fold Specific functions
    def get_foldType(self):
        return self.foldType

    def get_foldLine(self):
        return self.foldLine

    def __str__(self):
        print self.actionType, self.get_moveDestination()
        stringGrip = ""
        if (self.get_actionType() == 'fold'):
            for gripPt in self.get_gripPoints():
                stringGrip += "\n" + str(gripPt)
            stringGrip += "\nEndPoints\n"
            for endPt in self.get_endPoints():
                stringGrip += "\n" + str(endPt)
            return "Action Performed:" + str(self.get_actionType())+" FoldType:"+str(self.get_foldType())+" FoldLine: "+ str(self.foldLine) + stringGrip 
        elif (self.get_actionType() == 'move'):
            return "Action Performed:" + self.get_actionType()+" MoveDestination: " + self.get_moveDestination()
        elif (self.get_actionType() == 'drag'):
            stringGrip = ""
            for gripPt in self.get_gripPoints():
                stringGrip += "\n" + str(gripPt)
            return "Action Performed:" + str(self.get_actionType())+ "Drag Distance:" + str(self.get_dragDistance())+ "Drag Direction: " +str(self.get_dragDirection())+ stringGrip

    

class SearchState():
    
    def __init__(self,polys,dragHistory,availableFolds,completedFolds,g = 0.0, h = 0, action=None,parent=None,depth=0, robotPosition = 'table_front'):  
        self.robotPosition = robotPosition # can be "table_front","table_left", "table_right", "table_front_left", "table_front_left"
        self.polys = polys
        self.children = []
        self.depth = depth  # useful for depth limited search?
        self.g = g
        self.h = h
        self.action = action
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
        """
        make children of current node
        
        """
        numMove = 0
        numFold = 0
        numDrag = 0

        # add all move actions
        current_actions_move = list(actions_move)
        current_actions_move.remove(self.robotPosition)        
        for pos in current_actions_move:
            newPosition = pos
#            if DEBUG:
                #print "===== Move Cost =====" ,robot.move_cost(self.robotPosition+"_scoot",newPosition+"_scoot"), "Moving from ", self.robotPosition, "Moving to ", pos
            action = Action('move',gripPoints =  [],endPoints = [], moveDestination = pos)
            numMove+=1

            child  = SearchState(polys = self.get_polys(),
                                             robotPosition=newPosition,
                                             g=self.get_g()+robot.move_cost(self.robotPosition+"_scoot",newPosition+"_scoot"), 
                                             h=0,action=action,
                                             parent=self,
                                             depth = self.get_depth()+1,
                                             availableFolds = list(self.availableFolds), completedFolds= list(self.completedFolds), 
                                             dragHistory = list(self.dragHistory))

            child.h =getHeuristic(child)
            self.children.append(child)
        
        #Calculate net drag distance
        
        drag_x, drag_y = self.getDragDistance() 
       
        if DEBUG:
            print "Net Drag Distance x: %d, y: %d "%(drag_x, drag_y)

        # perform all available folds that are feasible                
        for fold in self.availableFolds:

            # translate fold w.r.t Drag History
            transFold = fold.dupl()
            transFold.translate(drag_x, drag_y)
            if DEBUG_CHILDREN:
                print "Original Fold", fold.getstart(), fold.getend()
                print "Fold After Translation", transFold.getstart(),transFold.getend()
                
            child, gripPts, endPts = simulateFold(self,fold,transFold)   
            
            if child:
                numFold+=1
                child.action = Action(actionType = "fold",foldLine = fold,gripPoints = gripPts,endPoints = endPts, foldType  = "blue")
                child.availableFolds, child.completedFolds = updateAvailableFolds(fold,list(self.availableFolds),list(self.completedFolds))                       
                child.g = child.g 
                child.h = getHeuristic(child)
                child.dragHistory = list(self.dragHistory)
                self.children.append(child)
                
        # perform drags
        
        for direction in self.getPossibleDragDirections(self.robotPositionXY()):
            d = 20
            while(d < 100):
                child = simulateDrag(self,d,direction)
                if child: 
                    numDrag+=1
                    self.children.append(child)
                else:
                    break
                d = d + 20 
            if DEBUG_CHILDREN:
                "Total Number of drags Performed  %d"%(d)
        if DEBUG_CHILDREN:
            print "Total Number of Children Added for Current Node: %d NumMoves= %d , NumFolds= %d, NumDrags= %d"%(len(self.children), numMove, numFold, numDrag)
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
            if(drag == "+x"):
                drag_x += distance
            elif(drag == "-x"):
                drag_x -= distance
            elif(drag == "+y"):
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
            for poly in state.polys:
                if poly not in self.polys:
                    return False
        return True                

    def __str__(self):
        return 'Current Length of Polys: '+ str(len(self.get_polys()))+' robotposition: '+self.robotPosition+str(self.action)


def simulateFold(parent,Fold,transFold = None, isHeuristic = False):
    if (Fold.getType() == 'r'):
        return simulateRedFold(parent,Fold,isHeuristic)
    elif(Fold.getType() == 'b'):
        return simulateBlueFold(parent,Fold,transFold,isHeuristic)
    else:
        return False,[],[]
                                    
def simulateRedFold(parent,Fold, isHeuristic):    
    
    return False,[],[]


def simulateBlueFold(parentNode,Fold,transFold,isHeuristic):

    """
    Simulates the current Blue Fold
    """
    if DEBUG:
        print "+++++++++++ Adding a blue fold +++++++++++++"
    
    drag_x, drag_y = parentNode.getDragDistance()
    
    newFold = Fold.dupl()
    newFold.translate(drag_x, drag_y)
    
    if DEBUG:
        print "Original Fold", Fold.getstart(), Fold.getend()
        print "Fold After Translation", newFold.getstart(),newFold.getend()
    
    (start,end,foldtype) = (newFold.getstart(),newFold.getend(),newFold.getType())
    foldline = Geometry2D.DirectedLineSegment(start,end)

    if gui.legalBlueFold(foldline,parentNode.get_polys()):
        child = SearchState(polys = [],robotPosition = parentNode.robotPosition, 
                            g = parentNode.get_g(),action = ("fold",Fold),
                            parent=parentNode,depth = parentNode.get_depth()+1,
                            availableFolds = [], completedFolds = [], dragHistory = [])  
        (activeVerts,gripPts,endPts) = gui.foldAll(parentNode.get_polys(),foldline,dragAction = False,SearchNode = child)

        if(len(gripPts) == 0):
            return False,[],[]
        if isHeuristic:
            return child, gripPts, endPts
        foldColor = 'blue'
        if(parentNode.action != None and parentNode.action.get_actionType() ==  'drag'):
                isRedFold = True
                for gripPt in gripPts:
                    if not gripPt in parentNode.action.get_endPoints():
                        isRedFold = False
                if isRedFold:
                    foldColor = 'red'

        gripPts3D,endPts3D = gui.convertGripPts(gripPts, endPts) 

        if DEBUG:
            print "GripPoints are:\n"
            for pt in zip(gripPts,gripPts3D):
                print pt[0],pt[1],"\n"

        (canFold,costFold) = robot.feasible_fold(gripPts3D,endPts3D,parentNode.robotPosition, foldColor)
        if (costFold == float("inf")) and DEBUG:
            for foldN in parentNode.get_availableFolds():
                print foldN.getstart(), foldN.getend()
            print "completed folds"
            for foldC in parentNode.get_completedFolds():
                print foldC.getstart(), foldC.getend()
        if not canFold :             
            print "Incomplete Fold : Can't reach all active vertices"
            return False,[],[]
        elif not costFold==float("inf"):
            child.g = costFold + parentNode.get_g()
            #print "===== Cost of Fold =====" , costFold, "Total Node Cost", child.g , "ParentNode Cost", parentNode.get_g()
            for pt in gripPts:
                print pt
            print "\n\n\n\n"        
            return child, gripPts, endPts
        else:
            return False,[],[]
    else:
        return False,[],[]

def simulateDrag(parentNode, distance, direction, isHeuristic  = False):
    foldline = gui.getOppositeTableEdgeForDrag(direction)
    (start,end) = (foldline.start(),foldline.end())
    if True:#gui.legalDragFold(foldline, parentNode.get_polys(), False):

        child = SearchState(polys = [],robotPosition = parentNode.robotPosition, g = parentNode.get_g(),
                           action = Action(actionType = "drag",dragDirection = direction,dragDistance = distance, gripPoints = [], endPoints = []),parent=parentNode,depth = parentNode.get_depth()+1, availableFolds = [], completedFolds = [], dragHistory = [])
        (activeVerts,gripPts,endPts) = gui.foldAll(parentNode.get_polys(),foldline,dragAction = True,SearchNode = child,d = distance, direction = direction)

        if(len(gripPts) ==0):
            if DEBUG_CHILDREN:
                print "Failed because of gripPoints" , len(gripPts)
            return False        
        #print "gripPts for simulate Drag"
        gripPts3D,endPts3D = gui.convertGripPts(gripPts, endPts)
        #for pt in zip(gripPts,gripPts3D):
        #    print pt[0],pt[1]
        #raw_input("Adding a Drag Fold")

        if isHeuristic:
            child.gripPts = gripPts
            return child

        if len(child.get_polys()) == 0:
            return False
        
        if not isHeuristic:
            (canDrag, costDrag) = robot.feasible_drag(gripPts3D,distance,direction,parentNode.robotPosition)
            if canDrag and not (costDrag == float("inf")):        
            #print "========Cost of Drag=======", costDrag, "distance", distance
        #add current drag to parent drag history
                child.g = costDrag+parentNode.get_g()
                child.availableFolds = list(parentNode.availableFolds)
                child.completedFolds  = list(parentNode.completedFolds)
                child.dragHistory = list(parentNode.dragHistory)
                child.dragHistory.append((direction, distance))
                child.h = getHeuristic(child)
                child.action.gripPoints  = gripPts
                child.action.endPoints = endPts

        #if(drag_x <= 0):
                return child
        else:
            if DEBUG_CHILDREN:
                print "Failed due to inf cost"
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
    #print "In Heuristic"
    #return 0
    allFoldList = list(fold_sequence)
    completedFoldList = list(currentNode.get_completedFolds())
    availableFoldList = list(currentNode.get_availableFolds())
    #print "length of completed FoldList before feasibility Check", len(completedFoldList)
    (drag_x, drag_y) = currentNode.getDragDistance()
    
    #for fold in availableFoldList:        
    """
    transFold = fold.dupl()
    transFold.translate(drag_x, drag_y)
    child, gripPts, endPts = simulateFold(currentNode, fold, transFold)
    if child:
    #print "Fold Made available advantage"
    """
    #    completedFoldList.append(fold)
        
    for fold in completedFoldList:
        allFoldList.remove(fold)
    #print "length of completed FoldList after feasibility Check", len(completedFoldList)
    h= sum((fold.getCost()) for fold in allFoldList)
    # if there is an infeasible fold, add a 3 second penalty
    infeasibleFold = False
    for fold in availableFoldList:        
        transFold = fold.dupl()
        transFold.translate(drag_x,drag_y)
        child,gripPts,endPts = simulateFold(currentNode,fold,transFold)
        if not child:
            infeasibleFold = True
            break
    if infeasibleFold:
        h = h+3
    #print "end of heuristic"
    return h

def setHeuristic(searchNode):
    print "start", searchNode
    allFoldList = list(fold_sequence)
    print allFoldList
    
    searchNode = searchNode
    for fold in allFoldList:
        print "\n\n\n\n\n\ New SearchNode"
        child2 = simulateDrag(searchNode,10, '-x', True)
        if child2:
            print "\n\n\n\n polys of drag \n\n\n"
            for poly in child2.get_polys():
                gui.addPropCVShape(poly)
                print poly
            
            print "In drag: drawing grippers"
            for g in child2.gripPts:
                gui.drawGripper(g)
            
            raw_input()
            gui.clearProposed()

        child, gripPts, endPts = simulateFold(searchNode,fold,transFold = fold,isHeuristic= True)
        
        #print "Child in set heuristic" , child, gripPts, endPts
        if child:
            print "Polys after fold"
            for poly in child.get_polys():
                gui.addPropCVShape(poly)
                print poly
            raw_input("In fold")
    
        #print "Child in set heuristic" , child, gripPts, endPts
            gui.clearProposed()
            maxDistance = float(max(Geometry2D.ptMagnitude(Geometry2D.ptDiff(pt1, pt2)) for pt1, pt2 in zip(gripPts, endPts)))   
            print "Current GripPoint"
            h = 3 + (((maxDistance/util.scale_factor)/0.25))*4 
            fold.setCost(h)
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
    #print "CALLED WITH availableFoldList = ",availableFoldList, " completed fold list", completedFoldList, "fold", fold
    #print "children", len(fold.getChildren())
    #raw_input()
    availableFoldList.remove(fold)
    completedFoldList.append(fold)
    for child in fold.getChildren():    
        allParentsAdded = True
        for parent in child.getParents():
            #print "parent is in child.getParents(). parent = ",parent
            if not parent in completedFoldList:
             #   print "all parents added = false"
                allParentsAdded = False

        if allParentsAdded and not child in availableFoldList:
            #print "appending child to available list"
            availableFoldList.append(child)
    return availableFoldList, completedFoldList

def goalTest(Node):
    if(len(Node.completedFolds) == len(fold_sequence)):
        return True
    #if(len(Node.availableFolds) == 0):
     #   print "Available Folds == 0" 
        #raw_input()
        #if(len(Node.completedFolds) == len(fold_sequence)):
         #   print "goaltest succeeded"
      #  return True

#def FoldingSearch(mygui,myrobot,startpoly):
#    if (PROFILE):
#       cProfile.runctx('FoldingSearch2(mygui,myrobot,startpoly)',globals(),locals(),'/home/apoorvas/apoorvas_sandbox/PR2-Towel-Folding/folding/folding_geometry/data/FoldProfiled')
#    else:
#       FoldingSearch2(mygui,myrobot,startpoly)

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
    maxCompletedFoldsPopped = 0
    maxCompletedFoldsPushed = 0
    for fold in foldTree:
        start_availableFolds.append(fold)
    alreadySeen = []
    searchQueue = util.PriorityQueue()
    searchQueue.push(SearchState(polys = [gui.startpoly],robotPosition='table_front',availableFolds = start_availableFolds, completedFolds = [], dragHistory = []),0)

    print "Start Poly" , gui.startpoly
    setHeuristic(searchQueue[0])
    #raw_input()
    for fold in fold_sequence:
        print fold, fold.getCost()
    raw_input()
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
        if (len(currentState.get_completedFolds()) > 0):
            for poly in currentState.get_polys():
                if (len(poly.getShape().vertices()) <= 2):
                    print "Error Too Few vertices in currentState", poly, currentState

        maxCompletedFoldsPopped = max(len(currentState.get_completedFolds()), maxCompletedFoldsPopped)
        if goalTest(currentState):
            break

        if currentState.get_g() == float("infinity"):
            continue

        if not currentState in alreadySeen:
            if DEBUG_CHILDREN:
                print "\n\n\n -------------------------------------------------------------------------------------------------------------------------"
                print "++++ Current State : Folds Completed" , len(currentState.get_completedFolds()), "current State", currentState, "g ", currentState.get_g(), " h\
 ", currentState.get_h(), "g+h: ", currentState.get_g()+ currentState.get_h()
                print "nodes expanded",totalNodesExpanded, "nodes added", totalNodesAdded, "Max CompletedFoldsPopped", maxCompletedFoldsPopped, "MaxCompletedFoldsPushed", maxCompletedFoldsPushed
                print "Completed Folds" #currentState.get_completedFolds(), len(currentState.availableFolds)                                                                                                                                     
                for fold in currentState.get_completedFolds():
                    print fold

                print "Available Folds"
                for fold in currentState.get_availableFolds():
                    print fold

                drag_x, drag_y = currentState.getDragDistance()
                print "\n\nNet Drag Distance x: %d, y: %d "%(drag_x, drag_y)


                if currentState.parent!=None:
                    print "\n\nParent Node", currentState.parent
                print "--------------------------------------------------------------------------------------------------------------------------\n\n\n"

            totalNodesExpanded+=1
            
            #print "CURRENTSTATE", currentState , type(currentState)
            alreadySeen.append(currentState)            
                       
            for child in currentState.makeChildren():
                maxCompletedFoldsPushed = max(len(child.get_completedFolds()), maxCompletedFoldsPushed)

                for poly in child.get_polys():
                    if (len(poly.getShape().vertices()) <= 2):
                        print "Error Too Few vertices in childState", poly, currentState, child
                        raw_input("WTH IS ONE LINE")
                        #continue

                totalNodesAdded+=1
                searchQueue.push(child,child.get_g()+child.get_h())    

                if len(child.get_polys()) == 0:
                    raw_input("WTH IS 0") 
                        
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
    if not (len(currentState.get_completedFolds()) == len(fold_sequence)): #currentState.availableFolds) > 0):
        print "Search failed!"
        sys.exit(1)
        return
    state = currentState
    actions = []
    costs = []
    states = []
    while(state.parent != None):
        #If a fold is performed immediately following a drag and gripPoints are same as endPoints, we make it a redFold.
        if(state.parent.action !=None):
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
    costs.reverse()
    states.reverse()
    print "\n\nFinished folding. actions = "
    for action,cost in zip(actions,costs):
        print action,cost
    gui.drawSimulationFolds(states)
    return states

def startFolding():   
    cProfile.run('foo()')

def foo():
    print "hi"

def beginFolding():
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
    #cProfiler.run('startFolding()', 'FoldingProfiled')
    rospy.spin()

