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
import FoldingGUIApoorva
from numpy import *

"""
Implements search utilities for folding
"""
gui = None

"""
 define robot actions
"""
actions_move = {"+y":["r","l"],
                "+x":["l"],
                "-x":["r"]}#["r"]}

""" 
hardcoded action costs
"""
move_cost = 10.0
drag_cost = 4.0
fold_cost = 1.0

class SearchState():
    
    def __init__(self,polys=[],robotPosition='+y',g=0.0,actionToHere="None",parent=None,depth=0,
                 dragHistory = [],availableFolds=[], completedFolds = []):  
        self.robotPosition = robotPosition # can be +y,+x,-x
        self.polys = polys
        self.children = []
        self.depth = depth  # useful for depth limited search?
        self.g = g
        self.actionToHere = actionToHere            
        self.parents = parents
        self.linkedParent = null
        self.lastFolded = []
        self.availableFolds = availableFolds
        self.completedFolds = completedFolds
        self.dragHistory = dragHistory # stores history of drags as (direction,distance)
        
    def get_g(self):
        return self.g

    def get_depth(self):
        return self.depth
    
    def get_polys(self):
        return self.polys

    def get_children(self):
        return self.children

    def get_parents(self):
        return self.parents

    def get_linkedParent(self):
        return self.linkedParent

    def get_availableFolds(self):
        return self.availableFolds

    def makeChildren(self):
        # perform all moves
        
        for action in actions_move[self.robotPosition]:
            newPosition = move(self,action)
            self.children.append(SearchState(polys = self.get_polys(),robotPosition=newPosition,g=self.get_g()+move_cost,actionToHere = ("move",action),
                                  linkedParent=self,parents = depth = self.get_depth()+1,availableFolds = list(self.availableFolds),completedFolds= list(self.completedFolds), dragHistory = list(self.dragHistory)))

                                             #Calculate net drag distance
        drag_x = 0
        drag_y = 0
    
        for drag,distance in self.dragHistory:
            if(self.robotPosition == "+x"):
                drag_x += distance
            elif(self.robotPosition == "-x"):
                drag_x -= distance
            elif(self.robotPosition == "+y"):
                drag_y += distance
               
        print " drag x,y ", drag_x, drag_y
        # perform all available folds that are feasible                
        for fold in self.availableFolds:
            # translate fold wrt Drag History
            print "Original Fold", fold.getstart(), fold.getend()
            newFold = fold.dupl()
            newFold.translate(drag_x, drag_y)
            print "Trying fold",newFold.getstart(),newFold.getend()
            child = simulateFold(self,newFold)   
            #print "simulate Fold on poly"
            #for poly in self.get_polys():
            #    print poly            
            if child:                
                child.actionToHere = ("fold",fold)
                child.availableFolds = updateAvailableFolds(fold,list(self.availableFolds))
                child.completedFolds = child.completedFolds.append(fold)
                child.dragHistory = list(self.dragHistory)
                self.children.append(child)
                
        # perform drags    
        """
               for d in range(10,50,10):            
               child = simulateDrag(self,d)
            if child:
                self.children.append(child)
        """
        d = 79
        while(d < 80):
            child = simulateDrag(self,d)
            if child:            
                self.children.append(child)
            d = d + 80 
        return self.children

    def getDragDistance(self):
        drag_x = 0
        drag_y = 0
        for drag,distance in self.dragHistory:
            if(self.robotPosition == "+x"):
                drag_x += distance
            elif(self.robotPosition == "-x"):
                drag_x -= distance
            elif(self.robotPosition == "+y"):
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
        return 'len_polys: '+ str(len(self.get_polys()))+' robotposition: '+self.robotPosition+ 'actiontohere: '+self.actionToHere[0]



def simulateFold(parent,Fold):
    if (Fold.getType() == 'r'):
        return simulateRedFold(parent,Fold)
    elif(Fold.getType() == 'b'):
        return simulateBlueFold(parent,Fold)
    else:
        return False
                                    
def simulateRedFold(parent,Fold):    
    
    return False

def simulateBlueFold(parentNode,Fold):
    (start,end,foldtype) = (Fold.getstart(),Fold.getend(),Fold.getType())
    foldline = Geometry2D.DirectedLineSegment(start,end)
    #gui.addTempCVShape(CVDirectedLineSegment(cv.RGB(0,0,255),gui.front(parentNode.get_polys()),foldline))
    if gui.legalBlueFold(foldline,parentNode.get_polys()):
        child = SearchState(polys = [],robotPosition = parentNode.robotPosition, g = parentNode.get_g()+fold_cost,actionToHere = ("fold",Fold),parent=parentNode,depth = parentNode.get_depth()+1)  
        (activeVerts,gripPts,endPts) = gui.foldAll(parentNode.get_polys(),foldline,dragAction = False,SearchNode = child)        
        (canFold,unreachablePts) = gui.can_fold(gripPts,endPts,parentNode.robotPosition)
        if not canFold: # cannot fold as is because of reachability constraints             
            print "cannot reach folds"
            for pt in unreachablePts:
                print pt
            return False
        else:
            print "can fold"
            for pt in activeVerts:
                print pt
#child.availableFolds = updateAvailableFolds(Fold,list(parentNode.availableFolds))            
            return child
    else:
        return False

def simulateDrag(parentNode,distance):
    # use opposite table edge as foldline, as friction vector always points towards it
    foldline = gui.getOppositeTableEdge(parentNode.robotPosition)
    (start,end) = (foldline.start(),foldline.end())
    if True:#gui.legalDragFold(foldline, parentNode.get_polys(), False):

        child = SearchState(polys = [],robotPosition = parentNode.robotPosition, g = parentNode.get_g()+drag_cost + distance/50,
                           actionToHere = ("drag",(parentNode.robotPosition,distance)),parent=parentNode,depth = parentNode.get_depth()+1)
        (activeVerts,gripPts,endPts) = gui.foldAll(parentNode.get_polys(),foldline,dragAction = True,SearchNode = child,d = distance, direction = parentNode.robotPosition)
        if len(child.get_polys()) == 0:
            return False
        
        canDrag = gui.can_drag(gripPts,direction=parentNode.robotPosition)
        if canDrag:        

        #add current drag to parent drag history
            child.dragHistory = list(parentNode.dragHistory)
            child.dragHistory.append((parentNode.robotPosition, distance))
            child.availableFolds = parentNode.availableFolds
            child.completedFolds  = parentNode.completedFolds
        #if(drag_x <= 0):
            return child
        else:
            return False
    return False

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




def updateAvailableFolds(fold,availableFoldList,completedFoldList):
    """                                                                                                                      
    Adds child nodes to fringe after current fold is performed                                                               
    """
    availableFoldList.remove(fold)
    completedFoldList.add(fold)
    for child in fold.getChildren():    
        for parent in child.getParents():
            if parent in completedFoldList:
                availableFoldList.add(child)
    return foldList

def goalTest(Node):
    if(len(Node.availableFolds) == 0):
        print "goaltest succeeded"
        return True

def FoldingSearch(startpoly):
    """
    implement a uniform cost search
    """
    print "FoldingSearch called"
    print "Timing Started"
    t = time.time() 
    # create the Fold Tree
    foldTree = gui.foldTree    
    start_availableFolds = []
    completedFolds = []
    for fold in foldTree:
        start_availableFolds.append(fold)
    alreadySeen = []
    searchQueue = util.PriorityQueue()
    searchQueue.push(SearchState(polys = [gui.startpoly],robotPosition='+y',availableFolds = start_availableFolds),0)

    #print "searchQueue",searchQueue.heap
    while not searchQueue.isEmpty():
        #print searchQueue.isEmpty()
        """
        print "\nQUEUE ELEMENTS"
        for el in searchQueue.heap:
            for poly in el[1].polys:
                print poly
        """        
        currentState = searchQueue.pop()
        print "popping currentState",currentState,"g ",currentState.get_g()
        #raw_input()
        if goalTest(currentState):
            break

        if not currentState in alreadySeen:
            #print "new node"
            alreadySeen.append(currentState)            
                       
            for child in currentState.makeChildren():
                searchQueue.push(child,child.get_g())    
                print "pushing child",child
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
    print "Time End" , time.time() - t
    # track back to find actions
    if(len(currentState.availableFolds) > 0):
        print "Search failed!"
        return
    state = currentState
    actions = []
    costs = []
    states = []
    while(state.parent != None):
        actions.append(state.actionToHere)
        states.append(state)
        costs.append(state.get_g())
        state = state.parent
        print state.actionToHere, state.get_g()
    states.append(state)
    actions.reverse()
    states.reverse()
    print "\n\nFinished folding. actions = "
    for action in actions:
        print action[0],"-",        
        if action[0] == "fold":
            print action[1].getstart(),action[1].getend()
        else:
            print action[1]

    gui.drawSimulationFolds(states)

def startFolding():   
    r = rospy.Rate(1)
    while not gui.readytoFold and not rospy.is_shutdown(): # spin until ready to fold
        r.sleep()
    print "starting folding"
    FoldingSearch(gui.startpoly)    

if __name__ == '__main__':    
    global gui
    rospy.init_node("FoldingSearch")    
    gui = FoldingGUIApoorva.FoldingGUI(name="FoldingGUI")
    gui.simFlag = True
    gui.setGripperLimit(2)
    startFolding()
    rospy.spin()

