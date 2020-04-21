#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 15:16:21 2019

@author: repa
@licence: GPL-v3.0
"""

# Panda3D imoprts
from direct.showbase.DirectObject import DirectObject
from direct.gui.DirectGui import DirectFrame, OnscreenText, DirectSlider
from panda3d.core import \
        TextNode, LineSegs, TextNode, Vec3, LColor, NodePath, Camera, \
        OrthographicLens
from math import pi, sin, cos, sqrt, atan, asin, acos, atan2
from numpy import degrees, deg2rad, radians

import numpy as np
from scipy import interpolate
import csv

# LineSegs

class Hud(DirectFrame):


    '''
    Display important parameters and create onscreen controls

    Uses DirectGui elements and text/line displays to provide information
    to the craft's operator. This is to be extended/developed into a 
    EID displays by the students of AERSP 597, Section 011, 2019/2020
    '''
    
    def __init__(self, base, marklist: list = None) -> None:

        '''
        Create a new hud

        @param base   A `ShowBase` object, used to access the Panda3d window
        '''

        # make a 2d overlay of the window, if e.g. your screen
        # aspect ratio is 3/4, this will create a coordinate system
        # with bottom left at (-4/3, -1), and top right at (4/3, 1) 
        dr = base.winList[0].makeDisplayRegion()
        # ask that it is drawn last, over all other elements there
        dr.setSort(20)

        # create the camera, 2D orthographic
        myCamera2d = NodePath(Camera('myCam2d'))
        lens = OrthographicLens()
        ar = base.getAspectRatio()
        if ar > 1:
            lens.setFilmSize(2*ar, 2)
        else:
            lens.setFilmSize(2/ar, 2)
        lens.setNearFar(-1000, 1000)
        myCamera2d.node().setLens(lens)

        # configure
        myRender2d = NodePath('myRender2d')
        myRender2d.setDepthTest(False)
        myRender2d.setDepthWrite(False)
        myCamera2d.reparentTo(myRender2d)
        dr.setCamera(myCamera2d)

        # Main background frame    
        self.fr = DirectFrame(
                pos=Vec3(-1.35,-1.0),
                frameSize = (0, 2.7, 0, 0.55),      
                frameColor = (0, 0, 0, 0.5))
        self.inf = DirectFrame(
                pos=Vec3(-1.35,-0.45),
                frameSize = (0, 1.85, 0, 0.1),
                frameColor = (1, 1, 1, 0.2))
        
        # # Map background frame
        self.map_LB = Vec3(0.5,-1.0)    # map left bot position
        self.map_origin = self.map_LB + Vec3(1000/3000,1200/4000)
        self.map_size = (0, 1.6, 0, 0.8)
        self.mp = DirectFrame(
                pos = self.map_LB,
                frameSize = self.map_size,
                frameColor = (0, 0, 0, 0.8))

        # Information Text (NextMark)   
        self.NextMark_display = OnscreenText(
                text="", pos=(0.8, 0.03), scale=0.06,
                fg=(1,1,0,1), align=TextNode.ARight,
                parent=self.mp)

        # Information Text (Control tiller & sail)   
        self.information_display = OnscreenText(
                text="", pos=(0.05, 0.03), scale=0.048,
                fg=(1,1,0,1), align=TextNode.ALeft,
                parent=self.inf)
        
        # Information Text (Reason of tacking)   
        self.tack_display = OnscreenText(
                text="", pos=(0.4, 0.64), scale=0.06,
                fg=(0,1,1,1.0), align=TextNode.ACenter,
                parent=self.mp)
        
        # # Recommended Tack frame (Left & Right)
        self.tkl = DirectFrame(
                pos=Vec3(0.52,-0.3),
                frameSize = (0, 0.13, 0, 0.08),
                frameColor = (0, 0, 0, 1))
        self.tkr = DirectFrame(
                pos=Vec3(1.18,-0.3),
                frameSize = (0, 0.13, 0, 0.08),
                frameColor = (0, 0, 0, 1)) 
        self.tack_l_monitor_display = OnscreenText(
                text="Tack", pos=(0.058, 0.03), scale=0.06,
                fg=(0,1,1,1), align=TextNode.ACenter,
                parent=self.tkl)
        self.tack_l_monitor0_display = OnscreenText(
                text="", pos=(0.058, 0.03), scale=0.06,
                fg=(0,0,0,1), align=TextNode.ACenter,
                parent=self.tkl)
        self.tack_r_monitor_display = OnscreenText(
                text="Tack", pos=(0.058, 0.03), scale=0.06,
                fg=(0,1,1,1), align=TextNode.ACenter,
                parent=self.tkr)        
        self.tack_r_monitor0_display = OnscreenText(
                text="", pos=(0.058, 0.03), scale=0.06,
                fg=(0,0,0,1), align=TextNode.ACenter,
                parent=self.tkr)
        
        # vehicle ground speed & relative speed to the mark display 
        self.sailtxt_display = OnscreenText(
                text="Ground Speed", pos=(0.03, 0.4), scale=0.05,  
                fg=(1,1,1,0.7), align=TextNode.ALeft,
                parent=self.fr)
        self.speed_display = OnscreenText(
                text="0.0", pos=(0.18, 0.3), scale=0.06,    
                fg=(0,1,0,0.7), align=TextNode.ALeft,
                parent=self.fr)

        self.sailtxt_display = OnscreenText(
                text="VMG Speed", pos=(0.03, 0.2), scale=0.05,  
                fg=(1,1,1,0.7), align=TextNode.ALeft,
                parent=self.fr)
        self.Vr_display = OnscreenText(
                text="0.0", pos=(0.18, 0.1), scale=0.07,    
                fg=(0,1,0,0.7), align=TextNode.ALeft,
                parent=self.fr)

        # current sail angle display
        self.sail_display = OnscreenText(
                text="0.0", pos=(0.7, 0.29), scale=0.06,    
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        # current sail angle display
        self.tiller_display = OnscreenText(
                text="0.0", pos=(1.2, 0.29), scale=0.06,    
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)

        # HDG angle display 
        self.hdg_display = OnscreenText(
                text="HDG  : 0.0", pos=(1.65, 0.4), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        # Wind direction display 
        self.psia_display = OnscreenText(
                text="WIND : 0.0", pos=(1.65, 0.3), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        # Information Text (Mode-Starbo/Port)         
        self.mode1_message_display = OnscreenText(
                text="", pos=(1.65, 0.2), scale=0.06,
                fg=(0,1,1,0.8), align=TextNode.ACenter,
                parent=self.fr)
        
        # Information Text (Mode-Down/UP-Wind)         
        self.mode2_message_display = OnscreenText(
                text="", pos=(1.65, 0.1), scale=0.06,
                fg=(0,1,1,0.8), align=TextNode.ACenter,
                parent=self.fr)
        
        # gui element for the tiller (steer the rudder)
        self.tillertxt_display = OnscreenText(
                text="-0.5       tiller       0.5", pos=(1.2, 0.04), scale=0.05,    
                fg=(1,1,1,0.7), align=TextNode.ACenter,
                parent=self.fr)
        self.tiller_gui = DirectSlider(
            pos=Vec3(-0.15, -0.9), scale=0.2,                 
            value=0.0, range=(-0.5,0.5), pageSize=0.05)

        # gui element for the mainsheet (control the sail)
        self.sailtxt_display = OnscreenText(
                text="0.0         sail         1.0", pos=(0.7, 0.04), scale=0.05,  
                fg=(1,1,1,0.7), align=TextNode.ACenter,
                parent=self.fr)
        self.mainsheet_gui = DirectSlider(
            pos=Vec3(-0.65, -0.9), scale=0.2,                  
            value=1.0, range=(0.05, 1.0), pageSize=0.05)
             

        # Temporary display*****************************************************************************
        self.ds_VMG_l_display = OnscreenText(
                text="0.0", pos=(0.75, 0.7), scale=0.06,    
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.ds_VMG_r_display = OnscreenText(
                text="0.0", pos=(0.75, 0.75), scale=0.06,    
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
            
        self.psi_VMG_l_display = OnscreenText(
                text="0.0", pos=(1.25, 0.7), scale=0.06,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.psi_VMG_r_display = OnscreenText(
                text="0.0", pos=(1.25, 0.75), scale=0.06,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)

  

    
        # example of a drawn instrument. Panda3d uses scene graph
        # techonology. Think of each graphical element as a node on
        # a branched tree. Each of the nodes can be scaled, rotated,
        # translated.

        # The compass consists of a LineSegs object with compass rose lines,
        # and we will combine that with N, E, S, W text labels
        # since we are doing 2d, only the first two element of vectors need
        # to be filled (x and y, z becomes zero).
        # Use moveTo and drawTo just like a pen, and if desired change color
        # and thickness along the way
        rose = LineSegs("rose")
        rose.setColor(LColor(1, 1, 1, 1))
        rose.setThickness(2.0)
        
        # compass rose lines
        for i in range(0,360,90):
            pnt = Vec3(sin(i/180*pi), cos(i/180*pi))
            rose.moveTo(pnt*0.75)
            rose.drawTo(pnt)
            for j in range(10,90,10):
                pnt = Vec3(sin((i + j)/180*pi), cos((i+j)/180*pi))
                rose.moveTo(pnt*0.85)
                rose.drawTo(pnt)
                
        # create and connect this to a new compass rose "node"
        self.compass_rose = NodePath("compass rose")
        self.compass_rose.attachNewNode(rose.create())
        
        # now connect the compass itself to the 2d window
        self.compass_rose.reparentTo(myRender2d)
        
        # now add the compass directions to the rose. These are (fixed)
        # TextNode objects
        # if you are new to Python, note that it can iterate over anything
        # iterable. Here we have a list (in []), with pairs ('tuples') of
        # data in it, each time a label and an angle
        # These are unpacked into l and angle variables
        for l, angle in [("N", 0), ("E", 90), 
                         ("S", 180), ("W", 270)]:
            
            # create the textnode object & set the text
            tn = TextNode("label " + l)
            tn.setText(l)
            
            # create an additional node in the center of the compass
            piv = NodePath("pivot " + l)
            
            # create & link the text node to the center pivot
            tnp = piv.attachNewNode(tn)
        
            # link the pivot point to the compass rose as a whole
            piv.reparentTo(self.compass_rose)
            
            # scale text, and offset relative to the pivot
            tnp.setScale(0.2)
            tnp.setPos(Vec3(-0.1*tn.getWidth(), 0.45))
            
            # now rotate the pivot
            piv.setHpr(0, 0, angle)
            
        # scale the combined compass & put it in the corner
        self.compass_rose.setScale(0.2)
        self.compass_rose.setPos(Vec3(0.3,-0.65))    
        
        # Mark direction Indicator
        mark_indicator = LineSegs("mark_indicator")
        mark_indicator.setColor(LColor(1, 0.5, 0, 1))
        mark_indicator.setThickness(4.0)

        pnt = Vec3(0.0,0.5)
        mark_indicator.moveTo(0.0)
        mark_indicator.drawTo(pnt)

        self.compass_mark = NodePath("compass mark")
        self.compass_mark.attachNewNode(mark_indicator.create())

        self.compass_mark.reparentTo(myRender2d)

        self.compass_mark.setScale(1)
        self.compass_mark.setPos(Vec3(0.3,-0.65)) 

        # Speed Rose
        speed = LineSegs("speed")
        speed.setColor(LColor(1, 1, 1, 1))
        speed.setThickness(2.0)
        
        # speed rose lines
        for i in range(0,360,90):
            pnt = Vec3(sin(i/180*pi), cos(i/180*pi))
            speed.moveTo(pnt*0.75)
            speed.drawTo(pnt)
            for j in range(10,90,9):
                pnt = Vec3(sin((i + j)/180*pi), cos((i+j)/180*pi))
                speed.moveTo(pnt*0.85)
                speed.drawTo(pnt)
                
        # create and connect this to a new speed rose "node"
        self.speed_rose = NodePath("speed indicator")
        self.speed_rose.attachNewNode(speed.create())
        
        # now connect the compass itself to the 2d window
        self.speed_rose.reparentTo(myRender2d)

        for l, angle in [("0", 0), ("10", 90), 
                         ("20", 180), ("30", 270)]:
            
            # create the textnode object & set the text
            tn = TextNode("label " + l)
            tn.setText(l)
            
            # create an additional node in the center of the compass
            piv = NodePath("pivot " + l)
            
            # create & link the text node to the center pivot
            tnp = piv.attachNewNode(tn)
        
            # link the pivot point to the speed rose as a whole
            piv.reparentTo(self.speed_rose)
            
            # scale text, and offset relative to the pivot
            tnp.setScale(0.3)
            tnp.setPos(Vec3(-0.2*tn.getWidth(), 0.45))
            
            # now rotate the pivot
            piv.setHpr(0, 0, angle)
            
        # scale the combined compass & put it in the corner
        self.speed_rose.setScale(0.2)
        self.speed_rose.setPos(Vec3(-1.1,-0.65))

        # Speed Indicator
        speed_indicator = LineSegs("speed_indicator")
        speed_indicator.setColor(LColor(1.0, 0.0, 0.0, 1))
        speed_indicator.setThickness(3.0)

        pnt = Vec3(0.0,0.5)
        speed_indicator.moveTo(0.0)
        speed_indicator.drawTo(pnt)

        self.speed_arrow = NodePath("speed_arrow")
        self.speed_arrow.attachNewNode(speed_indicator.create())

        self.speed_arrow.reparentTo(myRender2d)

        self.speed_arrow.setScale(0.4)
        self.speed_arrow.setPos(Vec3(-1.1,-0.65))

         # Sail Rose
        sail = LineSegs("sail")
        sail.setColor(LColor(1, 1, 1, 1))
        sail.setThickness(2.0)
        
        # sail rose lines
        for i in range(0,360,45):
            pnt = Vec3(sin(i/180*pi), cos(i/180*pi))
            sail.moveTo(pnt*0.7)
            sail.drawTo(pnt)
            for j in range(9,90,9):
                pnt = Vec3(sin((i + j)/180*pi), cos((i+j)/180*pi))
                sail.moveTo(pnt*0.85)
                sail.drawTo(pnt)
                
        # create and connect this to a new sail rose "node"
        self.sail_rose = NodePath("sail indicator")
        self.sail_rose.attachNewNode(sail.create())
        
        # now connect the compass itself to the 2d window
        self.sail_rose.reparentTo(myRender2d)

        for l, angle in [("- 0 +", 0), ("30", 90), 
                         ("60", 180), ("30", 270)]:
            
            # create the textnode object & set the text
            tn = TextNode("label " + l)
            tn.setText(l)
            
            # create an additional node in the center of the compass
            piv = NodePath("pivot " + l)
            
            # create & link the text node to the center pivot
            tnp = piv.attachNewNode(tn)
        
            # link the pivot point to the sail rose as a whole
            piv.reparentTo(self.sail_rose)
            
            # scale text, and offset relative to the pivot
            tnp.setScale(0.3)
            tnp.setPos(Vec3(-0.15*tn.getWidth(), 0.45))
            
            # now rotate the pivot
            piv.setHpr(0, 0, angle)
            
        # scale the combined compass & put it in the corner
        self.sail_rose.setScale(0.2)
        self.sail_rose.setPos(Vec3(-0.65,-0.65))

        # sail Indicator
        sail_indicator = LineSegs("sail_indicator")
        sail_indicator.setColor(LColor(1.0, 0.0, 0.0, 1))
        sail_indicator.setThickness(3.0)

        pnt = Vec3(0.0,0.5)
        sail_indicator.moveTo(0.0)
        sail_indicator.drawTo(pnt)

        self.sail_arrow = NodePath("sail_arrow")
        self.sail_arrow.attachNewNode(sail_indicator.create())

        self.sail_arrow.reparentTo(myRender2d)

        self.sail_arrow.setScale(0.4)
        self.sail_arrow.setPos(Vec3(-0.65,-0.65))

        # tiller Rose
        tiller = LineSegs("tiller")
        tiller.setColor(LColor(1, 1, 1, 1))
        tiller.setThickness(2.0)
        
        # tiller rose lines
        for i in range(0,360,90):
            pnt = Vec3(sin(i/180*pi), cos(i/180*pi))
            tiller.moveTo(pnt*0.7)
            tiller.drawTo(pnt)
            for j in range(15,90,15):
                pnt = Vec3(sin((i + j)/180*pi), cos((i+j)/180*pi))
                tiller.moveTo(pnt*0.85)
                tiller.drawTo(pnt)
                
        # create and connect this to a new tiller rose "node"
        self.tiller_rose = NodePath("tiller indicator")
        self.tiller_rose.attachNewNode(tiller.create())
        
        # now connect the compass itself to the 2d window
        self.tiller_rose.reparentTo(myRender2d)

        for l, angle in [("0", 0), ("14.3", 90), 
                         ("28.6", 180), ("18", 270)]:
            
            # create the textnode object & set the text
            tn = TextNode("label " + l)
            tn.setText(l)
            
            # create an additional node in the center of the compass
            piv = NodePath("pivot " + l)
            
            # create & link the text node to the center pivot
            tnp = piv.attachNewNode(tn)
        
            # link the pivot point to the tiller rose as a whole
            piv.reparentTo(self.tiller_rose)
            
            # scale text, and offset relative to the pivot
            tnp.setScale(0.3)
            tnp.setPos(Vec3(-0.2*tn.getWidth(), 0.45))
            
            # now rotate the pivot
            piv.setHpr(0, 0, angle)
            
        # scale the combined compass & put it in the corner
        self.tiller_rose.setScale(0.2)
        self.tiller_rose.setPos(Vec3(-0.15,-0.65))

        # tiller Indicator
        tiller_indicator = LineSegs("tiller_indicator")
        tiller_indicator.setColor(LColor(1.0, 0.0, 0.0, 1))
        tiller_indicator.setThickness(3.0)

        pnt = Vec3(0.0,0.5)
        tiller_indicator.moveTo(0.0)
        tiller_indicator.drawTo(pnt)

        self.tiller_arrow = NodePath("tiller_arrow")
        self.tiller_arrow.attachNewNode(tiller_indicator.create())

        self.tiller_arrow.reparentTo(myRender2d)

        self.tiller_arrow.setScale(0.4)
        self.tiller_arrow.setPos(Vec3(-0.15,-0.65))

        # boat icon
        boat = LineSegs("boat")
        boat.setColor(LColor(.7,.7,.7,1))
        boat.setThickness(3.0)

        boat.moveTo(Vec3(0.0,0.0))          
        boat.drawTo(Vec3(0.0, 0.75))        
        boat.moveTo(Vec3(-0.2,0.0))        
        boat.drawTo(Vec3(0.2,-0.0))         

        self.compass_boat = NodePath("compass boat")
        self.compass_boat.attachNewNode(boat.create())

        self.compass_boat.reparentTo(myRender2d)

        self.compass_boat.setScale(0.2)
        self.compass_boat.setPos(Vec3(0.3,-0.65))    

          

        # Wind Indicator
        wind_indicator = LineSegs("wind_indicator")
        wind_indicator.setColor(LColor(0, 0.7, 0.7, 1))
        wind_indicator.setThickness(4.0)

        pnt = Vec3(0.0,0.5)
        wind_indicator.moveTo(0.0)
        wind_indicator.drawTo(pnt)

        self.compass_wind = NodePath("compass wind")
        self.compass_wind.attachNewNode(wind_indicator.create())

        self.compass_wind.reparentTo(myRender2d)

        self.compass_wind.setScale(1)
        self.compass_wind.setPos(Vec3(0.3,-0.65))    

        # Sail Indicator
        sail_indicator = LineSegs("sail_indicator")
        sail_indicator.setColor(LColor(0, 1, 0, 1))
        sail_indicator.setThickness(4.0)

        pnt = Vec3(0.0,0.5)
        sail_indicator.moveTo(0.0)
        sail_indicator.drawTo(pnt)

        self.compass_sail = NodePath("compass sail")
        self.compass_sail.attachNewNode(sail_indicator.create())

        self.compass_sail.reparentTo(myRender2d)

        self.compass_sail.setScale(1)
        self.compass_sail.setPos(Vec3(0.3,-0.65))    
        

        # Boat position
        boat_pos = LineSegs("boat_pos")
        boat_pos.setColor(LColor(1, 1, 1, 1.0))
        boat_pos.setThickness(1.0)
        angleRadians = deg2rad(360)
        numSteps = 30
        boat_pos.moveTo(Vec3(0,2))
        boat_pos.drawTo(Vec3(0,0))
        for i in range(numSteps + 1):
            a = angleRadians * i / numSteps
            x = sin(a)
            y = cos(a)
            
            boat_pos.drawTo(Vec3(x,y))
            boat_pos.moveTo(Vec3(x,y))

        self.display_boat = NodePath("display boat")
        self.display_boat.attachNewNode(boat_pos.create())

        self.display_boat.reparentTo(myRender2d)
        self.display_boat.setScale(0.02)
        self.display_boat.setPos(Vec3(0.0,0.0))
        
        # Mark1 position
        mark1_pos = LineSegs("mark1_pos")
        mark1_pos.setColor(LColor(1.0, 0.6, 0.0, 1.0))
        mark1_pos.setThickness(2.0)
        angleRadians = deg2rad(360)
        numSteps = 30
        mark1_pos.moveTo(Vec3(0,-7.5))
        for i in range(numSteps + 1):
            a = angleRadians * i / numSteps
            x = sin(a)
            y = cos(a)
            
            mark1_pos.drawTo(Vec3(x,y))
            #mark1_pos.moveTo(Vec3(x,y))

        self.display_mark1 = NodePath("display mark1")
        self.display_mark1.attachNewNode(mark1_pos.create())

        self.display_mark1.reparentTo(myRender2d)
        self.display_mark1.setScale(0.01)
        self.display_mark1.setPos(Vec3(0.0,0.0))

        # Mark2 position
        mark2_pos = LineSegs("mark2_pos")
        mark2_pos.setColor(LColor(1.0, 0.6, 0.0, 1.0))
        mark2_pos.setThickness(2.0)
        angleRadians = deg2rad(360)
        numSteps = 30
        mark2_pos.moveTo(Vec3(0,-7.5))
        for i in range(numSteps + 1):
            a = angleRadians * i / numSteps
            x = sin(a)
            y = cos(a)
            
            mark2_pos.drawTo(Vec3(x,y))
            #mark2_pos.moveTo(Vec3(x,y))

        self.display_mark2 = NodePath("display mark2")
        self.display_mark2.attachNewNode(mark2_pos.create())

        self.display_mark2.reparentTo(myRender2d)
        self.display_mark2.setScale(0.01)
        self.display_mark2.setPos(Vec3(0.0,0.0))

        # Mark3 position
        mark3_pos = LineSegs("mark3_pos")
        mark3_pos.setColor(LColor(1.0, 0.6, 0.0, 1.0))
        mark3_pos.setThickness(2.0)
        angleRadians = deg2rad(360)
        numSteps = 30
        mark3_pos.moveTo(Vec3(0,-7.5))
        for i in range(numSteps + 1):
            a = angleRadians * i / numSteps
            x = sin(a)
            y = cos(a)
            
            mark3_pos.drawTo(Vec3(x,y))

        self.display_mark3 = NodePath("display mark3")
        self.display_mark3.attachNewNode(mark3_pos.create())

        self.display_mark3.reparentTo(myRender2d)
        self.display_mark3.setScale(0.01)
        self.display_mark2.setPos(Vec3(0.0,0.0))

        # Goal position
        goal_pos = LineSegs("goal_pos")
        goal_pos.setColor(LColor(1.0, 0.0, 0.0, 1.0))
        goal_pos.setThickness(2.0)
        angleRadians = deg2rad(360)
        numSteps = 30
        for i in range(numSteps + 1):
            a = angleRadians * i / numSteps
            x = sin(a)
            y = cos(a)
            
            goal_pos.drawTo(Vec3(x,y))
            goal_pos.moveTo(Vec3(x,y))
        goal_pos.moveTo(Vec3(-2,0))
        goal_pos.drawTo(Vec3(2,0))
        self.display_goal = NodePath("display goal")
        self.display_goal.attachNewNode(goal_pos.create())

        self.display_goal.reparentTo(myRender2d)
        self.display_goal.setScale(0.01)
        self.display_goal.setPos(Vec3(0.0,0.0))

        # get the cl alpha curve for the sail
        clcda = csv.reader(open('cl-alpha.csv'))
        clcda.__next__()   # skip header
        alpha=[]
        cl=[]
        for row in clcda:
            alpha.append(row[0])
            cl.append(row[1])
        self.cl_alpha = interpolate.splrep(alpha, cl)
        
        # and the cd alpha curve
        clcda = csv.reader(open('cd-alpha.csv'))
        clcda.__next__()
        alpha=[]
        cd=[]
        for row in clcda:
            alpha.append(row[0])
            cd.append(row[1])
        self.cd_alpha = interpolate.splrep(alpha, cd)
        self.doprint = -30

        # list of race marks
        # (type: '+'/'-' : round clockwise, counterclockwise
        #        'a'     : avoid (penalty)
        #        's'     : start
        #        'f'     : finish
        #  name:         : label for mark
        #  info:         : textual info
        #  xy            : position)
        self.marklist = marklist
        
    def update(self, x, y, psi, V, psiw, Vw, ds, others, NextMark, WIND, race_events=None):
        '''
        Update the information on the displays

        Parameters
        ----------
        x : float [m]
            X location (North from center of the map) of the vehicle.
        y : float [m]
            Y location (East from center of the map) of the vehicle.
        psi : float [deg]
            Heading, in degrees.
        V : float [kts]
            Vehicle's speed over the ground.
        psiw : float [deg]
            relative direction of the wind, with respect to the vehicle. 
            0 means wind coming from the bow (front), ranges between -180 and
            180
        Vw : float [kts]
            Relative wind speed, as would be measured by an anemometer in
            the mast.
        ds : float [deg]
            mainsheet angle, degrees, indicates how the boom/jib has moved.
        others : sequence of (...)
            Location of the other players in the field.
        race_events : sequence of (str, int, float)
            Notifications on race progress. The string indicating the event
            is 'R'; for rounding, 'P'; for penalty, 'F'; for finish.
            The integer indicates the rounded mark; 0=start
            the float indicates the elapsed time.

        Returns
        -------
        float [deg]
            Tiller commanded angle.
        float [deg]
            Mainsheet commanded angle.
        '''
        
        # Change the range, 0~360, of Heading(psi) and the relative wind direction 
        if psi < 0:
            psi = psi + 360
        if psiw < 0:
            psiw_ori = psiw
            psiw = psiw + 360
        else:
            psiw_ori = psiw
            psiw = psiw

        # Define the real wind direction(psia) and speed(Va)
        Va = abs(WIND[0])
        w_x = WIND[1]
        w_y = WIND[2]
        psia = degrees(atan2(WIND[2], WIND[1]))
        
        # Define Mode of Starbo/Port and UPwind/DOWNwind
        if (-90 <= psi - psia <= 0) or (270 <= psi - psia < 360):
            Mode = 1
            self.mode1_message_display.text = "Starboard"
            self.mode2_message_display.text = "UP-Wind"
        elif (-180 <= psi - psia < -90) or (180 <= psi - psia < 270):
            Mode = 2
            self.mode1_message_display.text = "Starboard"
            self.mode2_message_display.text = "DOWN-Wind"
        elif (-360 < psi - psia <= -270) or (0 < psi - psia <= 90):
            Mode = 3
            self.mode1_message_display.text = "Port"
            self.mode2_message_display.text = "UP-Wind"
        else:
            Mode = 4
            self.mode1_message_display.text = "Port"
            self.mode2_message_display.text = "DOWN-Wind"

        # update the compass rotation, and set the speed text
        self.compass_rose.setHpr(0, 0, -psi)
        self.speed_display.text = "{:2.1f} kts".format(V)
        self.sail_display.text = "{:2.0f} deg" .format(ds)
        self.tiller_display.text = "{:2.0f} deg" .format(self.tiller_gui['value']*180/pi)
        #self.ref_display.text = "ref:{:2.0f} deg" .format(abs(psiw-30))
        self.hdg_display.text = "HDG : {:2.0f}" .format(psi)
        
        # update the wind indicator on compass rotation
        self.compass_wind.setHpr(0,0, psiw) # get the true wind direction
        self.compass_wind.setScale(0.1+0.007*Vw)

        # update the sail indicator on compass rotation
        self.compass_sail.setHpr(0,0, 180+ds) # get the true sail direction
        self.compass_sail.setScale(0.2)

        # update the speed indicator on compass rotation
        self.speed_arrow.setHpr(0,0, V*9)

        # update the sail indicator on compass rotation
        self.sail_arrow.setHpr(0,0, ds*pi)

        # update the tiller indicator on compass rotation
        self.tiller_arrow.setHpr(0,0, self.tiller_gui['value']*360)
        
        # map position unit conversion
        map_scale_x = 3000
        map_scale_y = 4000
        map_x = self.map_origin[1] + y/map_scale_x
        map_y = self.map_origin[2] + x/map_scale_y

        # update the boat pose on map
        self.display_boat.setHpr(0,0,psi)
        self.display_boat.setPos(Vec3(map_x,map_y))

        # update marks position on map
        start_x = self.map_origin[1] - 650 /map_scale_x
        start_y = self.map_origin[2] - 200 /map_scale_y
        mark1_x = self.map_origin[1] - 340 /map_scale_x
        mark1_y = self.map_origin[2] - 900 /map_scale_y
        mark2_x = self.map_origin[1] + 325 /map_scale_x
        mark2_y = self.map_origin[2] + 165 /map_scale_y
        mark3_x = self.map_origin[1] + 1070/map_scale_x
        mark3_y = self.map_origin[2] - 410 /map_scale_y
        goal_x  = self.map_origin[1] + 555 /map_scale_x
        goal_y  = self.map_origin[2] + 1890/map_scale_y

        self.display_mark1.setPos(Vec3(mark1_x,mark1_y))
        self.display_mark2.setPos(Vec3(mark2_x,mark2_y))
        self.display_mark3.setPos(Vec3(mark3_x,mark3_y))
        self.display_goal.setPos(Vec3(goal_x,goal_y))
        #print("map_x: %.4f" %map_x,"  map_y: %.4f" %map_y)
           

        ############################################
        #        TOPPLEING CONTROL AND WARNING     #
        ############################################
        
        # Still testing in MATLAB, will update soon!!!

        m = 250
        rho = 1.225
        S = 6
        g=9.81
        gamma = 0.3897

        alpha = np.deg2rad(360-psiw-ds)
        
        if alpha > np.pi: alpha -= np.pi
        if -alpha < -np.pi: alpha += np.pi

        cl = interpolate.splev(abs(alpha)/np.pi*180, self.cl_alpha)
        cd = interpolate.splev(abs(alpha)/np.pi*180, self.cd_alpha)

        qS = 0.5 * rho* (Vw*0.5144) *(Vw*0.5144) *S
        D = qS * (cd + 0.05)
        L = qS * cl

        # Sail Moment
        beta = np.pi - atan2(cl,cd) - alpha
        F_sail = sqrt(L*L+D*D)*cos(pi/2-gamma-beta)
        if L < 0: F_sail *= -1
        M_sail = F_sail*2.8

        # Mass Moment
        M_mass = m * g * 3.3 * sin(gamma)

        # Returning Moment
        delta = self.tiller_gui['value']
        if delta < 0.0005 and delta >=0: R = -11200
        elif delta > -0.0005 and delta <=0: R = 11200
        else: R = -5.6/delta

        Fr = m * (V*0.5144)*(V*0.5144) / R * cos(delta-gamma)
        M_r = Fr*0.8
        Vw_ms = Vw*0.5144
        alpha_deg = np.rad2deg(alpha)
        print("M_r: %.2f" %M_r,"  Vw: %.2f" %Vw_ms,"  psiw: %.2f" %psiw,"  alpha: %.2f" %alpha_deg,"F_sail: %.2f" %F_sail,"  M_sail: %.2f" %M_sail)
        if (M_mass - abs(M_r + M_sail)) > 0:
            print("safe\n")
        else:
            print("Topple warning!!\n")


        #########################################
        #        TACKING and MARK OPERATION     #
        #########################################
        
        # Hey Masa! Here's some example codes to read mark positions:

        # Read Mark Position from marklist
        '''
        if len(self.marklist) != 0:
            print("mark1 info: ", self.marklist[1])
            for i in self.marklist:                
                #print("mark1 position: ", self.marklist[1][3])
                m_pos1 = self.marklist[1][3]
                mark1_x = m_pose1[0]
                mark1_y = m_pose1[1]
                
                #print("mark2 position: ", self.marklist[2][3])
                m_pos2 = self.marklist[2][3]
                mark2_x = m_pose2[0]
                mark2_y = m_pose2[1]
                
                #print("mark3 position: ", self.marklist[3][3])
                m_pos3 = self.marklist[3][3]
                mark3_x = m_pose3[0]
                mark3_y = m_pose3[1]
                
                #print("mark4 position: ", self.marklist[4][3])
                m_pos4 = self.marklist[4][3]
                mark4_x = m_pose4[0]
                mark4_y = m_pose4[1]
            
            print ("relative position to mark1: %.2f , %.2f" %(self.marklist[1][3][0]-x, self.marklist[1][3][1]-y))
            print ("distance to mark1: %.2f" %(sqrt(pow(self.marklist[1][3][0]-x,2)+pow(self.marklist[1][3][1]-y,2))))
        '''

        # Next Mark Message, and set the mark's position     
        if NextMark == 1:
            Mark_x = mark1_x
            Mark_y = mark1_y
            self.NextMark_display.text = "Go to Mark 1"
        elif NextMark == 2:
            Mark_x = mark2_x
            Mark_y = mark2_y
            self.NextMark_display.text = "Go to Mark 2"
        elif NextMark == 3:
            Mark_x = mark3_x
            Mark_y = mark3_y
            self.NextMark_display.text = "Go to Mark 3"
        elif NextMark == 4:
            Mark_x = goal_x
            Mark_y = goal_y
            self.NextMark_display.text = "Go to Finish line"
        else:
            Mark_x = 0
            Mark_y = 0
            self.NextMark_display.text = "FINISH !!!!"
        
        #Calculate the direction to the next mark(psim) and range to the mark(R)
        theta = (90 - degrees(atan2((Mark_y - map_y)* map_scale_y, (Mark_x - map_x)* map_scale_x))) - psi
        if theta < 0:
            psim = 360 + theta
        elif theta >= 360:
            psim = theta - 360
        else:
            psim = theta
        
        Mark_dir = theta + psi
        if Mark_dir < 0:
            Mark_dir = Mark_dir + 360
        elif Mark_dir >= 360:
            Mark_dir = Mark_dir -360
        
        R = sqrt(((map_x - Mark_x) * map_scale_x)**2 + ((map_y - Mark_y) * map_scale_y)**2)
        around2 = 100

        self.compass_mark.setHpr(0,0, psim)         # update psim on compass rotation
        self.compass_mark.setScale(0.42)
        
        
        #Calculate the relative speed to the mark VMG(Vr)
        Vr = V * cos(radians(theta))
        
        # Define the timing of the tack
        TackAngle = 30 * 2
        flg_tack_l = 0
        flg_tack_r = 0
        
        if Mode == 2 or Mode == 4:
            flg_tack_l = 0
            flg_tack_r = 0
            self.tack_display.text = "----"
        elif (x <= (0-(-1250))/(490-(-480)) * y - 600 and -480 <= y <= 490) or (200 < y < 400 and x < 0):
            flg_tack_l = 1
            self.tack_display.text = "Avoid Wall"
        elif Mode == 1 and (TackAngle/2 + 15 < abs(psia - Mark_dir) < (360 - TackAngle/2 - 15)):         
            flg_tack_r = 1
            self.tack_display.text = "Mark Position"
        elif Mode == 3 and (TackAngle/2+15 < abs(psia - Mark_dir) < (360 - TackAngle/2 - 15)):
            flg_tack_l = 1
            self.tack_display.text = "Mark Position"
        #elif sqrt(((map_x - others[1]) * map_scale_x)**2 + ((map_y - others[2]) * map_scale_y)**2) < 15 and 0 <= psiw <= 180:
        #    flg_tack_r = 1
        #    self.tack_display.text = "Blanket"
        #elif sqrt(((map_x - others[1]) * map_scale_x)**2 + ((map_y - others[2]) * map_scale_y)**2) < 15 and 180 < psiw < 360:
        #    flg_tack_l = 1
        #    self.tack_display.text = "Blanket"
        # elif Mode == 1 and V * cos(radians(TackAngle - (psim-psi))) >= Vr * 1.5:
        #     flg_tack_r = 1
        #     self.tack_display.text = "Move away from Mark"
        # elif Mode == 3 and V * cos(radians(TackAngle - (psi-psim))) >= Vr * 1.5:
        #     flg_tack_l = 1
        #     self.tack_display.text = "Move away from Mark"
        else:
            flg_tack_l = 0
            flg_tack_r = 0
            self.tack_display.text = "----"
            
        if flg_tack_l == 1:
            self.tack_l_monitor0_display.text = ""
        elif flg_tack_r == 1:
            self.tack_r_monitor0_display.text = ""
        else:
            self.tack_l_monitor0_display.text = "Tack"
            self.tack_r_monitor0_display.text = "Tack"
            

        # Calculate best heading(psi_VMG)/mainsheet(ds_VMG) for getting VMG
        ds_VMG_l = 9999
        ds_VMG_r = 9999
        psi_VMG_l = 9999
        psi_VMG_r = 9999
        if R <= around2:                  # Mark roundabout
            ds_VMG_l = 9999
            ds_VMG_r = 9999
            psi_VMG_l = 9999
            psi_VMG_r = 9999
            if V <= 15:
                self.information_display.text = "Round! Fix Sail=0. Then move Tiller quickly to the Next Mark without touching."
            elif V > 15:
                self.information_display.text = "Round! Loose Sail in half. Then move Tiller quickly to the Next Mark without touching."
        elif (abs(psia - Mark_dir) <= TackAngle/2+20) or (abs(psia - Mark_dir) >= (360 - TackAngle/2 - 20)):       # UPwind(close-hauled)
            ds_VMG_l = 0
            ds_VMG_r = 0
            if psia + TackAngle/2 >= 360:
                psi_VMG_r = psia + TackAngle/2 - 360
            else:
                psi_VMG_r = psia + TackAngle/2
            if psia - TackAngle/2 < 0:
                psi_VMG_l = psia - TackAngle/2 + 360
            else:
                psi_VMG_l = psia - TackAngle/2
            if (flg_tack_l==1 or flg_tack_r==1) and V <= 15:
                self.information_display.text = "Tack! Fix Sail=0. Then move Tiller quickly to the another side Green."
            elif (flg_tack_l==1 or flg_tack_r==1) and V > 15:
                self.information_display.text = "Tack! Loose Sail in half. Then move Tiller quickly to the another side Green."
            else:
                self.information_display.text = "Fix Sail=0. Then control Tiller on Green."
        elif 90 < psiw < 270:        # DownWind(reaching)
            ds_VMG_l = 60
            ds_VMG_r = -60
            psi_VMG_l = Mark_dir
            psi_VMG_r = Mark_dir
            self.information_display.text = "Fix Sail=MAX. After going UP-wind & speeding UP, control Tiller to Next Mark. "
        else:                               # Abeam
            if psiw_ori < 0:
                ds_VMG_r = psiw_ori + 30
                if ds_VMG_r <= -60:
                    ds_VMG_r = -60
                elif ds_VMG_r >= 0:
                    ds_VMG_r = 0
            else:
                ds_VMG_l = psiw_ori - 30
                if ds_VMG_l >= 60:
                    ds_VMG_l = 60
                elif ds_VMG_l <= 0:
                    ds_VMG_l = 0
            psi_VMG_r = Mark_dir
            psi_VMG_l = Mark_dir
            self.information_display.text = "Set Tiller to Next Mark. Then control Sail on Green."

        # For display of text values
        self.Vr_display.text = "{:2.1f}" .format(Vr)
        self.ds_VMG_l_display.text = "sail for VMG:{:2.0f}" .format(ds_VMG_l)
        self.ds_VMG_r_display.text = "sail for VMG:{:2.0f}" .format(ds_VMG_r)
        self.psi_VMG_l_display.text = "HDG for VMG:{:2.0f}" .format(psi_VMG_l)
        self.psi_VMG_r_display.text = "HDG for VMG:{:2.0f}" .format(psi_VMG_r)
        self.psia_display.text = "WIND : {:2.0f}" .format(psia)


        # return the control inputs as a pair of values
        return self.tiller_gui['value'], self.mainsheet_gui['value']
    
    
if __name__ == '__main__':

    # this section is only executed if the hud.py is directly started,
    # it is for testing
    #
    # normally hud.py is imported by skater.py, and the test is not
    # performed (because then __name__ will be 'hud', Python trick ;-)

    # additional imports
    from direct.showbase.ShowBase import ShowBase
    from direct.task import Task

    
    class TestHud(ShowBase):
        '''
        Test object to testdrive the hud with dummy sine inputs
        '''
        
        def __init__(self):
            ''' 
            Create a test object
            '''
            
            ShowBase.__init__(self)
            self.taskMgr.add(self.testupdate, "testupdate")
            self.scene = self.loader.loadModel("models/environment")
            self.scene.reparentTo(self.render)
            self.scene.setScale(0.25, 0.25, 0.25)
            self.scene.setPos(-8, 42, 0)
            self.hud = Hud(self)
            self.dashboard = DirectFrame(
                    pos=Vec3(-0.8, 0.4), frameSize = (0, 0.4, 0, 0.4),
                    frameColor=(1, 1, 1, 0.5))
            
            
        def testupdate(self, task):
            '''
            drive the hud, callback
            '''
            
            # generate some nonsense data
            psi = sin(0.2*task.time)*360
            x = sin(0.1*task.time)*10
            y = cos(0.1*task.time)*10
            V = 5 + sin(0.1*task.time)
            psiw = psi + 0.2*sin(0.1*task.time)
            Vw = 8 + sin(0.05*task.time)
            ds = degrees(sin(task.time))
            others = [ (-20 + 3*sin(0.2*task.time), 20 + 3*cos(0.2*task.time)),
                       ( 20 + 3*sin(0.2*task.time), 20 + 3*cos(0.2*task.time)) ]

            self.hud.update(x, y, psi, V, psiw, Vw, ds, others, NextMark, WIND)
            
            return Task.cont
    
    # create and run
    testhud = TestHud()
    testhud.run()