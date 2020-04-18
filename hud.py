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
from math import pi, sin, cos, sqrt, atan, asin, acos
from numpy import degrees, deg2rad, radians

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

        # example of a colored, fixed frame    
        self.fr = DirectFrame(
                pos=Vec3(-1.35,-1.0),
                frameSize = (0, 2.7, 0, 0.55),      #(by MASA, 04/17/2020)
                frameColor = (0, 0, 0, 0.5))
        
        self.map_LB = Vec3(0.5,-1.0)    # map left bot position
        self.map_origin = self.map_LB + Vec3(1000/3000,1200/4000)
        self.map_size = (0, 1.6, 0, 0.8)
        self.mp = DirectFrame(
                pos = self.map_LB,
                frameSize = self.map_size,
                frameColor = (0, 0, 0, 0.8))
                
        
        # Fixed Frame & Text for Recommended Tack (by MASA, 04/17/2020) 
        self.tkl = DirectFrame(
                pos=Vec3(0.52,-0.3),
                frameSize = (0, 0.13, 0, 0.08),
                frameColor = (1, 0.75, 0, 1))
        self.speed_display = OnscreenText(
                text="Tack", pos=(0.058, 0.03), scale=0.05,
                fg=(0,0,0,1), align=TextNode.ACenter,
                parent=self.tkl)
        
        self.tkr = DirectFrame(
                pos=Vec3(1.18,-0.3),
                frameSize = (0, 0.13, 0, 0.08),
                frameColor = (0.4, 0.4, 0.4, 1))
        self.speed_display = OnscreenText(
                text="Tack", pos=(0.058, 0.03), scale=0.05,
                fg=(0,0,0,1), align=TextNode.ACenter,
                parent=self.tkr)
       
        
        # Information Text (by MASA, 04/17/2020) 
        self.speed_display = OnscreenText(
                text="Blanket", pos=(0.4, 0.7), scale=0.06,
                fg=(0,1,0,0.8), align=TextNode.ACenter,
                parent=self.mp)
        
        self.tkl = DirectFrame(
                pos=Vec3(-1.35,-0.45),
                frameSize = (0, 1.85, 0, 0.1),
                frameColor = (1, 1, 1, 0.2))
        '''
        self.NextMark_display = OnscreenText(
                text="", pos=(0.05, 0.03), scale=0.05,
                fg=(1,1,0,1), align=TextNode.ALeft,
                parent=self.tkl)
        '''
        self.information_display = OnscreenText(
                text="", pos=(0.6, 0.03), scale=0.05,
                fg=(1,1,0,1), align=TextNode.ALeft,
                parent=self.tkl)


        # vehicle speed display, fixed (easy) number
        self.speed_display = OnscreenText(
                text="0.0", pos=(0.25, 0.29), scale=0.07,    #(by MASA, 04/17/2020)
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.Vr_display = OnscreenText(
                text="0.0", pos=(0.25, 0.35), scale=0.07,    #(by MASA, 04/17/2020)
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)

        # sail angle display, fixed (easy) number
        self.sail_display = OnscreenText(
                text="0.0", pos=(0.75, 0.29), scale=0.07,    #(by MASA, 04/17/2020)
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.ds_VMG_l_display = OnscreenText(
                text="0.0", pos=(0.75, 0.35), scale=0.07,    #(by MASA, 04/17/2020)
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.ds_VMG_r_display = OnscreenText(
                text="0.0", pos=(0.75, 0.4), scale=0.07,    #(by MASA, 04/17/2020)
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        # tiller angle display, fixed (easy) number (by MASA, 04/17/2020)
        self.tiller_display = OnscreenText(
                text="0.0", pos=(1.25, 0.29), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.psi_VMG_l_display = OnscreenText(
                text="0.0", pos=(1.25, 0.35), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.psi_VMG_r_display = OnscreenText(
                text="0.0", pos=(1.25, 0.4), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.psia_display = OnscreenText(
                text="0.0", pos=(1.25, 0.45), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.psim_display = OnscreenText(
                text="0.0", pos=(1.25, 0.5), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.Mode_display = OnscreenText(
                text="0.0", pos=(1.25, 0.55), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.AOA_display = OnscreenText(
                text="0.0", pos=(1.25, 0.6), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.psia_2_display = OnscreenText(
                text="0.0", pos=(1.25, 0.65), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
    
        
        self.theta_display = OnscreenText(
                text="0.0", pos=(1.25, 0.7), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.Va_display = OnscreenText(
                text="0.0", pos=(1.25, 0.75), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.R_display = OnscreenText(
                text="0.0", pos=(1.25, 0.8), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.NextMark_display = OnscreenText(
                text="0.0", pos=(1.25, 0.85), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        self.Vw_display = OnscreenText(
                text="0.0", pos=(1.25, 0.9), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)






        
        
        
        
        
        


        # HDG angle display, fixed (easy) number (by MASA, 04/17/2020)
        self.hdg_display = OnscreenText(
                text="0.0", pos=(1.65, 0.29), scale=0.07,
                fg=(0,1,0,0.7), align=TextNode.ACenter,
                parent=self.fr)

        # gui element for the tiller (steer the rudder)
        self.tillertxt_display = OnscreenText(
                text="-0.5       tiller       0.5", pos=(1.25, 0.04), scale=0.05,    #(by MASA, 04/17/2020)
                fg=(1,1,1,0.7), align=TextNode.ACenter,
                parent=self.fr)
        self.tiller_gui = DirectSlider(
            pos=Vec3(-0.1, -0.9), scale=0.2,                 #(by MASA, 04/17/2020)
            value=0.0, range=(-0.5,0.5), pageSize=0.05)

        # gui element for the mainsheet (control the sail)
        self.sailtxt_display = OnscreenText(
                text="0.0         sail         1.0", pos=(0.75, 0.04), scale=0.05,  #(by MASA, 04/17/2020)
                fg=(1,1,1,0.7), align=TextNode.ACenter,
                parent=self.fr)
        self.mainsheet_gui = DirectSlider(
            pos=Vec3(-0.6, -0.9), scale=0.2,                  #(by MASA, 04/17/2020)
            value=1.0, range=(0.05, 1.0), pageSize=0.05)
        
        # gui element for the Speed (by MASA, 04/17/2020)
        self.sailtxt_display = OnscreenText(
                text="Speed", pos=(0.25, 0.04), scale=0.05,  #(by MASA, 04/17/2020)
                fg=(1,1,1,0.7), align=TextNode.ACenter,
                parent=self.fr)
        
        # gui element for the HDG (by MASA, 04/17/2020)
        self.sailtxt_display = OnscreenText(
                text="HDG", pos=(1.65, 0.04), scale=0.05,  #(by MASA, 04/17/2020)
                fg=(1,1,1,0.7), align=TextNode.ACenter,
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
        self.compass_rose.setPos(Vec3(0.3,-0.65))    #(by MASA, 04/17/2020)
        
        # boat icon
        boat = LineSegs("boat")
        boat.setColor(LColor(.7,.7,.7,1))
        boat.setThickness(3.0)

        boat.moveTo(Vec3(0.0,0.0))          #(by MASA, 04/17/2020)
        boat.drawTo(Vec3(0.0, 0.35))        #(by MASA, 04/17/2020)
        boat.moveTo(Vec3(-0.2,0.0))        #(by MASA, 04/17/2020)
        boat.drawTo(Vec3(0.2,-0.0))         #(by MASA, 04/17/2020)

        self.compass_boat = NodePath("compass boat")
        self.compass_boat.attachNewNode(boat.create())

        self.compass_boat.reparentTo(myRender2d)

        self.compass_boat.setScale(0.2)
        self.compass_boat.setPos(Vec3(0.3,-0.65))    #(by MASA, 04/17/2020)

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
        self.compass_wind.setPos(Vec3(0.3,-0.65))    #(by MASA, 04/17/2020)

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
        self.compass_sail.setPos(Vec3(0.3,-0.65))    #(by MASA, 04/17/2020)
        

        '''
        # Reference Indicator
        ref_indicator = LineSegs("ref_indicator")
        ref_indicator.setColor(LColor(1, 0.2, 0.2, 0.5))
        ref_indicator.setThickness(2.0)

        pnt1 = Vec3(cos(pi/3),sin(pi/3))
        pnt2 = Vec3(-cos(pi/3),-sin(pi/3))
        pnt3 = Vec3(-cos(pi/3),sin(pi/3))
        pnt4 = Vec3(cos(pi/3),-sin(pi/3))
        pnt5 = Vec3(cos(0.4*pi),sin(0.4*pi))
        pnt6 = Vec3(-cos(0.4*pi),-sin(0.4*pi))
        pnt7 = Vec3(-cos(0.4*pi),sin(0.4*pi))
        pnt8 = Vec3(cos(0.4*pi),-sin(0.4*pi))
        ref_indicator.moveTo(pnt1)
        ref_indicator.drawTo(pnt2)
        ref_indicator.moveTo(pnt3)
        ref_indicator.drawTo(pnt4)
        ref_indicator.moveTo(pnt5)
        ref_indicator.drawTo(pnt6)
        ref_indicator.moveTo(pnt7)
        ref_indicator.drawTo(pnt8)

        self.compass_ref = NodePath("compass ref")
        self.compass_ref.attachNewNode(ref_indicator.create())

        self.compass_ref.reparentTo(myRender2d)

        self.compass_ref.setScale(1)
        self.compass_ref.setPos(Vec3(0.3,-0.65))    #(by MASA, 04/17/2020)
        '''
        

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

        # list of race marks
        # (type: '+'/'-' : round clockwise, counterclockwise
        #        'a'     : avoid (penalty)
        #        's'     : start
        #        'f'     : finish
        #  name:         : label for mark
        #  info:         : textual info
        #  xy            : position)
        self.marklist = marklist
        
    def update(self, x, y, psi, V, psiw, Vw, ds, others, race_events=None):
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
        
        
        # Define Port/Starbo (by MASA, 04/18/2020)
        if psi < 0:
            psi = psi + 360
            
        if psiw < 0:
            psiw = psiw + 360
            
        # Calculate the AOA real wind speed(Va)/direction(psia) (by MASA, 04/18/2020)
        if psiw > 180:
            AOA = 360 - psiw
        else:
            AOA = psiw
        
        Va = sqrt(V ** 2 + Vw ** 2 - 2 * V * Vw * cos(radians(AOA)))
        if psiw <= 180:
            psia = psi + (AOA + degrees(acos((Va ** 2 + Vw ** 2 - V ** 2)/(2 * Va * Vw + 0.001))))
        else:
            psia = psi - (AOA + degrees(acos((Va ** 2 + Vw ** 2 - V ** 2)/(2 * Va * Vw + 0.001))))
            
        if psia < 0:
            psia = psia + 360
            
        psia_2 = degrees(acos((Va ** 2 + Vw ** 2 - V ** 2)/(2 * Va * Vw + 0.001)))
            
        if (-90 <= psi - psia <= 0) or (270 <= psi - psia < 360):
            Mode = 1            # Starbo/UPwind
        elif (-180 <= psi - psia < -90) or (180 <= psi - psia < 270):
            Mode = 2            # Starbo/DOWNwind
        elif (-360 < psi - psia <= -270) or (0 < psi - psia <= 90):
            Mode = 3            # Port/UPwind
        else:
            Mode = 4            # Port/DOENwind
                
        
        
        

        #print("BoatDir: %.2f" % psi,"  Wind(dir,V): %.2f, %.2f" %(psiw, Vw))

        # update the compass rotation, and set the speed text
        self.compass_rose.setHpr(0, 0, -psi)
        self.speed_display.text = "{:2.1f}".format(V)
        self.sail_display.text = "{:2.0f}" .format(ds)
        #self.ref_display.text = "ref:{:2.0f} deg" .format(abs(psiw-30))
        self.tiller_display.text = "{:2.0f}" .format(psi)
        self.hdg_display.text = "{:2.0f}" .format(psi)
        # update the wind indicator on compass rotation
        self.compass_wind.setHpr(0,0, psiw) # get the true wind direction
        self.compass_wind.setScale(0.1+0.007*Vw)

        # update the sail indicator on compass rotation
        self.compass_sail.setHpr(0,0, 180+ds) # get the true sail direction
        self.compass_sail.setScale(0.2)
        
        # update the ref indicator on compass rotation
        #self.compass_ref.setHpr(0,0, 180+ds) # get the true ref direction
        #self.compass_ref.setScale(0.2)
        
        # map position unit conversion
        map_scale_x = 3000
        map_scale_y = 4000
        map_x = self.map_origin[1] + y/map_scale_x
        map_y = self.map_origin[2] + x/map_scale_y

         # update the boat pose on map
        self.display_boat.setHpr(0,0,psi)
        self.display_boat.setPos(Vec3(map_x,map_y))
        #print("x: %.4f" %map_x,"  y: %.4f" %map_y)

        # update marks position on map
        start_x = self.map_origin[1] - 650 /map_scale_x     # (by MASA, 04/18/2020)
        start_y = self.map_origin[2] - 200 /map_scale_y     # (by MASA, 04/18/2020)
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
        print("map_x: %.4f" %map_x,"  map_y: %.4f" %map_y)
        
        #print("marklist len: ", len(self.marklist))
        """
        if len(self.marklist) != 0:
            #print("marklist[0]: ", self.marklist[0])
            print("mark1 info: ", self.marklist[1])
            for i in self.marklist:
                #print("mark1 position: ", self.marklist[1][3])
                m_pos1 = self.marklist[1][3]
                #print("mark2 position: ", self.marklist[2][3])
                m_pos2 = self.marklist[2][3]
                #print("mark3 position: ", self.marklist[3][3])
                m_pos3 = self.marklist[3][3]
                #print("mark4 position: ", self.marklist[4][3])
                m_pos4 = self.marklist[4][3]
            print ("boat position: %.2f , %.2f" %(x, y))
            print ("relative position to mark1: %.2f , %.2f" %(self.marklist[1][3][0]-x, self.marklist[1][3][1]-y))
            print ("distance to mark1: %.2f" %(sqrt(pow(self.marklist[1][3][0]-x,2)+pow(self.marklist[1][3][1]-y,2))))
                
        """
        
        # Define Next Mark and calculate the direction(psim)/range(R)/relative speed(Vr) to the next mark (by MASA, 04/18/2020)
        #if sqrt(((x - start_x) * map_scale_x)**2 + ((y - start_y) * map_scale_y)**2) < 50:
        #    NextMark = 1
        
        NextMark = 1
        if NextMark == 2:
            Mark_x = mark2_x
            Mark_y = mark2_y
        elif NextMark == 3:
            Mark_x = mark3_x
            Mark_y = mark3_y
        elif NextMark == 4:
            Mark_x = goal_x
            Mark_y = goal_y
        elif NextMark == 5:
            Mark_x = 0
            Mark_y = 0
        else:
            NextMark = 1
            Mark_x = mark1_x
            Mark_y = mark1_y
            
            
        R = sqrt(((x - Mark_x) * map_scale_x)**2 + ((y - Mark_y) * map_scale_y)**2)
        around = 15

        if NextMark == 1 and R > around:
            NextMark = 1
            #self.NextMark_display.text = "Go to Mark 1"
        elif (NextMark == 1 and R <= around) or (NextMark == 2 and R > around):
            NextMark = 2
            #self.NextMark_display.text = "Go to Mark 2"
        elif (NextMark == 2 and R <= around) or (NextMark == 3 and R > around):
            NextMark = 3
            #self.NextMark_display.text = "Go to Mark 3"
        elif (NextMark == 3 and R <= around) or (NextMark == 4 and R > around):
            NextMark = 4
            #self.NextMark_display.text = "Go to Finish line"
        else:
            NextMark = 5
            #self.NextMark_display.text = "FINISH!!!!"
        
        theta = 180 - (degrees(atan((Mark_y - y)/(Mark_x - x))) - 90) - psi
        if theta < 0:
            psim = 360 + theta
        elif theta >= 360:
            psim = theta - 360
        else:
            psim = theta
        
        Vr = V * cos(radians(theta))
        
        
        #Calculate best heading(psi_VMG)/mainsheet(ds_VMG) for getting VMG
        TackAngle = 40 * 2
        if (abs(psia - psim) <= TackAngle/2) or (abs(psia - psim) >= (360 - TackAngle/2)):       # UPwind(close-hauled)
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
            self.information_display.text = "First set the sail = 0. Then control the tiller."
        elif Mode == 2 or Mode == 4:        # DownWind(reaching)
            ds_VMG_l = 60
            ds_VMG_r = -60
            psi_VMG_l = psim
            psi_VMG_r = psim
            self.information_display.text = "First set the tiller to the Mark. Then control the sail."
        else:                               # Abeam
            ds_VMG_l = AOA - 30
            ds_VMG_r = AOA + 30
            psi_VMG_l = psim
            psi_VMG_r = psim
            self.information_display.text = "First set the tiller to the Mark. Then control the sail."
            
        self.Vr_display.text = "{:2.1f}" .format(Vr)
        self.ds_VMG_l_display.text = "{:2.0f}" .format(ds_VMG_l)
        self.ds_VMG_r_display.text = "{:2.0f}" .format(ds_VMG_r)
        self.psi_VMG_l_display.text = "{:2.0f}" .format(psi_VMG_l)
        self.psi_VMG_r_display.text = "{:2.0f}" .format(psi_VMG_r)
        self.psia_display.text = "{:2.0f}" .format(psia)
        self.psim_display.text = "{:2.0f}" .format(psim)
        self.Mode_display.text = "{:2.0f}" .format(Mode)
        self.Mode_display.text = "{:2.0f}" .format(Mode)
        self.AOA_display.text = "{:2.0f}" .format(AOA)
        self.psia_2_display.text = "{:2.0f}" .format(psia_2)
        self.theta_display.text = "{:2.0f}" .format(theta)
        #self.psiw_display.text = "{:2.0f}" .format(psiw)
        self.Va_display.text = "{:2.1f}" .format(Va)
        self.R_display.text = "{:2.1f}" .format(R)
        self.NextMark_display.text = "{:2.1f}" .format(NextMark)
        self.Vw_display.text = "{:2.1f}" .format(Vw)
            
        
        
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

            self.hud.update(x, y, psi, V, psiw, Vw, ds, others)
            
            return Task.cont
    
    # create and run
    testhud = TestHud()
    testhud.run()