#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 15:16:21 2019

@author: repa
@licence: GPL-v3.0
"""

# panda3d stuff
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import Vec3, WindowProperties, CardMaker, Texture
from communicator import Communicator
from configparser import ConfigParser
from odegrid import terrainGeom

# head-up display and controls
from hud import Hud

# generic utilities
import copy

# dynamic simulation with ode
import ode

# numpy and scipy for cl/cd table
import numpy as np
from scipy import interpolate
import csv

# imaging for 'licence plates'
from PIL import Image, ImageDraw, ImageFont

# generic os interaction
import os
from io import BytesIO

from math import pi, sin, cos, sqrt

# weight of the world....
g0 = 9.813

def phithetapsiToQuaternion(phi, tht, psi):
    '''
    Create an ODE-compatible quaternion from Euler-Rodriquez angles

    Parameters
    ----------
    phi : float [rad]
        Roll angle.
    tht : float [rad]
        Pitch angle.
    psi : float [rad]
        Yaw angle.

    Returns
    -------
    4-tuple of float
        Quaternion describing attitude.
        
    '''
    return (np.cos(0.5*phi)*np.cos(0.5*tht)*np.cos(0.5*psi) +
            np.sin(0.5*phi)*np.sin(0.5*tht)*np.sin(0.5*psi),
            np.sin(0.5*phi)*np.cos(0.5*tht)*np.cos(0.5*psi) -
            np.cos(0.5*phi)*np.sin(0.5*tht)*np.sin(0.5*psi),
            np.cos(0.5*phi)*np.sin(0.5*tht)*np.cos(0.5*psi) +
            np.sin(0.5*phi)*np.cos(0.5*tht)*np.sin(0.5*psi),
            np.cos(0.5*phi)*np.cos(0.5*tht)*np.sin(0.5*psi) -
            np.sin(0.5*phi)*np.sin(0.5*tht)*np.cos(0.5*psi))


class Wind:
    """
    Constant wind speed and direction model

    Maybe to be updated with variable + location-dependent wind?
    This uses a North - East - Down axis system
    """
    def __init__(self, spd = (-2, -4, 0)):
        '''
        Create a wind object

        Parameters
        ----------
        spd : 3-tuple of float
            Wind speed represented as a vector. This indicates the direction
            of the wind, e.g. (5, 0, 0) means 5 m/s to the north, which is
            a wind from the south. The default is (-2, -4, 0).

        Returns
        -------
        None.

        '''
        self._speed = np.array(spd)
        
    def speed(self, loc):
        '''
        Return wind speed

        @param loc   Location 
        '''
        return self._speed

    
class IceSailer:
    """
    Ice sailing dynamics with ODE
    """

    # global time step
    dt_max = 1.0/120.0
    
    # maximum mainsheet angle
    ds_max = 1.0
    
    def __init__(self, world, space, x=(0,0,0), psi=0) -> None:
        """
        Create a new ice sailer craft

        @param world    ODE dynamics/collision world in which to create 
                        the craft
        @param space    3d space for the craft's geometric objects    
        @param x        Initial position vector for the craft, x=north, 
                        y=east, z=down, origin of the body is at cg center 
        @param psi      Initial heading of the craft, degrees
        """

        # helper params, base properties
        self.length = length = 5.3
        self.xcg = xcg = 2.8
        self.xmast = xmast = 2
        mastheight = 6
        self.mastbase = mastbase = 0.8
        width = 5.0
        self.skate_width = skate_width = 2.3
        mass = 250
        skate_radius = 0.3
        
        self.skate = []
        self.trans = []
        self.psi = np.radians(psi)  # initial heading
        self.gamma = 0              # initial relative wind
        self.Vw = 0                 # initial relative wind speed
        self.dr = 0.0               # rudder steering
        self.ds = IceSailer.ds_max  # mainsheet steering (max angle)
        self._ds = 0                # current sail angle
        self.S = 6                  # m2 of sail surface??
        self.arm = 0.5              # force on sail attaches 0.5 m aft of mast  
        self.xmast = xmast          # mast position forward of (000) datum
        self.zmast = -mastbase - 0.2*mastheight # force sail z
        self.V = 0                  # total speed
        
        # first step, create a composite body
        self.body = ode.Body(world)
        m = ode.Mass()

        # approximate, to get inertia?
        m.setBoxTotal(mass, length, width, mastbase)
        print('mass',  m)
        self.body.setMass(m)

        # positions of the skates
        self.xscates = \
            ( ((length-xcg, 0, mastbase-skate_radius), "front"), 
              ((     -xcg, -skate_width, mastbase-skate_radius), "left"), 
              ((     -xcg,  skate_width, mastbase-skate_radius), "right") )

        # each skate is a sphere in ODE, the contact dynamics will
        # later create the skating direction
        for (s, name) in self.xscates:
            self.skate.append(ode.GeomSphere(None, skate_radius))
            self.skate[-1].nam = name
            self.trans.append(ode.GeomTransform(space))
            self.trans[-1].nam = "t_" + name

            # define callbacks on the geometry, these give the lateral/
            # longitudinal direction of the skate
            if s[1]:
                self.trans[-1].align = self.heading
            else:
                self.trans[-1].align = self.steer
            self.trans[-1].setGeom(self.skate[-1])
            self.skate[-1].setPosition(s)
            self.trans[-1].setBody(self.body)
            
        # the body has the three skates added to it now.

        # forward beam is simplified
        # initially aligned along the z axis
        bar_radius = 0.2
        self.hull = ode.GeomCapsule(None, bar_radius, length-2*bar_radius)
        self.hull.nam = "hull"
        self.trans.append(ode.GeomTransform(space))
        self.trans[-1].setGeom(self.hull)
        self.trans[-1].nam = 't_hull'

        # rotate the beam 90 deg along y axis
        q = ( np.cos(np.pi/4.0), 0, np.sin(np.pi/4), 0)
        self.hull.setQuaternion(q)

        # and shift it forward/backward to match cg
        self.hull.setPosition((0.5*length-xcg, 0, -0.0*mastbase))

        # attach it to the body
        self.trans[-1].setBody(self.body)

        # cross beam also simplified
        self.hullc = ode.GeomCapsule(None, bar_radius, width-2*bar_radius)
        self.hullc.name = "hullc"
        self.trans.append(ode.GeomTransform(space))
        self.trans[-1].setGeom(self.hull)
        self.trans[-1].nam = 't_hullc'

        # rotate 90 deg along x axis
        q = ( np.cos(np.pi/4.0), np.sin(np.pi/4), 0, 0)
        self.hullc.setQuaternion(q)

        # and shift it backward to match cg
        self.hull.setPosition((-xcg, 0, -0.0*mastbase))

        # attach it to the body
        self.trans[-1].setBody(self.body)
        
        # fix the mast
        self.mast = ode.GeomCylinder(None, bar_radius, mastheight-2*bar_radius)
        self.mast.nam = "mast"
        self.trans.append(ode.GeomTransform(space))
        self.trans[-1].setGeom(self.mast)
        self.trans[-1].nam = 't_mast'

        # move it up by 0.5 height + the base height
        self.mast.setPosition((self.xmast-xcg, 0, -0.5*mastheight))

        # attach to the body
        self.trans[-1].setBody(self.body)

        # put the body at a specific position
        self.body.setPosition(x)
        try:
            self.body.setQuaternion(
                phithetapsiToQuaternion(psi[0],0.5*psi[1],psi[2]))
        except TypeError:
            self.body.setQuaternion([np.cos(0.5*self.psi), 0, 0, 
                                     np.sin(0.5 * self.psi)]) 

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
        

    def heading(self):
        """return the current orientation vector of the craft, 
        for the purpose of calculating side forces on the rear skates"""
        
        R = self.body.getRotation()
        
        if self.doprint == 0:
            print("skate orient", np.array((R[0], R[3], R[6])))
            
        return np.array((R[0], R[3], R[6]))
    
    def steer(self):
        """
        Orientation of the front scate

        return the current orientation vector of the front skate
        for the purpose of calculating side forces.
        """
        
        R = self.body.getRotation()
        nose = np.array((R[0], R[3], R[6]))
        cosa = np.cos(self.dr)
        sina = np.sin(self.dr)
        rot = np.array( ((cosa, -sina, 0), 
                         (sina, cosa, 0), 
                         (0, 0, 0) ) )
        if self.doprint == 0:
            print("nose orient", rot @ nose)
        return rot @ nose

    def force(self, wind):
        """
        Calculate the wind force on the sail

        @param wind:       wind parameters
        """

        # get the horizontal speed as a vector, in world coordinates
        spd = np.array(self.body.getLinearVel())

        # get wind speed calculate relative speed, world coordinates
        rspd = wind.speed(self.body.getPosition()) - spd
        rspd[2] = 0

        # total speed vector size and "heading"
        V = np.linalg.norm(rspd)
        self.Vw = 3600.0/1852.0*V
        self.V = 3600.0/1852.0*np.linalg.norm(spd)
        if V < 1.0E-8: 
            # don't continue, don't apply force, and return
            return
        
        # convert the relative speed vector to body coordinates
        rspd_b = self.body.vectorFromWorld(rspd)

        if self.doprint == 0:
            print("windspeed", wind.speed(self.body.getPosition()),
                  "ownspeed", self.body.getLinearVel(),
                  "\nrel", rspd, "relb", rspd_b)
       
        # relative wind angle
        Psi_wr = np.arctan2(-rspd_b[1], -rspd_b[0])
        self.gamma = Psi_wr

        # angle of attack 
        alpha = self.alpha = Psi_wr - self._ds
        if alpha > np.pi: alpha -= np.pi
        if -alpha < -np.pi: alpha += np.pi
        
        # lift and drag coefficients. Correct alpha for range
        cl = interpolate.splev(abs(alpha)/np.pi*180, self.cl_alpha)
        cd = interpolate.splev(abs(alpha)/np.pi*180, self.cd_alpha) 

        # dynamic pressure, and from there drag and lift
        qS = 0.5 * 1.225 * V*V * self.S
        D = qS * (cd + 0.05)
        L = qS * cl
        
        # let the sail follow the wind force/aoa
        # incorrect!
        self._ds += min(abs(L)*0.00005, 0.01)*np.sign(alpha)
        if self._ds > self.ds:
            self._ds = self.ds
        elif self._ds < -self.ds:
            self._ds = -self.ds
        
        if alpha < 0: L = -L

        if self.doprint == 0:
            print("gamma", round(np.degrees(Psi_wr)),
                  "ds", round(np.degrees(self.ds)),
                  "_ds", round(np.degrees(self._ds)),
                  "alpha",  round(np.degrees(alpha)),
                  "D", D, "L", L)

        # Lift and drag are defined relative to the wind axes
        # this transforms the (D, L) vector to world coordinates
        wtrans = np.matrix( ( ( rspd[0]/V, -rspd[1]/V, 0), 
                              ( rspd[1]/V,  rspd[0]/V, 0),
                              (     0,          0,    1) ) )
        Wforce = wtrans * np.matrix ( ((D,), (L,) ,(0,)) )
        if self.doprint == 0:
            print("Wind force", Wforce[0,0], Wforce[1,0])

        # apply the force
        self.body.addForceAtRelPos(
            Wforce, 
            (self.xmast - self.arm*np.cos(self._ds), 
             -self.arm*np.sin(self._ds), self.zmast))
        
    def copyState(self):
        return (self.body.getPosition(), self.body.getQuaternion(), 
                self.body.getLinearVel(), self.body.getAngularVel(), 
                self.dr, self._ds)
    
        
    def updateTiller(self, value):
        '''
        Update tiller input
        
        @param value new input
        '''
        self.dr = max(min(1.0, value), -1.0)


    def updateMainsheet(self, value):
        '''
        Update mainsheet input
        
        @param value new input
        '''
        self.ds = max(min(1.0, value), 0.05)
        
        
    def newObstacle(self, name, otype, coords):
        '''
        Place a new fixed-obstacle geom in the world. 

        Parameters
        ----------
        name : str
            Descriptive label for the obstacle.
        otype : str
            Obstacle type, currently implemented 'sphere', 'capsule', 
            'cylinder' and 'box'.
        coords : numpy array
            Coordinates, x, y, z, sx, sy, xz, phi, theta, psi.

        Returns
        -------
        None.

        '''
        global fixed_obstacles
        if otype == 'sphere':
            ngeom = ode.GeomSphere(space, coords[3])
        elif otype == 'box':
            ngeom = ode.GeomBox(space, lengths=coords[3:6])
        elif otype == 'capsule':
            ngeom = ode.GeomCapsule(space, radius=coords[4], length=coords[3])
        elif otype == 'cylinder':
            ngeom = ode.GeomCylinder(space, radius=coords[4], length=coords[3])
        ngeom.setPosition(coords[:3])
        ngeom.setQuaternion(phithetapsiToQuaternion(*(np.radians(coords[6:]))))
        ngeom.nam = name
        print(f"new obstacle {name}, type {otype}, at {coords}")
        fixed_obstacles.append(ngeom)
        
def loadCraft(self, name, render, loader):
    ''' 
    Helper function to load icecraft models
    '''

    # basic frame
    try:
        self.frame = loader.loadModel(f"blender/frame-{name}.egg")
    except OSError:
        print(f"Cannot load frame-{name}, using default frame")
        self.frame = loader.loadModel(f"blender/frame.egg")    
    self.frame.reparentTo(render)

    # skates
    try:
        self.skate = loader.loadModel(f"blender/skate-{name}.egg")
    except OSError:
        print(f"cannot load skate-{name}, using default skate")
        self.skate = loader.loadModel("blender/skate.egg")

    # mast and sail
    try:
        self.mast = loader.loadModel(f"blender/mast-and-sail-{name}.egg")
    except OSError:
        print(f"cannot load skate-{name}, using default skate")
        self.mast = loader.loadModel("blender/mast-and-sail.egg")

    # attach mast to frame    
    self.mast.reparentTo(self.frame)
    self.mast.setPos(self.xmast-self.xcg, 0, -0.3)
    
    # triple to three skates
    self.skates = (copy.copy(self.skate), copy.copy(self.skate))
    
    # and attach to frame
    self.skate.reparentTo(self.frame)
    self.skate.setPos(self.length-self.xcg, 0, -0.5)
    self.skates[0].reparentTo(self.frame)
    self.skates[0].setPos(-self.xcg, -self.skate_width, -0.5)
    self.skates[1].reparentTo(self.frame)
    self.skates[1].setPos(-self.xcg, self.skate_width, -0.5)

    # license plate
    global platemaker
    self.licenseplate = render.attachNewNode(platemaker.generate())
    
    # use PIL to load the base plate and write team name
    img = Image.open('PS-plate.png').convert('RGBA')
    fnt = ImageFont.truetype('steelfish rounded bd.ttf', 200)
    d = ImageDraw.Draw(img)
    d.text((220,60), name, font=fnt, fill=(21,34,71,255))
    
    # use that to create a Panda3D texture
    tex = Texture('licenseplate')
    tex.setup2dTexture(img.size[0], img.size[1], 
                       Texture.TUnsignedByte, Texture.FRgba)
    tex.setRamImageAs(img.tobytes(), 'RGBA')
    
    # attach to the license plate, and locate the plate on the rear
    self.licenseplate.setTexture(tex)
    self.licenseplate.reparentTo(self.frame)
    self.licenseplate.setPos(-3, 0, 0)
    self.licenseplate.setH(-90)
  
        
class OtherCraft(IceSailer):
    
    def __init__(self, name, index, *args, **kwargs):
        global craft
        super().__init__(*args, **kwargs)
                
        self.name, self.index = name, index
        loadCraft(self, name, craft.render, craft.loader)
        self.body.setPosition((0.0, 100.0*index, 100))
        self.body.disable()
        
    def force(self, wind):
        return
        
    def updateMainsheet(self, value):
        return
        
    def follow(self, xrem, qrem, vrem, wrem, dr, ds):
        self.body.setPosition(xrem)
        self.body.setQuaternion(qrem)
        self.body.setLinearVel(vrem)
        self.body.setAngularVel(wrem)
        self.dr = dr
        self._ds = ds
        if not self.body.isEnabled():
            print('first enable')
            print(f"Start of {self.index}:{self.name} at {xrem}")
            self.body.enable()
        
    def updateCoordinates(self):
        """
        Update Panda data to reflect ODE simulation
        """
        qW, qx, qy, qz = self.body.getQuaternion()
        x, y, z = self.body.getPosition()
        self.frame.setPosQuat((x, -y, -z), (qW, qx, -qy, -qz))
        self.psi = np.arctan2(
            2.0*qx*qy + qW*qz,
            qW*qW + qx*qx - qy*qy - qz*qz) 
        self.skate.setH(np.degrees(-self.dr))
        self.mast.setH(np.degrees(-self._ds))

class StaticObject:
    
    def __init__(self, name, coord):
        print(f"creating static object {name}")
        self.object = craft.loader.loadModel(f"blender/{name}.egg")
        self.object.reparentTo(craft.render)
        self.object.setPos((coord[0], -coord[1], -coord[2]))
        self.object.setH(-coord[3])

class MyCraft(ShowBase,IceSailer):
    '''
    Visual representation of own craft
    
    Inherits from the IceSailer ODE dynamics, and adds visual representation &
    interaction with Panda3d
    '''
    
    def __init__(self, world, space, x, psi, name='Anon.'):
        ''' 
        Create the ownship

        @param world   ODE world
        @param space   ODE contact/collision space
        @param x       initial craft cg position
        @param psi     initial heading (1 value), or initial (phi, theta, psi)
        @param name    Name/label of the craft
        '''

        ShowBase.__init__(self)
        IceSailer.__init__(self, world, space, x, psi)

        self.comm = None
        # additional objects in the world
        self.objects = []
        
        # window name
        props = WindowProperties()
        props.setTitle("CSE/EID iceboat simulation AERSP597")
        self.win.requestProperties(props)
                           
        # load the skydome
        self.dome = self.loader.loadModel("blender/skydome.egg")
        scale = 5000.0
        self.dome.setScale(scale, scale, scale)
        self.dome.setBin('background', 1)
        self.dome.setDepthWrite(0)
        self.dome.reparentTo(self.render)
        
        # Load the environment model.
        self.scene = self.loader.loadModel("blender/terrain.egg")
        self.scene.reparentTo(self.render)

        # Add the skateAlong procedure to the task manager.
        self.taskMgr.add(self.skateAlong, "skateAlong")
        self.lasttime = 0

        # use this helper function to load the 3D models
        loadCraft(self, name, self.render, self.loader)
        
        # list with events on sail progress
        self.eventlist = []

        # create the hud
        self.hud = Hud(self, marklist)

        # follow distance to stay behind the craft
        self.followdist = 30
                
        # make sure the world reflects the ODE coordinates
        self.updateCoordinates()

        # initial camera position
        self.resetCamera()
        
        
        
        
    def newStaticObject(self, name, coord):
        new_object = StaticObject(name, coord)
        self.objects.append(new_object)
        
    def newOtherCraft(self, name, index):
        return OtherCraft(name, index, world, space)
        
    def updateCoordinates(self):
        """
        Update Panda data to reflect ODE simulation
        """
        global othercraft

        qW, qx, qy, qz = self.body.getQuaternion()
        x, y, z = self.body.getPosition()
        self.frame.setPosQuat((x, -y, -z), (qW, qx, -qy, -qz))
        self.psi = np.arctan2(
            2.0*qx*qy + qW*qz,
            qW*qW + qx*qx - qy*qy - qz*qz) 
        self.skate.setH(np.degrees(-self.dr))
        self.mast.setH(np.degrees(-self._ds))

        if self.doprint == 0:
            print("position", self.body.getPosition(), 
                  "heading", np.degrees(self.psi))
            self.doprint = 60
        self.doprint -= 1
        
        # also call the hud with the information
        tiller, mainsheet = self.hud.update(
            x, y, np.degrees(self.psi), self.V,
            np.degrees(self.gamma), self.Vw, np.degrees(self._ds), 
            [o.body.getPosition()[:2] for o in othercraft.values()],
            self.eventlist)
        #print("user input(tiller,mainsheet):.%2f,%.2f" %(tiller,mainsheet))
        # and set returned control values
        #tiller = tiller + 0.1
        #mainsheet = mainsheet - 0.1
        self.updateTiller(tiller)
        self.updateMainsheet(mainsheet)
        #print("mod input(tiller,mainsheet):.%2f,%.2f" %(tiller,mainsheet))

        # Show info
        if len(marklist) != 0:
            #print("marklist[0]: ", marklist[0])
            print("mark1 info: ", marklist[1])
            for i in marklist:
                #print("mark1 position: ", marklist[1][3])
                m_pos1 = marklist[1][3]
                #print("mark2 position: ", marklist[2][3])
                m_pos2 = marklist[2][3]
                #print("mark3 position: ", marklist[3][3])
                m_pos3 = marklist[3][3]
                #print("mark4 position: ", marklist[4][3])
                m_pos4 = marklist[4][3]
            print ("boat position: %.2f , %.2f" %(x, y))
            print ("relative position to mark1: %.2f , %.2f" %(marklist[1][3][0]-x, marklist[1][3][1]-y))
            print ("distance to mark1: %.2f" %(sqrt(pow(marklist[1][3][0]-x,2)+pow(marklist[1][3][1]-y,2))))




        if self.comm:
            data = np.zeros((15,), dtype=np.float32)
            data[:3] = self.body.getPosition()
            data[3:7] = self.body.getQuaternion()
            data[7:10] = self.body.getLinearVel()
            data[10:13] = self.body.getAngularVel()
            data[13] = self.dr
            data[14] = self._ds
            self.comm.update(data)
        
    def skateAlong(self, task):
        '''
        Callback routine to update simulation
        
        @param task   Panda3d task information
        '''

        # all craft to be updated
        global craft, othercraft, wind, space, \
            contactgroup, coll_callback
        
        dt = task.time - self.lasttime
        if np.abs(dt - 2*IceSailer.dt_max) > 0.5*IceSailer.dt_max:
            #print("strange time jump", dt)
            pass

        self.lasttime = task.time
        for i in range(2):
            # owncraft wind force
            craft.force(wind)

            # calculate collisions
            space.collide( (world, contactgroup), coll_callback)

            # update the world
            world.step(IceSailer.dt_max)

            # clear contacts for next round
            contactgroup.empty()


        # move all objects around according to the ODE result
        craft.updateCoordinates()
        for k, c in othercraft.items():
            c.updateCoordinates()
        
        # update camera position
        dist = self.frame.getPos() - self.campos
        tomove = dist.length() - self.followdist
        if tomove > 0:
            dx, dy, dz = dist * (0.1*tomove)
            self.campos[0] += dx
            self.campos[1] += dy
        self.camera.setPos(self.campos[0], self.campos[1], 5)
        chi = np.arctan2(dist[1], dist[0])
        self.camera.setHpr(np.degrees(chi)-90, -10, 0)

        # tell the simulation to continue
        return Task.cont
    
    def setCommunicator(self, comm):
        self.comm = comm
        
    def resetCamera(self):
        x, y, z = self.body.getPosition()
        qW, qx, qy, qz = self.body.getQuaternion()
        self.psi = np.arctan2(
            2.0*qx*qy + qW*qz,
            qW*qW + qx*qx - qy*qy - qz*qz)
        x -= np.cos(self.psi) * 0.8*self.followdist
        y -= np.sin(self.psi) * 0.8*self.followdist
        self.campos = Vec3(x, -y, 5)
        self.camera.setPos(self.campos[0], self.campos[1], self.campos[2])
        self.camera.setHpr(np.degrees(self.psi)-90, 0, 0)

# collision callback function
def coll_callback(args, geom1, geom2):
    '''
    Collision callback function

    This creates collision contacts, and sets their parameters
    '''
    contacts = ode.collide(geom1, geom2)
    if not contacts:
        return

    if geom2.nam != "ground":
        print("%i collisions %s %s" % 
              (len(contacts), geom1.nam, geom2.nam))
        
    orient = None
    try:
        orient = geom1.align()
    except AttributeError:
        try:
            orient = geom2.align()
        except AttributeError:
            pass
        
    world, contactgroup = args
    for c in contacts:
        if orient is not None:
            # skating contact, small friction along the orientation, large
            # across
            # print("orientation %s" % orient)
            c.setFDir1(orient)
            c.setMu(0.001)
            c.setMu2(1000)
            c.setMode(c.getMode() | ode.ContactMu2 | ode.ContactFDir1| ode.ContactApprox1)
            c.setSlip2(0)
            c.setSlip1(0)
        else:
            # non-skating, uniform mu
            c.setMu(10)

        c.setBounce(0.02)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

if __name__ == '__main__':

    config = ConfigParser()
    config.read('iceboat.conf')
    serverurl = config.get('server', 'url', fallback=None)
    name = config.get('player', 'name', fallback='anonymous')
    
    # ode world
    world = ode.World()
    world.setERP(0.8)
    world.setCFM(1E-5)
    wind = Wind((-2, 5, 0))
    world.setGravity((0, 0, g0))
    
    # flat ground plane
    space = ode.Space()
    ground = ode.GeomPlane(space, (0, 0, -1), 0)
    ground.nam = "ground"
    
    # terrain as obstacle
    terrain = terrainGeom('blender/terrain.egg', space)[0]
    terrain.nam = "terrain"
    
    # a dictionary for other craft in the world
    othercraft = dict()

    # a list for the race mark information
    marklist = list()

    # 'license' plates
    platemaker = CardMaker('licenseplate')
    platemaker.setUvRange((0,1), (1,0))
    platemaker.setFrame(-1, 1, 0, 1)

    # get license plate font
    if not os.path.isfile('steelfish rounded bd.ttf'):
        import requests
        import zipfile
        with requests.get(
            'https://dl.1001fonts.com/steelfish-rounded.zip') as fontfile:
            zf = zipfile.ZipFile(BytesIO(fontfile.content), mode='r')
            zf.extract('steelfish rounded bd.ttf')
    
    # get license plate background
    if not os.path.isfile('PS-plate.png'):
        import requests
        with requests.get(
               'http://pennstateplate.com/images/PS-plate.png') as plate:
            img = Image.open(BytesIO(plate.content))
            draw = ImageDraw.Draw(img)
            draw.rectangle(((207,300), (705,90)), fill=(255,255,255,255))
            img.save('PS-plate.png')

    # dynamics of the own craft
    craft = MyCraft(world, space, (0, 2, -0.8), (0.0, 0.0, -0.2), name)
        
    # a list for fixed obstacles
    fixed_obstacles = list()
    
    # create a link to the server, if desired
    if serverurl:
        comm = Communicator(name, craft, othercraft, marklist,
                            wind, server=serverurl)
        craft.setCommunicator(comm)
        
    # contacts
    contactgroup = ode.JointGroup()
    
    # start Panda3d engine
    craft.run()
