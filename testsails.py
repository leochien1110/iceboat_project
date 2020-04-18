#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 28 15:25:07 2020

@author: repa
"""

import ode
from iceboat import Wind, IceSailer, coll_callback
g0 = 9.80665
from numpy import radians, cos, sin, zeros, arange, degrees, arctan2, sqrt
from matplotlib import pyplot as plt

if __name__ == '__main__':
    
    #%% set-up the simulations
    # ode world
    world = ode.World()
    world.setERP(0.8)
    world.setCFM(1E-5)
    wind = Wind((-5, 0, 0))
    world.setGravity((0, 0, g0))

    space = ode.Space()
    ground = ode.GeomPlane(space, (0, 0, -1), 0)
    ground.nam = "ground"

    # create 360/<dhdr> craft in a circle with <R> m radius
    dhdr = 10
    R = 250
    craft = [ IceSailer
             (world, space, 
              (R*cos(radians(hdg)), R*sin(radians(hdg)), -0.8), 
              (0.0, 0.0, radians(hdg))) for hdg in range(0,360,dhdr) ]
    
    # contacts for skating and collision
    contactgroup = ode.JointGroup()

    # set initial sail & keep rudder straight
    # in a next step, should vary the mainsheet angle, to see which
    # angle gives the best speed
    for c in craft:
        c.updateMainsheet(0.1)
        
    # iterate, 120 Hz, over 60 seconds, record data every 0.1 second
    rate = 120
    rdiv = 12
    span = 60

    # room for result
    npts = span * 10
    result = [ dict(ds=zeros((npts,)), 
                    xy=zeros((npts,2)), 
                    V=zeros((npts,)),
                    Vw=zeros((npts,)),
                    psi=zeros((npts,)),
                    psir=zeros((npts,)),
                    Vmg=zeros((npts,)),
                    alpha=zeros((npts,))) for c in craft ]

    #%% - iterate over 60 seconds of simulation
    for it in range(rate*span):
        
        # wind effect on sail
        for c in craft:
            c.force(wind)
        
        # calculate collisions
        space.collide( (world, contactgroup), coll_callback)
            
        # update the world
        world.step(1/rate)

        # clear contacts for next round
        contactgroup.empty()
 
        if it % rdiv == rdiv - 1:
            idx = it // rdiv
            for r, c in zip(result, craft):
                r['ds'][idx] = c._ds
                r['xy'][idx] = c.body.getPosition()[:2]
                r['V'][idx] = c.V
                r['Vw'][idx] = c.Vw
                r['psir'][idx] = c.gamma
                qW, qx, qy, qz = c.body.getQuaternion()
                r['psi'][idx] = arctan2(2.0*qx*qy + qW*qz,
                                           qW*qW + qx*qx - qy*qy - qz*qz)  
                r['Vmg'][idx] = c.body.getRelPointVel((0,0,0))[0]
                r['alpha'][idx] = c.alpha
                
    #%% - plot the results
    t = arange(0, span, rdiv/rate)
    plt.close('all')
    for r,hdg in zip(result, range(0,360, dhdr)):
        f = plt.figure()
        plt.subplot(311)
        plt.plot(t, degrees(r['ds']))
        plt.subplot(312)
        plt.plot(t, r['V'], t, r['Vw'], t, r['Vmg'])
        plt.suptitle(f'craft departing in heading {hdg}')
        plt.subplot(313)
        plt.plot(t, degrees(r['psir']), t, degrees(r['psi']), 
                 t, degrees(r['alpha']))
        
    plt.figure()
    for r,hdg in zip(result, range(0,360, dhdr)):
        plt.plot(r['xy'][:,1], r['xy'][:,0], r['xy'][0,1], r['xy'][0,0], 'o')
    plt.axis('equal')
    plt.suptitle(f'trajectories for all headings, step {dhdr} deg')
    '''
    for r,hdg in zip(result, range(0,360, dhdr)):
        f = plt.figure()
        plt.plot(r['xy'][:,1], r['xy'][:,0], r['xy'][0,1], r['xy'][0,0], 'o')
        plt.axis('equal')
        plt.suptitle(f'trajectory for heading {hdg}')
    '''
    plt.show()
