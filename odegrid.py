#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 28 22:57:01 2020

@author: repa
"""

from panda3d.egg import EggData
import numpy as np
import ode

'''
Data on egg format coordinates

ODE world is set up as x-north, y-east, z-down

Egg data is x-north, y-west, z-up

Conversion egg -> ODE: x_ode = x_egg, y_ode = -y_egg, z_ode = -z_egg
'''

def terrainGeom(eggfile, space):
    
    # read the terrain, assumes specific model shape, with Grid labeled
    ed = EggData()
    ed.read(eggfile)
    grid = ed.findChild('Root').findChild('Grid').getFirstChild()
    
    # vpool is the pool with vertices, recode these into a verts array
    vpool = grid.getFirstChild()
    verts = np.zeros((vpool.getHighestIndex()+1,3))
    for i in range(vpool.getHighestIndex()+1):
        verts[i,:] = vpool.getVertex(i).getPos3()
    
    # create room for the indices denoting the triangle faces
    faces = []
    # we don't use the "high" triangles, assuming we don't get up
    # there with the skates, keep an index of used vertices
    usedv = np.zeros((vpool.getHighestIndex()+1,), dtype=bool)
    
    # iterate over all triangles
    c = grid.getNextChild()
    while c:
        # used vertices
        vidx = np.array((c.getVertex(0).getIndex(), 
                         c.getVertex(1).getIndex(), 
                         c.getVertex(2).getIndex()))
        
        # test if we are not looking at high terrain
        if np.min(verts[vidx,1]) < 20:
            # remember these are used
            usedv[vidx] = True
            # and include this face
            faces.append(vidx)
            
        c = grid.getNextChild()
    
    # since we are not using all vertices; recode to only the used ones
    recodev = np.cumsum(usedv) - 1
    
    # now recode the face indices
    faces = np.array([recodev[i] for i in faces])
    
    # create an array with only the used vertices
    verts = verts[usedv,:]
    
    # flip the y and z vertices, to match the ODE world
    verts[:,1:] = -verts[:,:0:-1]
    
    print(verts[:3,:])
    
    # stack into ODE mesh
    trimeshdata = ode.TriMeshData()
    trimeshdata.build(verts, faces)
    geom = ode.GeomTriMesh(trimeshdata, space)
    
    return geom, verts

if __name__ == '__main__':

    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib import pyplot as plt
    from matplotlib import cm
    
    world = ode.World()
    world.setERP(0.8)
    world.setCFM(1E-5)
    world.setGravity((0, 0, 10))

    space = ode.Space()
    terraingeom, verts = terrainGeom('blender/terrain.egg', space)
    terraingeom.nam = 'terrain'

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    surf = ax.scatter(verts[:,0], verts[:,1], verts[:,2],
                      s=1, marker='o')
    fig.colorbar(surf, shrink=0.5, aspect=5)
    
    fig = plt.figure(figsize=(8.5,11))
    ax = fig.gca()
    ax.scatter(verts[:,1], verts[:,0], marker='o', s=1)
    ax.axis('equal')
    ax.plot([0,0], [0,1000], 'g', [0,1000], [0,0], 'r')
    plt.savefig('coursemap.pdf')    
    plt.show()
    
    body = ode.Body(world)
    m = ode.Mass()
    m.setBoxTotal(10, 1, 1, 1)
    body.setMass(m)
    ball = ode.GeomSphere(space, 0.5)
    ball.nam = 'tester'
    ball.setBody(body)
    body.setPosition((-2485, -159, -80))
    ncoll = 0
    contactgroup = ode.JointGroup()
    
    # collision callback function
    def coll_callback(args, geom1, geom2):
        '''
        Collision callback function
    
        This creates collision contacts, and sets their parameters
        '''
        global ncoll
        contacts = ode.collide(geom1, geom2)
        if not contacts:
            return
    
        if geom2.nam != "ground":
            print("%i collisions %s %s" % 
                  (len(contacts), geom1.nam, geom2.nam))
            
        world, contactgroup = args
        for c in contacts:
            # non-skating, uniform mu
            c.setMu(5000)
            c.setBounce(0.02)
            j = ode.ContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())
            ncoll += 1
            
    while ncoll < 10 and body.getPosition()[2] < 10:
        space.collide( (world, contactgroup), coll_callback)
        world.step(0.01)
        contactgroup.empty()
        
        print(body.getPosition())
        
