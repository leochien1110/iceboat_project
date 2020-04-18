#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 27 13:12:31 2020

@author: repa
"""
import numpy as np

class StartBox:
    def __init__(self, x, y, z, psi, r):
        '''
        Start position, with monitoring of occupation

        Parameters
        ----------
        x : float
            X (Northing) position.
        y : float
            Y (Easting) position.
        z : float
            Z (vertical) position.
        psi : float
            start heading.
        r : float
            Size of the box.

        Returns
        -------
        None.

        '''
        self.xyz = np.array((x, y, z), dtype=np.float32)
        self.quat0 = np.zeros((4,), dtype=np.float32)
        self.quat0[0] = np.cos(np.radians(0.5*psi))
        self.quat0[3] = np.sin(np.radians(0.5*psi))
        self.r = r
        self.craftid = None
        
    def reserve(self, craftid):
        '''
        Let a craft start from this box
        
        Parameters
        ----------
        craftid : int
            ID of the departing craft
        '''
        self.craftid = craftid
        ndata = np.zeros((7,), dtype=np.float32)
        ndata[:3] = self.xyz
        ndata[3:7] = self.quat0
        return ndata

    def update(self, pos):
        '''
        Update start box availability

        Parameters
        ----------
        pos : iterable of float, x, y
            position.

        Returns
        -------
        True if the box is now free again.

        '''
        if np.linalg.norm(self.xyz[:2] - pos) > self.r:
            self.craftid = None
            return True
        return False
        
class StartBoxes:
    '''
    Inventory of initial start positions
    '''
    
    def __init__(self, x0, y0, z0, psi0, dx0, dy0, npositions):
        '''
        Create a row of starting positions

        Parameters
        ----------
        x0 : float
            Initial start position.
        y0 : float
            Initial start position.
        z0 : float
            Initial start height.
        psi0 : float
            initial start heading.
        dx0 : flaot
            x-step in start box position.
        dy0 : float
            y-step in start box position.
        npositions : int
            Number of available start boxes.

        Returns
        -------
        None.

        '''
        # circular distance determining box is free
        R = np.linalg.norm(np.array((dx0, dy0))) * 0.5
        
        # create empty boxes
        self.available = [
            StartBox(x0+dx0*i, y0+dy0*i, z0, psi0, R)
            for i in range(npositions)]
        
        # room for occupied boxes
        self.taken = {}

    def assign(self, craftid):
        '''
        Assign one of the available boxes

        Parameters
        ----------
        craftid : int
            Number for the new participant.

        Returns
        -------
        Start position + orientation quaternion.

        '''
        if self.taken.get(craftid, None) is not None:
            raise ValueError('Double craft ID in startboxes?????')            
        if not self.available:
            raise IndexError('No start position free')

        box = self.available.pop(0)        
        self.taken[craftid] = box
        print(f'Start box at {box.xyz} assigned to {craftid}')
        return box.reserve(craftid)
    
    def update(self, craftid, pos):
        '''
        Check if a vehicle left the box already

        Parameters
        ----------
        craftid : TYPE
            DESCRIPTION.
        pos : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        '''
        box = self.taken.get(craftid, None)
        if box is None:
            return
        if box.update(pos):
            print(f'Start box at {box.xyz} released by {craftid}')
            del self.taken[craftid]
            self.available.append(box)
            
    def left(self, craftid):
        box = self.taken.get(craftid, None)
        if box is None:
            return
        print(f'Start box at {box.xyz}, given up by {craftid}')
        del self.taken[craftid]
        self.available.append(box)
        
if __name__ == '__main__':
    
    startboxes = StartBoxes(0, 0, 0, 0.1, 10, 2, 3)
    
    startboxes.assign(0)
    startboxes.assign(1)
    startboxes.assign(2)
    try:
        startboxes.assign(2)
    except ValueError:
        pass
    
    try:
        startboxes.assign(5)
    except IndexError:
        pass
    
    startboxes.update(0, np.array((50, 0)))
    startboxes.assign(0)
    
    startboxes.left(2)
    startboxes.assign(16)