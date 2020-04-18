#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 11:25:50 2020

@author: repa
"""
from numpy import zeros, array, sqrt, exp, float32
from numpy.random import normal
import base64

class WindModel:
    def __init__(self, vx: float, vy: float, var: float, tau: float=100.0) -> None:
        '''
        Create a wind model, with varying wind

        Parameters
        ----------
        vx : float
            Average wind, x / northing direction.
        vy : float
            Average wind, y / easting direction.
        var : float
            Standard deviation of variation process.
        tau : float
            Normalized fime constant of variation process.

        Returns
        -------
        None
        
        '''
        self.vbase = array((vx, vy), dtype=float32)
        self.vvar = self.vbase.copy()
        self.var = var / sqrt(0.5/tau)
        self.psi = 1.0 - exp(-1.0/tau)
        self.message = 'E'.encode('ascii') + base64.b64encode(self.vvar)
        
    def update(self):
        '''
        Update step of the wind, returns array with current wind

        Returns
        -------
        array of float (copied!).

        '''
        self.vvar += -self.psi * self.vvar + \
            self.psi * normal(loc=self.vbase, scale=self.var)
        self.message = 'E'.encode('ascii') + base64.b64encode(self.vvar)
        return self.vvar
    
if __name__ == '__main__':
    
    from numpy import std
    import matplotlib.pyplot as plt
    
    vxy1 = []
    vxy2 = []
    
    wind = WindModel(5, 1, 0.5, 100)
    for i in range(1000):
        vxy1.append(wind.update().copy())
    vxy1 = array(vxy1)
        
    wind = WindModel(5, 1, 0.5, 10)
    for i in range(1000):
        vxy2.append(wind.update().copy())
    vxy2 = array(vxy2)
            
    f = plt.figure()
    plt.subplot(211)
    plt.plot(vxy1[:,0])
    plt.plot(vxy1[:,1])

    plt.subplot(212)
    plt.plot(vxy2[:,0])
    plt.plot(vxy2[:,1])

    print(f'var 1 x {std(vxy1[:,0])} y {std(vxy1[:,1])}')
    print(f'var 2 x {std(vxy2[:,0])} y {std(vxy2[:,1])}')
    