#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 12 16:07:04 2020

@author: repa
"""
from numpy import cos, sin, radians, array, float32, sum as np_sum
from numpy.linalg import norm
from datetime import datetime
from collections import defaultdict
import base64
import time


class MarkState:
    '''
    Collection of data on a contestant, relative to marks
    '''
    
    def __init__(self, name: str, cid: int) -> None:
        '''
        Create an object/inventory for mark tracking

        Parameters
        ----------
        name : str
            Name of the tracked contestant.
        cid : int
            Index of the contestant.

        Returns
        -------
        None

        '''
        self.name = name
        self.cid = cid
        self.d = defaultdict(lambda: 0.0)
        self.start()
        self.debug = 50
        
    def start(self):
        self.index = -1
        self.starttime = datetime.now()
        self.penalty = 0

    def elapsed(self):
        duration = datetime.now() - self.starttime
        return duration.seconds + 1e-2 * (duration.microseconds // 10000)
        
class RaceMark:
    '''
    Capture data on race mark
    '''
    
    def __init__(self, name: str, x: float, y: float, 
                 radial: float, distance: float, 
                 rounding: str, score: int, info: str) -> None:
        '''
        Create a new race mark

        Parameters
        ----------
        name : str
            Name of the mark.
        x : float [m]
            X-location of the mark.
        y : float [m]
            Y-location of the mark.
        radial : float [deg]
            Heading of the mark line.
        distance : float [m]
            Length of the mark line.
        rounding : str
            Direction of rounding, 'no': penalty, don't round, 'cw' or 'ccw'.
        score : int [s] or 'finish'
            Time penalty due to rounding.
        info : str
            Information on the mark.

        Returns
        -------
        None
 
        '''
        self.name = name
        cpsi, spsi = cos(radians(radial)), sin(radians(radial))

        if rounding == 'ccw':
            self.lineeq = array((spsi, -cpsi, -spsi*x+cpsi*y))
        else:
            self.lineeq = array((-spsi, cpsi, spsi*x-cpsi*y))
            
        if rounding == 'near':
            self.area1 = array((cpsi, spsi, -cpsi*x-spsi*y+distance))
        else:
            self.area1 = array((cpsi, spsi, -cpsi*x-spsi*y))    
        self.area2 = array((-cpsi, -spsi, cpsi*x+spsi*y+distance))
        
        self.pos = array((x, y))
        self.info = info
        self.rounding = rounding
        self.score = score
        self.distance = distance
        
        print(f'race mark {name}, {x},{y}, {rounding}, d={distance}')
        
    def update(self, state: MarkState, pos, index: int) -> None:
        '''
        Update the race position

        Parameters
        ----------
        state : MarkState
            Information on race state.
        pos : tuple of float
            Location of the craft, northing, easting
        index : int
            Index of this mark in a list
            
        Returns
        -------
        None. Updates the mark state.

        '''
        # calculate distance from mark
        loc = array((pos[0], pos[1], 1.0))
        d = self.lineeq @ loc
        dist = self.area1 @ loc > 0 and self.area2 @ loc > 0
        event = None
        '''
        if state.debug == 0:
            print(f'{self.name} {state.d[index]}->{d} l:{self.area1 @ loc} r:{self.area2 @ loc}')
        if index == 0:
            if state.debug == -1:
                state.debug = 50
            else:
                state.debug -= 1
        '''    

        if self.rounding == 'no':

            if state.d[index] == 0.0:
                # print(f'{index} initial d {d}')
                state.d[index] = d
                return

            # penalty mark. Automatically advance, and always do calculations
            if state.index == index - 1:
                state.index = index
                            
            if dist and state.d[index] * d < 0.0:
                state.penalty += self.score
                print(f"ship {state.name} penalty {self.score}s: {self.info}")
                event = f'SP{index}:{self.score}'.encode('ascii')
            
            if d != 0.0:
                state.d[index] = d
                    
            # handled
            return event
        
        # skip exact 0 calculations and when far from mark
        if d == 0.0 or not dist or index != state.index + 1:
            #if index == state.index + 1:
            #    print(f"{index} no further calculation {d} {dist}")
            return
    
        if state.d[index] == 0.0:
            #print(f'{index} initial d {d}')
            state.d[index] = d
            return
    
        # only count passage from - to + 
        # print(f"{index} evaluating {state.d[index]} to {d}")
        if state.d[index] < 0.0 and d > 0.0:
            state.index = index
            rtime = state.elapsed()
            if self.score == 'finish':
                print(f"ship {state.name}, finished in {rtime}s, penalty {state.penalty}")
                event = f'SF{index}:{rtime}'.encode('ascii')
            else:
                print(f"ship {state.name} rounded mark {self.name}")
                event = f'SR{index}:{rtime}'.encode('ascii')
                state.penalty += self.score
                
        state.d[index] = d
        return event
            
    def transmit(self, index: int):
        return f'M{index}:{self.name}:{self.info}:'.encode('ascii') + \
            base64.b64encode(array(self.pos, dtype=float32))
            
    def plotMe(self, ax):
        ax.plot(self.pos[1], self.pos[0], '.')
        if self.rounding == 'near':
            line = array((self.pos + self.area2[:2] * self.distance, 
                          self.pos + self.area1[:2] * self.distance)) 
        else:   
            line = array((self.pos, 
                          self.pos + self.area1[:2] * self.distance))
        #print(line)
        ax.plot(line[:,1], line[:,0])

if __name__ == '__main__':
    from matplotlib import pyplot as plt 
    
    ms = MarkState('testship', 0)
    
    marks = [
        RaceMark('start', 0, 0, 90, 10, 'near', 0, 'start line'),
        RaceMark('nextto', 2, 0, 90, 10, 'ccw', 0, 'start line'),
        RaceMark('keepaway', -5, -10, 180, 10, 'no', 10, 'keepaway'), 
        RaceMark('next', 0, 50, 270, 10, 'cw', 0, 'next'),
        RaceMark('finish', 10, 20, 90, 10, 'near', 'finish', 'Finish line')
        ]

    path =  ( (1, 5), (-1, 5), # cross start line
                (1, 2), (3, 2), (1, 0),
                (-3, -5), (-3, -12), (-4, -7),
                (-6, -5), (-6, -11), (-7, -5), # cross penalty twice
                (-1, 45), (1, 45),            # wrong way for mark
                (-1, 45), (-1, 80),
                (11, 19), (12, 22),            # finish
                (9, 22), (11, 23) )
    
    
    f = plt.figure()
    plt.plot(array(path)[:,1], array(path)[:,0])
    ax = plt.gca()
    for m in marks:
        m.plotMe(ax)
    plt.show()
    events = []
    for xy in path:
        print(f"at position {xy}")
        for i, m in enumerate(marks):
            m.update(ms, array(xy), i)
        time.sleep(0.1)
            