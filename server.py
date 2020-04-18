#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 23 16:26:52 2020

@author: repa
"""
import numpy as np
import asyncio
import websockets
import base64
from configparser import ConfigParser
from sailmark import RaceMark, MarkState
from windmodel import WindModel
import matplotlib.pyplot as plt
import json
from datetime import datetime
from startboxes import StartBoxes

"""
Server configuration file, server.conf:

- Wind section, labeled [wind]
  x = Wind speed in x direction (towards North), [m/s]
  y = Wind speed in y direction (towards Ease), [m/s]

- Start positions for the participating vehicles [start]
  x = X location [m]
  y = Y location [m]
  z = Z location [m], needs to be -0.8 for the standard ice skater
  psi = Initial heading [deg]
  dx = shift in x position for next craft [m]
  dy = shift in y position for next craft [m]

- Network properties [network]
  ip = IP address for network connection
  port = port number for network connection

  Note that network connections use the websockets protocol

- Graphical objects placed in the world [object....]
  name = file name for the object, base (no .egg or .blend extension)
  x = X location [m]
  y = Y location [m]
  z = Z location [m]
  psi = Object orientation [deg]

- Obstructions for the ODE collision dynamics, normally corresponding
  to the graphical objects [obstruction....]
  name = name for the obstruction, default from the ellipsis after 
         obstruction keyword.
  x = X location [m]
  y = Y location [m]
  z = Z location [m]
  size = size, as 3 numbers, comma-separated [m]. Depending on the geometry
         type, 1, 2 or 3 of these numbers are used. 
  orientation = orientation, euler angles (phi, theta, psi) as three numbers, 
                comma-separates [deg]. First psi is applied, then theta, 
                then phi
  geom = geometry type, either sphere, capsule, cylinder, box

"""

all_connected = dict()
def pprint(*a):
    #print(*a)
    pass


class LogObject:
    def __init__(self, name):
        self.name = name
        self.x = []
        self.y = []
        self.t = []
        self.line = None

class Server:
    """Remember and distribute object data"""
    _splitchar = ':'.encode('ascii')


    async def _communicate(self, websocket, path):
        """
        Back-end, handles the connection to one of the clients

        Parameters
        ----------
        websocket : TYPE
            DESCRIPTION.
        path : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        try:
            print('in _communicate')
            data = await websocket.recv()
            if data[:1] != b'B':
                raise ConnectionError(
                    f"Incorrect start protocol {data}")
                
            name = data[1:].decode('ascii')
            print(f'new player {name}')
                        
            # step 1, initial connection / birth
            index = self.lifecounter
            self.lifecounter += 1
            
            # get a start box
            while True:
                try:
                    ndata = self.startboxes.assign(index)
                    break
                except IndexError as e:
                    print(f'Start position for {index}, error {e}')
                    await asyncio.sleep(2)
            
            #ndata = np.zeros((7,), dtype=np.float32)
            #ndata[:3] = self.pos0 + self.dpos*index
            #ndata[3:7] = self.quat0            

            # produce the welcome message with the chosen start position
            data = f'W{index}:'.encode('ascii') + \
                base64.b64encode(ndata)
            await websocket.send(data)
            
            # send the environment configuration
            for d in self.config:
                #print(f"sending {d}")
                await websocket.send(d)
            
            # copy birth to others, and inform this one of other players
            birth = f'B{index}:{name}'.encode('ascii')
            if all_connected:
                print(f"sending {birth} to {len(all_connected)} players")
                await asyncio.wait(
                    [sock.send(birth) for sock in all_connected])

            # inform this one of other players
            for b2 in all_connected.values():
                print(f"sending {b2} to new player")
                await websocket.send(b2)

            # store in dict with connected items                
            all_connected[websocket] = birth
            
            # create a mark status for this vehicle
            markstate = MarkState(name, index)
            
            for im, m in enumerate(self.marks):
                # send information on the marks
                await websocket.send(m.transmit(im))

            # for logging and plotting
            self.clog[index] = LogObject(name)

            # remember wind time sent
            seconds = 0

            # repeat step 2, Updates until Death
            while True:
                
                data = await websocket.recv()
                
                if data[:1] == b'D':
                    print(f"death of connected {index}")
                        
                    break
                    
                if data[:1] != b'U':
                    ConnectionError(
                        f"Incorrect run protocol {data}")
                
                pprint(f"data from {index}")
                # update for own view
                self.clist[index] = data

                # send/reflect to all others
                if len(all_connected) > 1:
                    await asyncio.wait(
                        [user.send(data) for user in all_connected 
                         if user != websocket])
                pprint("copied to", len(all_connected) - 1, "others")
                
                # decode position
                ndata = np.frombuffer(
                    base64.decodebytes(
                        data.split(self._splitchar)[1]), 
                        dtype=np.float32)
                pos = ndata[:2].astype(float)
                #print(f"incoming position {pos}")
                
                # check the start box status
                self.startboxes.update(index, pos)
                
                for im, m in enumerate(self.marks):
                    event = m.update(markstate, pos, im)
                    if event:
                        await websocket.send(event)
                        
                if seconds != self.seconds:
                    seconds = self.seconds
                    await websocket.send(self.wind.message)
                    self.clog[index].x.append(pos[0])
                    self.clog[index].y.append(pos[1])
                    self.clog[index].t.append(markstate.elapsed())
        
        except websockets.ConnectionClosedError as e:
            print(f"Close error {e}")             
        finally:
            print(f"removing {index}")
            del all_connected[websocket]
            data = f'D{index}'.encode('ascii')
            if all_connected:
                try:
                    await asyncio.wait(
                        [user.send(data) for user in all_connected])
                except Exception as e:
                    print(f"Deletion sent failed {e}")  
            else:
                self.lifecounter = 0
                
            filename = datetime.now().strftime(
                f"saillog-%m%d-%H%M{name}-{index}.json")
            print(f"saving data for {name} to {filename}")
            with open(filename, 'w') as f:
                json.dump(dict(n=name, x=self.clog[index].x, 
                               y=self.clog[index].y,
                               t=self.clog[index].t), f)
            
    async def _update_wind(self):
        while True:
            
            # also do any plotting
            for s in self.clog.values():
                if s.line is None:
                    s.line, = self.ax.plot(s.y, s.x, label=s.name)
                else:
                    s.line.set_data(s.y, s.x)
            self.fig.canvas.draw()
            plt.pause(0.01)
            
            # update the wind data
            self.seconds += 1
            self.wind.update()
            self.wind.update()
            await asyncio.sleep(2)
            
    def __init__(self, hostip: str, port: int, 
                 config: list, marklist: list, wind: WindModel,
                 startboxes: StartBoxes):
        
        # initial and incremental position for participants
        self.startboxes = startboxes
        
        # game configuration, race marks and wind
        self.config = config
        self.marks = marklist
        self.wind = wind

        # client list and id counter
        self.clist = {}
        self.clog = {}
        self.lifecounter = 0
        
        # wind and time
        self.seconds = 0
        
        # start server
        self.start_server = websockets.serve(
            self._communicate, hostip, port)
        #asyncio.get_event_loop().run_until_complete(
        #    self.start_server)
        
        # plt.ion()
        #plt.plot()
        #plt.pause(0.001)
        self.fig = plt.figure(figsize=(5,7))
        self.ax = self.fig.add_subplot(111)
        for m in marklist:
            m.plotMe(self.ax)
        self.ax.axis('equal')
        self.fig.show()
        plt.pause(0.1)
        
        asyncio.gather(self.start_server, self._update_wind())
        asyncio.get_event_loop().run_forever()

if __name__ == '__main__':
    # create a server
    config = ConfigParser()
    config.read('server.conf')
    
    # wind configuration
    wx = config.getfloat('wind', 'x', fallback=0.0)
    wy = config.getfloat('wind', 'y', fallback=0.0)
    varwind = config.getfloat('wind', 'var', fallback=0.0)
    tauwind = config.getfloat('wind', 'tau', fallback=100.0)
    
    # ip / network connection
    ip = config.get('network', 'ip', fallback='127.0.0.1')
    port = config.getint('network', 'port', fallback=8300)
    
    # starting spots for participants
    x0 = config.getfloat('start', 'x', fallback=0.0)
    y0 = config.getfloat('start', 'y', fallback=0.0)
    z0 = config.getfloat('start', 'z', fallback=-0.8)
    dx0 = config.getfloat('start', 'dx', fallback=0.0)
    dy0 = config.getfloat('start', 'dy', fallback=0.0)
    psi0 = config.getfloat('start', 'psi', fallback=0.0)
    nstart = config.getint('start', 'npositions', fallback=8)
    startboxes = StartBoxes(x0, y0, z0, psi0, dx0, dy0, nstart)    
    
    # state of the simulation wind, objects and race marks  
    wind = WindModel(wx, wy, varwind, tauwind)
    configlist = [ ]
    marklist = [ ]
    
    # read the config file
    for sname, sprox in config.items():
        if sname.startswith('object'):
            x = sprox.getfloat('x', fallback=0.0)
            y = sprox.getfloat('y', fallback=0.0)
            z = sprox.getfloat('z', fallback=0.0)
            psi = sprox.getfloat('psi', fallback=0.0)
            name = sprox.get('name', fallback=sname[len('object'):])
            coord = base64.b64encode(
                np.array((x, y, z, psi), dtype=np.float32))
            configlist.append(
                f'L{name}:'.encode('ascii') + coord)
            
        elif sname.startswith('obstruction'):
            name=sprox.get('name', fallback=sname[len('obstruction'):])
            x = sprox.getfloat('x', fallback=0.0)
            y = sprox.getfloat('y', fallback=0.0)
            z = sprox.getfloat('z', fallback=0.0)
            gsx, gsy, gsz = map(
                float, sprox.get('size', fallback="1,1,1").split(','))
            gox, goy, goz = map(
                float, sprox.get('orientation', fallback="0,0,0").split(','))
            geom = sprox.get('geom', fallback=None)
            if geom not in ('sphere', 'capsule', 'cylinder', 'box'):
                print(f"cannot handle geom type {geom}, skipping")
                continue
            coord = base64.b64encode(
                np.array((x, y, z, gsx, gsy, gsz, gox, goy, goz), 
                         dtype=np.float32))
            configlist.append(
                f'O{name}:{geom}:'.encode('ascii') + coord)
            
        elif sname.startswith('mark'):
            name=sprox.get('name', fallback=sname[len('mark'):])
            x = sprox.getfloat('x', fallback=0.0)
            y = sprox.getfloat('y', fallback=0.0)
            radial = sprox.getfloat('radial', fallback=0)
            distance = sprox.getfloat('distance', fallback=100)
            rounding = sprox.get('rounding', fallback='cw')
            info = sprox.get('info', fallback='No information')
            score = sprox.get('score', fallback=0)
            try:
                score = int(score)
            except ValueError:
                if score != 'finish':
                    raise
            marklist.append(
                RaceMark(name, x, y, radial, distance, rounding,
                         score, info))
            
    print(f"number of objects+obstructions {len(configlist)}")
    print(f"Number of race marks {len(marklist)}")
            
    srv = Server(ip, port, configlist, marklist, wind, 
                 startboxes)
