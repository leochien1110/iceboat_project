#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 22 08:45:46 2020

@author: repa
"""

import asyncio
import websockets as ws
import base64
#from threading import Lock
import numpy as np
objectlist = []

"""
Communication protocol:

- contact by client, sends Born, name

  message "B<name>"

- welcome confirmation by server, assignes integer ID and start position, 7
  32bit floats with xyz + quaternion, data coded in ASCII64

  message "W<id>:data"
  
- new client information
    
  message "B<id>:name"

- updates by client, send U, integer ID in char and updated data, 
  data contains 32 bit floats, xyz, quaternion, linear vel, angular vel, dr, ds
  data coded in ASCII64

  message "U<id>:data

- death by client, send D, integer ID

  message "D<id>"

- after an update, a client receives the update / death messages from all
  other clients
  
- environemnt messages by server:
    
  message "E<windx><windy>"
  
- landscaping messages (fixed objects) by server:
    
  message 
  "L<objectname>:<geomtype>:<xyz><psi>"
  
- obstruction geoms for landscaping

  message
  "O<name>:<geomtype>:<xyz><size xyz><orientation phithetapsi>"
  
- Mark location information by server:

  message
  "M<markid>:<name>:<info>:<xy>"
  
  Note that for a finish or start line the center is given. For other marks
  (buoys to round) the location of the buoy is given:
  "MS0:Start:Start line up to 100 M east of referee boat:<xy>"
  "MA2:Buoy 3:Pass offshore of orange buoy:xy"
  "M-3:Buoy 4:Round Buoy counter-clockwise:xy"
  "M+4:Buoy 5:Round buoy clockwise:xy"
  "MF5:Finish:Finish line to 100 M North of referee boat:<xy>" 
  
- Progress information by server:

    message for finish
    "SF<id>:<markid>:<time>:<penalty>"
    
    message for rounding a mark (mark 0 is the start line)
    "SR<id>:<markid>:<time>"
  
    message for penalty
    "SP<id>:<markid>:<penalty>"
  
"""

def pprint(*a):
    #print(*a)
    pass

class Communicator:
    """
    Create a link with a distributing game server
    """
    _splitchar = ':'.encode('ascii')

    async def _sendData(self, data):
        pprint("sending")
        await self.server.send(data)
        pprint("sending done")
            
        
    async def _getOtherData(self):
        pprint("entering getOtherData")
        while True:
            try:
                data = None
                # data = await self.server.recv()
                data = await asyncio.wait_for(
                    self.server.recv(), timeout=0.001)
               
                # position update from one of the players
                if data[0] == ord('U'):
                    index = int(data[1:].split(self._splitchar)[0])
                    pprint(f"got update on {index}")
                    ndata = np.frombuffer(
                        base64.decodebytes(
                            data[1:].split(self._splitchar)[1]), 
                        dtype=np.float32)
                    pos = ndata[:3].astype(float)
                    quat = ndata[3:7].astype(float)
                    vel = ndata[7:10].astype(float)
                    omg = ndata[10:13].astype(float)
                    dr = ndata[13].astype(float)
                    ds = ndata[14].astype(float)
                    try:
                        self.othercraft[index].follow(
                            pos, quat, vel, omg, dr, ds)
                    except IndexError:
                        print(f"other ship {index} not yet known")
                        
                # deletion/death of the player with given index
                elif data[0] == ord('D'):
                    index = int(data[1:])
                    print(f"got delete on {index}")
                    del self.othercraft[index]

                # creation/birth of a player, given index and name
                elif data[0] == ord('B'):
                    index = int(data[1:].split(self._splitchar)[0])
                    name = data[1:].split(self._splitchar)[1].decode('ascii')
                    print(f"new craft detected {index}:{name}")
                    self.othercraft[index] = \
                        self.craft.newOtherCraft(name, index)

                # environment (wind speed) update
                elif data[0] == ord('E'):
                    ndata = np.frombuffer(
                        base64.decodebytes(data[1:]), dtype=np.float32)
                    #print(f"setting wind {ndata}")
                    self.wind._speed[:2] = ndata.astype(float)

                # visible model added
                elif data[0] == ord('L'):
                    name = data[1:].split(self._splitchar)[0].decode('ascii')
                    posn = np.frombuffer(
                        base64.decodebytes(data[1:].split(self._splitchar)[1]),
                        dtype=np.float32).astype('float')
                    print(f"creating object {name}")
                    objectlist.append(self.craft.newStaticObject(name, posn))
                    print(f"object {name}")

                # dynamics world obstruction geometry
                elif data[0] == ord('O'):
                    name = data[1:].split(self._splitchar)[0].decode('ascii')
                    gtype = data[1:].split(self._splitchar)[1].decode('ascii')
                    coords = np.frombuffer(
                        base64.decodebytes(data[1:].split(self._splitchar)[2]),
                        dtype=np.float32).astype(float)
                    self.craft.newObstacle(
                        name, gtype, coords)

                # mark location/type in the sail race
                elif data[0] == ord('M'):
                    cmd, name, info, xy = data.split(self._splitchar)
                    name = name.decode('ascii')
                    info = info.decode('ascii')
                    coords = np.frombuffer(
                        base64.decodebytes(xy), 
                        dtype=np.float32).astype('float')
                    mtype = cmd.decode('ascii')[1]
                    print(f'adding mark {name}, type {mtype}, '
                          f'info {info} at {coords}')
                    self.marklist.append((mtype, name, info, coords))

                # sailing advance, craft rounding a mark, penalty or finish
                elif data[0] == ord('S'):
                    midx, time = data[2:].split(self._splitchar)
                    midx = int(midx)
                    evtype = data[1:2].decode('ascii')
                    time = float(time)
                    self.craft.eventlist.append((evtype, midx, time))
                    print(f'sail event for {self.idx}: {evtype} at {midx}')

                else:
                    raise ConnectionError("unknown data message")

            except asyncio.TimeoutError:
                return
            except Exception as e:
                print(f"exception in _getotherData {e}, data={data}")
        pprint("exit otherdata")
            
    async def _initCommunication(self, server: str, data: bytes) -> None:
        
        print("entered _doCommunicate")
        
        self.server = await ws.connect(server)
        
        print("have connection to server")
        
        # initial hello message
        await self.server.send(data)
        print("sent initial data")
        
        # read confirmation; gives my index
        print("waiting for server reply")
        conf = await self.server.recv()
        if conf[:1] != 'W'.encode('ascii'):
            raise ConnectionError("incorrect reply on init")

        # decode confirmation message, zoom to initial position
        self.idx = int(conf[1:].split(self._splitchar)[0])
        print("server reply with index", self.idx)
        
        # assume the initial position   
        ndata = np.frombuffer(
            base64.decodebytes(conf.split(self._splitchar)[1]), 
            dtype=np.float32)
        print(ndata)
        self.craft.body.setPosition(ndata[:3].astype(float))
        self.craft.body.setQuaternion(ndata[3:].astype(float))
        self.craft.resetCamera()
       
        print("_initCommunication done")
        
    async def _closeCommunication(self) -> None:
        data = 'D{self.idx}'.encode('ascii')
        await self.server.send(data)
        await self.server.close()
                    
    def __init__(self, myname, 
                 craft, craftdict, marklist, wind,
                 server="ws://127.0.0.1:8300"):
        """
        Create a new game connection

        Parameters
        ----------
        myname : String
            Team name, player name
        craft : MyCraft
            Vehicle associated with this communicator
        craftdict : dict of OtherCraft, indexed by integer
            Set of other vehicles active in the environment
        marklist : list
            Empty list, will be filled with static information on race marks
        wind : Wind
            Environment object
        server : String, optional
            URL to the server websocket. The default
            is "ws://127.0.0.1:8300".
        Returns
        -------
        None.

        """
        self.name = myname
        self.craft = craft
        self.othercraft = craftdict
        self.marklist = marklist
        self.wind = wind
        self.task_receive = None
        self.task_send = None
        
        # step 1, in this time frame, send name receive 
        # confirmation
        data = f"B{myname}".encode('ascii')
        pprint("creating async communication task")
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._initCommunication(server, data))
                
    def __del__(self) -> None:
        try:
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self._closeCommunication())
        except Exception as e:
            print(f"problem closing off {e}")
        print("communicator ended")
        
    async def _ioCycle(self, data):
        tasks = [ asyncio.create_task(self._sendData(data)), 
                  asyncio.create_task(self._getOtherData()) ]
        try:
            await asyncio.wait(tasks)
        except Exception as e:
            print(e)
        #try:
        #    for t in tasks:
        #        t.cancel()
        #except Exception as e:
        #    print(f"problem cancelling {e}")
        #print(res)
        
    def update(self, data):
        """
        Check for other player data and send own data

        Parameters
        ----------
        data : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        #print("update")
        dbytes = f'U{self.idx}:'.encode('ascii') + \
                base64.b64encode(data)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._ioCycle(dbytes))
        
        
if __name__ == '__main__':
    
    from iceboat import IceSailer, OtherCraft
    import ode
    import time
    from collections import defaultdict
    
    world = ode.World()
    world.setGravity((0, 0, -10))
    space = ode.Space()

    
    # create some clients, and run them
    c0 = IceSailer(world, space, 
                   (0, 2, -0.8), (0.0, 0.0, -0.2))
    c0list = defaultdict(lambda: OtherCraft("other", 0, world, space))
    c0com = Communicator("justme", c0, c0list, 
                         "ws://127.0.0.1:8300")

    c1 = IceSailer(world, space, 
                        (0, 2, -0.8), (0.0, 0.0, -0.2))
    c1list = defaultdict(lambda: OtherCraft("other", 0, world, space))
    c1com = Communicator("justme2", c1, c1list, 
                         "ws://127.0.0.1:8300")
    
    for i in range(10):
        data = np.zeros((15,), dtype=np.float32)
        data[:3] = c0.body.getPosition()
        data[3:7] = c0.body.getQuaternion()
        data[7:10] = c0.body.getLinearVel()
        data[10:13] = c0.body.getAngularVel()
        data[13] = c0.dr
        data[14] = c0._ds
        c0com.update(data)
        data[:3] = c1.body.getPosition()
        data[3:7] = c1.body.getQuaternion()
        c1com.update(data)
        time.sleep(0.1)
        
    c2 = IceSailer(world, space, 
                        (0, 2, -0.8), (0.0, 0.0, -0.2))
    c2list = defaultdict(lambda: OtherCraft("other", 0, world, space))
    c2com = Communicator("justme3", c2, c2list, 
                         "ws://127.0.0.1:8300")
     
    for i in range(10):
        c0com.update(data)
        c1com.update(data)
        c2com.update(data)
        time.sleep(0.1)
  
    del c1com
    
    for i in range(10):
        c0com.update(data)
        c2com.update(data)
        time.sleep(0.1)
    
    time.sleep(5)
    #asyncio.get_event_loop().run_forever()
