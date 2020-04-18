# Ice Sailing Simulation for AERSP 597, spring 2020

## Introduction

The course AERSP597 looks at the way humans interact with machines,
specifically vehicles, using an approach founded in Cognitive Systems
Engineering.

The participants in the course will also act as the humans in the
human-machine system. The machine you will be using here is a
simulation of an ice sailing craft or ice boat. This simulation was
originally created for a course at TU Delft on distributed simulation,
and is now adapted and expanded to run on Python. It is a vehicle for
which no displays exist, and the current simulation is equipped with
only a simple compass and a speed indicator. You can sail it, crash
it, and we will upgrade the simulation to include other teams as well,
so you will be able to crash the craft into each other and crash them
against the shore.

Ultimately we will run a sailing race with these craft.

The basic simulation is created in two files. One is `iceboat.py`, this
file provides the vehicle dynamics and loads the visual models. Your
instructors will over time update this to make the competition more
challenging. The second file is `hud.py`. The HUD currently shows a
simple compass and prints the speed, and it is meant as a starter. The
`hud.py` file is thus yours to modify, you should create the
instruments, using the theory in the course, that help you sail a
successful race. The design of these instruments must be
**motivated**, and we will test design and theory in the races.

## Technical details

The simulation uses the widely available open-source programming
language Python. The programming done in the course is limited, and
you should be able to get by mostly by modifying the given
examples.

You will need a number of software components to run the
simulation. The software should run on major platforms (Windows, Apple
Mac OSX, Linux), and packages are available on the conda ecosystem.

Steps to take for installation:

1. Install the Anaconda python distribution, download from
   the [anaconda](https://www.anaconda.com/download/) site and install. Use
   the Python 3 version, Python 2.7 will be obsoleted in January 2020.

2. When on Windows, open the Anaconda powershell prompt, to enter commands
   to configure your conda installation.

   On Linux or Linux or Mac OSX, you can simply open a terminal, and first
   enter the following command to activate the conda Python installation:

   ```
   conda activate
   ```

3. From the conda powershell, or your Linux/OSX shell with conda actated, 
   enter:

   ```
   conda update conda
   conda update --all
   ```

4. Now add `conda-forge`, an additional software channel

   ```
   conda config --add channels conda-forge
   conda update --all
   ```

5. We need three pieces of sofware specifically for the
   simulation. `git` will be used to get the simulation source,
   `Panda3d` gives the visualization, and `pyode` with `libode` calculates the
   dynamics of the ice sailer. Because Panda3d is not in a conda
   package, we will use another program, `pip`, to install that. If you
   already have git, you do not need to install it through conda. 

   ```
   conda install git
   conda install libode
   conda install pyode
   conda install pip
   conda install websockets
   pip install panda3d
   ```

   NOTE: libode and pyode were recently (Jan 2020) added to the
   conda-forge distribution.

5. Now get the source. I am hosting this on the gitlab of the Delft
   University of Technology.

   ```
   git clone https://gitlab.tudelft.nl/rvanpaassen/cse-vehicle-sailer.git
   ```

6. To start sailing, enter the newly created folder and run with python:

   ```
   cd cse-vehicle-sailer
   python iceboat.py
   ```
   
   **If** you have a file called `iceboat.conf` (see 
   `iceboat.conf.example`), then the simulation will try to connect 
   with a server. This server may initially run on your own machine, 
   over the local network interface. 
   
   With the server, you can add course elements to the scene (buoys 
   and a referee boat), the wind can be modified, and the start location
   is now given by the server. Note that the objects can also be 'felt'
   in the simulation, if you crash into them you get stopped, and 
   depending on the wind direction you might also get stuck. 
   
   Likewise the shores of the simulation also affect the dynamics. 
   
   You can start the server by running:
   
   ```
   python server.py
   ```
   
   The server reads the file `server.conf` for its configuration.
   
   You can also run these programs from the 'Spyder' editor. To run
   multiple simulations simultaneously, you need to re-configure the 
   run mode in Spyder; select 'Run', 'Configuration per file' and 
   under Console select 'Run in an external system terminal'.

   Note that we currently have a temporary database, the large green
   and red tubes are not visible in the simulated dynamics (you can
   sail through them), and point north and east; these are used for
   testing, and for getting a feel of the dynamics.

7. If there are updates to the simulation or support files, you can
   get these locally by entering:

   ```
   git pull
   ```

## To Do

*  Remove the testing x and y axis visualization

*  Make the wind a little variable

*  Add the markers for the race track

*  Create proper interfaces (your work!)

## Resources

[PANDA3D](https://www.panda3d.org/) 3D rendering framework.

[Open Dynamics Engine](https://www.ode.org) physics library.

[NumPy](https://numpy.org) scientific computing with Python.

[Python](https://www.python.org/about/gettingstarted/) for beginners.

[The Canvas page for this course](https://psu.instructure.com/courses/2049293).

[Blender](https://blender.org) was used to create the 3d models.
