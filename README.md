# pyrosim: A python robot simulator. 

Pyrosim enables the creation of robots with arbitrary body plans
and neural controllers, and the optimization of them in arbitrary simulated
environments. <!-- [Read the Docs](https://ccappelle.github.io/pyrosim) -->

## Installation instructions.

  Download or clone the GitHub [repository](https://github.com/ccappelle/pyrosim).

  If you downloaded the zipped version, unzip the directory somewhere you can access it.
  Make sure the directory path does not have any spaces in any of the folders.

  Pyrosim should work with python 2.x and 3.x.

### For Mac and Linux users:

  Open a terminal window, and navigate into that directory. For example:

  ```bash
  $ cd ~/Desktop/pyrosim
  ```
  If you are installing from Linux make sure you have build-essential,
  freeglut3, and a variety of opengl libraries installed.

  ```
  $ sudo apt-get install build-essential freeglut3-dev libx11-dev xorg-dev libglu1-mesa-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev$
  ```

  If installing from Mac, make sure to have command line tools for xcode installed

  ```
  $ xcode-select --install
  ```

  Now, to build the underlying physics engine, ODE, run
  build.sh:

  ```bash
  $ sh build.sh
  ```

  This takes three to five minutes. 

  This builds ODE and compiles the local C++ code on your machine in the 
  pyrosim/simulator directory. 

  You can now use the package locally or install it using pip
  ```bash
  $ pip install -e .
  ```

  After installation you can test the package by changing to the Demos directory and running
  any one of the provided demos. 
  For example to play the first demo run
  ```bash
  $ python Demo_00_Empty_Sim.py 
  ```
  This should bring up an OpenGL window displaying an empty, virtual world.
  The window will close on its own after a few seconds.


### Notes
  
  If you already have pyrosim installed and want the latest update from the repository,
  change into the modules root directory (contains setup.py) and pull 
  ```bash
  $ git pull
  ```
  The C++ code is likely to have changed so  you have to recompile it by going int the simulator directory and typing make
  ```bash
  $ cd pyrosim/simulator
  $ make
  ```
  The simulator directory contains all of the C++ code used in pyrosim.
  
### Next steps.

Now you can start making robots [here](https://www.reddit.com/r/ludobots/wiki/pyrosim/simulation), starting at step #3.

