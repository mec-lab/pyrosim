.. _overview:

Advanced Options
================

Pyrosim is intended to be a starting off point for people interested in the easy simulation of neurally controlled robots.
A user may reach a point where they want to extend pyrosim for their personal use.
This section will hopefully provide a good foundation on the structure of pyrosim.

Core Concepts
-------------

Pyrosim works by using the python subprocess module to send strings through stdin to a C++ executable which is running the actual simulation.
When the python side is finished sending commands to the C++ side, the C++ side starts the actual physics simulation.
After simulation is complete, the C++ side then returns sensor data back through stdout and error/debug output through stderr to the python side.
The user can then read the sensor data and debug output as seen previously.

There are two main types of objects which are sent into the C++ side: Parameters_ and Entities_.

Parameters
----------

Parameters are options which alter simulation, gravity, camera position, etc.
On the python side, the send commands are located in the main *pyrosim.py* file and have the form ``set_x(*args)`` where ``x`` is whatever parameter is to be set by the user.
For example, ``set_gravity(x, y, z)`` would set the gravity parameters in the simulation.
Parameters should be sent to the simulator using ``self._send_parameter( str, val )`` where ``str`` is the key and ``val`` is the actual value to be stored in the map.

On the C++ side, parameters are handled in a single map called ``parameters`` located in *simulator.cpp*.
It is a map which takes a string key value and stores a float value.
Currently, the way it is set up, only float values can be stored.
If you need to use a non-float parameter, it must be type cast at the point of use.
Further, if the parameter is a vector or array, it must be split into different key values in the map.

In order to create a new parameter, you must create a ``set`` command in python and an matching initialization in C++.
Below is the complete example of ``set_gravity``.

.. literalinclude:: /../../pyrosim/pyrosim.py
    :caption: pyrosim/pyrosim.py
    :lines: 232-236
    :language: python
    :dedent: 4

.. literalinclude:: /../../pyrosim/simulator/src/simulator.cpp
    :caption: pyrosim/simulator/src/simulator.cpp
    :language: c++
    :lines: 296,322-324

The string values sent by the ``self._send_parameter(str, val)`` must match the string used in map by C++.
Now the gravity parameters can be used to set the gravity in simulation

.. literalinclude:: /../../pyrosim/simulator/src/simulator.cpp
    :caption: pyrosim/simulator/src/simulator.cpp
    :language: c++
    :lines: 149-154

Entities
--------

Entities are the main components of simulation.
There are currently six types of entities in pyrosim.

- Bodies
- Joints
- Actuators
- Sensors
- Neurons
- Synapses

On the python side, entities are organized using `mixin classes`_ allowing for better organization by seperating entity relevant code in specific python files.
For example, all python code regarding sensors is in *pyrosim/_sensor.py*
Each entity is sent to the C++ side using ``send_x( *args )`` where ``x`` is the name of the entity, e.g. `sensor`.

.. _mixin classes: http://www.martinbroadhurst.com/mixins-in-python.html

On the C++ side, Each entity type is a sub-class of the entity base class and each specific variation of an entity should be its own sub-class.
For example, the touch sensor has its own sub-class.
The entity base class is contained within *pyrosim/simulator/src/entity.hpp*.
It has a few standard basic methods used by all entities.

The `takeStep` method is what ultimately differentiates the entities.
This is the method in which actuators will act, sensors will sense, the network will update etc.


.. All entities are held within 
.. Simulation Step
.. ---------------


