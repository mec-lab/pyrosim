.. _simulator:

Simulator Settings
==================

Empty Simulator
---------------

The smallest amount of code to get a simulator up and running is:

.. literalinclude:: /../../demos/docs/settings/00_empty.py
    :caption: demos/docs/settings/00_empty.py

This should cause a graphics window to pop up showing the simulated world before closing itself down after a default 100 time steps (1 second of simulated time, roughly 1 second of real time).

.. figure:: /img/empty.png
    :align: center
    :width: 75%

    Screenshot of the empty simulator. The camera defaults pointing into the *+y* direction with *+x* to the right and *+z* up.


Adjusting Parameters
--------------------

There are several parameters which can be modified during the initilization of ``Simulator``.
The most important to users will be 

- ``eval_steps`` : how many simulation steps are taken before closing (use -1 for endless simulation)
- ``dt`` : the amount of time between simulation steps. Lower `dt` values lead to more stable, but longer, simulations
- ``play_blind`` : Allows for headless running of simulation (no graphics). 
- ``play_paused`` : Allows simulation to start paused.

Multiple Simulations
--------------------

Because pyrosim uses subprocess, each simulation which is started is independent of others.
This allows for easy parallelization through using batches.
The following example compares runing serially vs. batching.

.. literalinclude:: /../../demos/docs/settings/01_multiple.py
    :caption: demos/docs/settings/01_multiple.py

.. code-block:: bash

    Serial running took 0.16266107559204102 seconds
    Batch running took 0.042613983154296875 seconds

The output may vary depending on your computer's specifications.

Useful Keyboard Commands
------------------------

The following commands are useful to use if you are running a heads up display of your simulation.

- ``ctrl-p``          : toggle pause/unpause
- ``x`` or ``ctrl-x`` : exit
- ``d``               : toggle draw joints
- ``s``               : toggle draw subspaces
- ``ctrl-t``          : toggle draw textures