.. _simulator:

Simulator Settings
==================

Empty Simulator
---------------

The smallest amount of code to get a simulator up and running is:

.. literalinclude:: /../../demos/docs/settings/00_empty.py
    :caption: demos/docs/settings/00_empty.py

This should cause a graphics window to pop up showing the simulated world before closing itself down after 100 time steps (1 second of simulated time, roughly 1 second of real time).

.. figure:: /img/empty.png
    :align: center
    :width: 75%

    Screenshot of the empty simulator. The camera defaults pointing into the *+y* direction with *+x* to the right and *+z* up.