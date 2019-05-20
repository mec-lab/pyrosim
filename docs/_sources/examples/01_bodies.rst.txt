.. _bodies:

Bodies
======

Basics
------

There are three types of bodies which can be sent

1. Spheres
2. Boxes
3. Cylinders

After sending a body to the simulator, an integer ID is returned which is used as a handle to reference the body later when attaching sensors, joints, etc.

Composite Bodies
----------------

Sometimes, you may want an object that is composed of multiple body prototypes.
This can be implemented using ``send_composite_body()`` and then attaching the specific bodies as shown below.

.. warning::
    Placing bodies in a composite body such that their surfaces are flush should be avoided.
    It will cause issues with collision detection and lead to strange behavior.

The following code creates a set of body prototypes and an example of a composite body.

.. literalinclude:: /../../demos/docs/bodies/00_all_bodies.py
    :caption: demos/docs/bodies/00_all_bodies.py

.. raw :: html

    <iframe width="100%" height="395" style="float:middle" src="https://www.youtube.com/embed/-Vp9At22ZvQ" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>