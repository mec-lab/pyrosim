.. _installation:

Installation
============

Currently, the make files in pyrosim are only set up to install ODE on Linux and Mac machines.

After downloading the repo from `github <http://github.com/ccappelle/pyrosim>`_, change into the *pyrosim* root folder in terminal and run

.. code-block:: bash

    $ sh build.sh

This will build the underlying physics enngine, ODE, and compile the *pyrosim* C++ files creating the executable "simulator".

*pyrosim* can now be used locally, or alternatively, you can install it system wide with 

.. code-block:: bash

    $ sh pip install -e .
