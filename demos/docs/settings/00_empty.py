import pyrosim

# create the simulator
sim = pyrosim.Simulator()

# start the simulator
sim.start()

# wait for simulator to finish
sim.wait_to_finish()