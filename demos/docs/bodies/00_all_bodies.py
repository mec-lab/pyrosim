import pyrosim

sim = pyrosim.Simulator( play_paused = True, eval_steps = -1)

box = sim.send_box( position = [-1, 0, 1] )
cyl = sim.send_cylinder( position = [0, 0, 1] )
sphere = sim.send_sphere( position = [1, 0, 1] )

# create composite cube made of spheres
composite = sim.send_composite_body()
for x in [ 1.75, 2.25 ]:
    for y in [ -0.25, 0.25 ]:
        for z in [ 0.75, 1.25 ]:
            sim.add_sphere_to_composite( composite,
                                         position = [ x, y, z ],
                                         radius = 0.1 )

sim.start()
sim.wait_to_finish()