import sys
sys.path.insert(0, '../../')
import pyrosim

sim = pyrosim.Simulator( eval_steps = -1, play_paused = True )


light_box = sim.send_box( position=(0, 0, 1) )
sim.add_light_to_body( light_box )

sense_box = sim.send_box( position = ( 0.5, 0, 1 ) )
light_sensor = sim.send_light_sensor( sense_box )
# swing the sense back to get different values
sim.send_hinge_joint( -1, sense_box,
                      anchor = ( 0, 0, 1 ),
                      axis = ( 0, 1, 0 ) )

sim.start()
sim.wait_to_finish()

light_data = sim.get_sensor_data( light_sensor )

print( light_data )