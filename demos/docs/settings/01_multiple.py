import pyrosim
import time # for timing purposes

n_simulators = 10
eval_steps = 10000

# serial running of simulators
start_time = time.time()
for i in range( n_simulators ):
    sim = pyrosim.Simulator( play_blind = True,
                             eval_steps = eval_steps,
                             play_paused = False )

    sim.start()
    sim.wait_to_finish()
end_time = time.time()

print( 'Serial running took', end_time - start_time, 'seconds' )

# batch running of simulators
start_time = time.time()
sims = [0] * n_simulators
# initialize simulators as batch
for i in range( n_simulators ):
    sims[i] = pyrosim.Simulator( play_blind = True,
                                 eval_steps = eval_steps,
                                 play_paused = False )
    sims[i].start()

# wait for simulators to finish
for i in range( n_simulators ):
    sims[i].wait_to_finish()

end_time = time.time()

print( 'Batch running took', end_time - start_time, 'seconds' )