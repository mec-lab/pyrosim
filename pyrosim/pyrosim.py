from __future__ import division, print_function
import numpy as np
import os
import subprocess
import copy

# C.C NOTE: Mixin convention -
# mixin files should be named _name.py and
# contain one Mixin class labeled Mixin.
# see _body.py for more info


# C.C. we can probably remove this after a certain point
# this is necessary for being able to run
# both as __main__ and as package
if __package__ is None or __package__ == '':
    # uses current directory visibility
    import _body
    import _joint
    import _actuator
    import _network
    import _sensor
else:
    # uses current package visibility
    from . import _body
    from . import _joint
    from . import _actuator
    from . import _network
    from . import _sensor


class Simulator(_body.Mixin,
                _joint.Mixin,
                _actuator.Mixin,
                _network.Mixin,
                _sensor.Mixin):
    """Python Interface for ODE robotics simulator

    Attributes
    ----------
    eval_steps     : int (optional)
        Number of simulated steps (default 100)
    dt             : float (optional)
        Length of simulated time in seconds of each step (default 0.01)
    play_blind     : bool (optional)
        Run without graphics (default False)
    play_paused    : bool (optional)
        Start simulation paused (default False)
    draw_joints    : bool ( optional )
        Starts simulation with joints drawn on screen. This can be toggled
        while the simulation is running by pressing 'd'. (default False)
    """

    def __init__(self,
                 eval_steps = 100,
                 dt = 0.01,
                 play_blind = False,
                 play_paused = False,
                 update_network = 1,
                 draw_joints = False,
                 use_textures = True,
                 draw_shadows = True,
                 ):

        # location of this file
        self._this_file_path = os.path.dirname(os.path.abspath(__file__))
        # location of simulator executable
        self._simulator_path = os.path.join(
            self._this_file_path, 'simulator/build')

        self._num_entities = 0
        self._num_actuators = 0

        self._entities = []

        # commands to be sent
        self._strings_to_send = ''

        # playback parameters
        self._play_blind = play_blind
        self._play_paused = play_paused

        # body parameters
        self._current_space = 'None'
        self._current_collision_group = 'None'

        # sim parameters
        self._eval_steps = eval_steps
        self._dt = dt

        # draw parameters
        self._draw_joints = draw_joints
        self._use_textures = use_textures
        self._draw_shadows = draw_shadows

        self._raw_cerr = ''
        self._sensor_data = {}

    def assign_collision(self, group1, group2):
        """Specifies that members of *group1* and *group2* should collide in simulation

        Parameters
        ----------
        group1 : str
            String name of group 1
        group2  : str
            String name of group 2

        Returns
        -------
        bool
            True if successful
        """

        self._send('AssignCollision', group1, group2)
        return True

    def get_all_sensor_data( self ):
        """Outputs all sensor data created by the simulation

        Returns
        -------
        dict
            The sensor dictionary. The keys are is the id of the
            sensor and the value a list containing the sensory 
            information at each time step.
        """
        return copy.copy( self._sensor_data )

    def get_debug_output(self):
        """Returns the debug output from the simulation"""
        return self._strings_to_send + '\n' + self._raw_cerr

    def get_sensor_data(self, sensor_id = None):
        """Returns the sensor data of the specified sensor at each
        time step"""
        
        if sensor_id is None:
            return copy.copy( self._sensor_data )
        else:
            self._assert_sensor(sensor_id, 'sensor_id')
            data_to_return = None
            try:
                data_to_return = copy.copy( self._sensor_data[sensor_id][:] )
            except:
                return None
                # print( 'Invalid sensor_id' )
                # print( 'All Data' )
                # print( self._sensor_data )
                # print( 'raw output' )
                # print( self._raw_cerr )

            assert( data_to_return is not None )

            return data_to_return
    
    def set_current_collision_group(self, group_name):
        """Set the current group name for future bodies to use as default"""
        self._current_collision_group = group_name

    def set_current_space(self, space_name):
        """Set the current space name for future bodies to use as default"""

        self._current_space = space_name

    def set_camera(self, xyz, hpr, tracking = 'none', body_to_track = 0 ):
        """Sets how the camera starts in simulation

        Parameters
        ----------
        xyz : triple float
            Indicates the statring position of the camera
        hpr : triple float
            Heading, Pitch, and Roll of the camera.
        tracking : string
            No tracking, pan tracking, or follow tracking
        body_to_track : body id
            Body to have to camera follow
        """

        assert len(xyz) == 3
        assert len(hpr) == 3
        x, y, z = xyz
        h, p, r = hpr

        if tracking == 'none':
            tracking = 0
        elif tracking == 'pan':
            tracking = 1
        elif tracking == 'follow':
            tracking = 2
        else:
            tracking = 0

        # C.C. parameters oare singular values so 
        # must write out vectors. May change in the future

        self._send_parameter('CameraX', x)
        self._send_parameter('CameraY', y)
        self._send_parameter('CameraZ', z)

        self._send_parameter('CameraH', h)
        self._send_parameter('CameraP', p)
        self._send_parameter('CameraR', r)

        self._send_parameter( 'CameraTracking', tracking )
        self._send_parameter( 'CameraBody', body_to_track )

    def set_friction( self, mu = 'Infinite' ):
        """Set the friction parameter

        Note
        ----
        `mu` is a single parameter in ODEs more complex friction model.
        In general lower `mu` will create a simulation with less friction.

        Parameters
        ----------
        mu : float (optional)
            Determines how friction is calculated in simulation. 
            Higher implies more friction. (Default is 'infinite')
        """

        if mu == 'Infinite':
            mu = -1.0
        assert float( mu ) == mu, 'mu must be Infinite or float'

        self._send_parameter( 'Friction', mu )


    def set_gravity(self, x=0, y=0, z=-9.8):
        """Set the gravity for simulation"""
        self._send_parameter('GravityX', x)
        self._send_parameter('GravityY', y)
        self._send_parameter('GravityZ', z)

    def set_network_update_interval( self, steps_between_evals ):
        """The number of evaluation steps between network updates.

        Allows for larger time steps between computing an iteration of the
        network controller.

        Note
        ----
            May produce unexpected results, use with caution.

        Parameters
        ----------
        steps_between_evals : int
            a update_network value of 1 means the network is updated every
            simulation step, a value of 10 means the network updates
            every 10 eval steps.
        """
        assert( steps_between_evals == int( steps_between_evals ) and
                steps_between_evals > 0 ), ( 'steps_between_evals ' +
                                             'must > 0 and an integer')

        self._send_parameter('NetworkUpdate', steps_between_evals )

    def start(self):
        """Start the simulation"""

        assert(self._play_blind is False or self._eval_steps > 0), (
            'Cannot infinitely play blind. Change play_blind to False '
            'or set eval_steps to a positive number'
        )

        assert(self._play_paused is False or self._play_blind is False), (
            'Cannot play blind and paused. Change truth value of play_blind '
            'or play_paused'
        )

        commands = [self._simulator_path + '/simulator']
        if self._play_blind:
            commands.append('-blind')
        if self._play_paused:
            commands.append('-pause')
        if not self._use_textures:
            commands.append('-notex')
        if not self._draw_shadows:
            commands.append('-noshadow')
            
        # create pipe to simulator
        self.pipe = subprocess.Popen(
            commands,
            bufsize=0,  # necessary to not halt
            stdout=subprocess.PIPE,  # connects stdout
            stderr=subprocess.PIPE,  # connects stderr
            stdin=subprocess.PIPE,  # connects stdin
            universal_newlines=True,  # necessary for 3.x
            cwd=self._simulator_path,  # helps with textures
        )
        # write parameters
        self._send_simulator_parameters()
        self.pipe.stdin.write(self._strings_to_send)
        # finish by writing done
        self.pipe.stdin.write('Done\n')
        # self.pipe.stdin.close()

    def wait_to_finish(self):
        """Communicate with pipe once simulation is complete

        .. note:: for **python 3.x** it is necessary to use this command after `sim.start()`
            in order for the simulation to run properly.
        """

        # for line in iter(self.pipe.stdout.readline, b''):
        #     # print(line)
        #     pass
        # self.pipe.terminate()
        # code = self.pipe.poll()
        data = self.pipe.communicate()
        self._raw_cout = data[0]
        self._raw_cerr = data[1]

        # cut out annoying drawstuff commands
        start_str = 'Simulation test environment v0.02'
        end_str = 'sideways and up.'

        start_index = self._raw_cerr.find(start_str)
        end_index = self._raw_cerr.find(end_str)

        if not (start_index == -1 or end_index == -1):
            self._raw_cerr = self._raw_cerr[:start_index] + \
                self._raw_cerr[end_index + len(end_str):]

        self._read_sensor_data()

    def _read_sensor_data(self):
        # sensor data comes back as a long string of single values delimited by a space 
        # character. This function reads the value and splits the data accordingly
        # into the proper sensor data entry

        sensor_vector = self._raw_cout.split(' ')

        if len(sensor_vector) > 1: # at least one sensor present so contitue
            time_steps = int(sensor_vector.pop(0))

            while(len(sensor_vector) > 0):
                # first pop off id tag of the sensor
                entity_index = int(sensor_vector.pop(0))

                # create list entry in data dict to populate with sensor values
                # for each time step
                self._sensor_data[entity_index] = [0] * time_steps

                for t in range(time_steps):
                    self._sensor_data[entity_index][t] = float(sensor_vector.pop(0))

    def _send(self, command, *args):
        """Append to string containing commands for C++ program to read in.
        
        `command` string should have a corresponding catch on the c++ side in environment.cpp
        Remaining args should be read in by `readFromPython()` method in corresponding objects
        C++ code.
        """
        assert isinstance(command, str), ('Command must be string')

        # each entry is delimited by \n
        string_to_send = command + '\n'
        for arg in args:
            try: # arg is a list or string
                i = iter(arg)
            except: # arg is a single value
                string_to_send += str(arg) + '\n'
            else:
                if isinstance(arg, str):
                    string_to_send += arg + '\n'
                else:
                    string_to_send += '\n'.join(str(entry) for entry in arg) + '\n'
        self._strings_to_send += string_to_send

    def _send_add_command(self, *args):
        self._send('Add', *args)

    def _send_entity(self, entity_type, *args):
        valid_entity_types = ['Entity', 'Body',
                              'Actuator', 'Joint',
                              'Sensor', 'Neuron',
                              'Synapse']
        assert entity_type in valid_entity_types, ('Entity type inputed: ' + str(entity_type) +
                                                   ' must be one of ' + str(valid_entity_types))

        self._entities.append(entity_type)
        entity_id = self._num_entities
        self._send('Entity', *args)
        self._num_entities += 1
        return entity_id

    def _send_parameter(self, *args):
        self._send('Parameter', *args)

    def _send_simulator_parameters(self):
        # send eval steps
        self._send_parameter('EvalSteps', int(self._eval_steps))
        # send DT
        self._send_parameter('DT', self._dt)

        # send initial draw state
        self._send_parameter('DrawJoints', int( self._draw_joints ) )

    def _assert_body(self, id_tag, tag_name=''):
        if (id_tag == -1):
            return
        assert self._entities[id_tag] == 'Body', ('Input id tag ' + str(tag_name) + ': ' +
                                                  str(id_tag) +' does not correspond to body')

    def _assert_actuator(self, id_tag, tag_name=''):
        assert self._entities[id_tag] == 'Actuator', ('Input id tag ' + str(tag_name) + ': ' +
                                                      str(id_tag) +' does not correspond to actuator')

    def _assert_joint(self, id_tag, tag_name=''):
        assert self._entities[id_tag] == 'Joint', ('Input id tag ' + str(tag_name) + ': ' +
                                                   str(id_tag) +' does not correspond to joint')

    def _assert_sensor(self, id_tag, tag_name=''):
        assert self._entities[id_tag] == 'Sensor', ('Input id tag ' + str(tag_name) + ': ' +
                                                    str(id_tag) +' does not correspond to sensor')

    def _assert_neuron(self, id_tag, tag_name=''):
        assert self._entities[id_tag] == 'Neuron', ('Input id tag ' + str(tag_name) + ': ' +
                                                    str(id_tag) +' does not correspond to neuron')

if __name__ == '__main__':

    sim = Simulator(play_blind=False, play_paused=True)

    sim.send_cylinder(capped=False, space='hi')

    sim.start()

    sim.wait_to_finish()

    debug = sim.get_debug_output()
    print(debug)
