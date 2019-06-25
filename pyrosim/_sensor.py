
class Mixin(object):
    def _send_sensor(self, *args):
        return self._send_entity('Sensor', *args)

    def send_distance_to_sensor( self, body1_id, body2_id, write_back = True ):
        """Send a distance sensor which measures the distance between body1 and body2


        Parameters
        ----------
        """

        self._assert_body( body1_id, 'body1_id' )
        self._assert_body( body2_id, 'body2_id' )

        return self._send_sensor( 'DistanceToSensor', body1_id, body2_id, int( write_back ))

    # ------- LIGHT SENSOR ---------------------
    def send_light_sensor( self, body_id, write_back = True ):
        """Send a light sensor embedded within a body

        Parameters
        ----------
        body_id     : int
            The id tag of the specified body
        write_back  : bool
            If True, write back to python after simulation. (Default True)

        Returns
        -------
        int
            The id tag of the sensor
        """
        self._assert_body( body_id, 'body_id' )
        return self._send_sensor( 'LightSensor', body_id, int( write_back ) )

    # ------------ POSITION SENSOR -----------------------------
    def send_position_sensor(self, body_id, which_dimension='x', write_back = True):
        """Send position sensor which tracks the body specified in body_id

        Parameters
        ----------
        body_id         : int
            The id tag of the specifed body
        which_dimension : str or int (optional)
            Specifies which dimension to track: 'x', 'y', or 'z'.
            You can also use 0, 1, or 2. (default is 'x')
        write_back  : bool
            If True, write back to python after simulation. (Default True)
        Returns
        -------
        int
            The id tag of the sensor
        """
        if which_dimension == 'x':
            which_dimension = 0
        elif which_dimension == 'y':
            which_dimension = 1
        elif which_dimension == 'z':
            which_dimension = 2
        else:
            which_dimension = -1

        assert which_dimension >=0 and which_dimension <=2, ('Which dimension must be x, y, or z')
        self._assert_body(body_id, 'body_id')

        return  self._send_sensor('PositionSensor', body_id, which_dimension, int( write_back ) )

    def send_position_x_sensor(self, body_id, write_back = True ):
        """Send x position sensor which tracks the body specified in *body_id*"""
        return self.send_position_sensor(body_id, which_dimension='x', write_back = write_back)

    def send_position_y_sensor(self, body_id, write_back = True ):
        """Send y position sensor which tracks the body specified in *body_id*"""
        return self.send_position_sensor(body_id, which_dimension='y', write_back = write_back)

    def send_position_z_sensor(self, body_id, write_back = True):
        """Send z position sensor which tracks the body specified in *body_id*"""
        return self.send_position_sensor(body_id, which_dimension='z', write_back = write_back)

    # ----------- RAY SENSOR ------------------------------------
    def send_ray_sensor(self, ray_id, which_sense='d', write_back=True):
        """
        Sends a ray sensor to the simulator.

        Ray sensors connect to ray entities. They can return the distance
        to this object or the color of this object (r, g, b).

        Parameters
        ----------
        ray_id      :  int
            The id tag of the ray to attach to.
        which_sense : str or int (optional)
            Specifies which sense of the ray to return.
            Use 'd' for distance, 'r' for red, 'g' for green, 'b' for blue.
        write_back  : bool
            If True, write back to python after simulation. (Default True)
        Returns
        -------
        int
            the id tag of the sensor.
        """
        if which_sense == 'd':
            which_sense = 0
        elif which_sense == 'r':
            which_sense = 1
        elif which_sense == 'g':
            which_sense = 2
        elif which_sense == 'b':
            which_sense = 3

        assert which_sense >=0 and which_sense <=3, ('Which sense must be d, r, g, b or 0, 1, 2, 3')
        self._assert_body(ray_id, 'ray_id')
        return self._send_sensor('RaySensor', ray_id, which_sense, int( write_back) )

    def send_ray_distance_sensor(self, ray_id, write_back = True ):
        """Send a distance sensor attached to the ray at *ray_id*"""
        return self.send_ray_sensor(ray_id, which_sense='d', write_back = write_back)

    def send_ray_red_sensor(self, ray_id, write_back = True ):
        """Send a red color sensor attached to the ray at *ray_id*"""
        return self.send_ray_sensor(ray_id, which_sense='r', write_back = write_back)

    def send_ray_green_sensor(self, ray_id, write_back = True ):
        """Send a green color sensor attached to the ray at *ray_id*"""
        return self.send_ray_sensor(ray_id, which_sense='g', write_back = write_back)

    def send_ray_blue_sensor(self, ray_id, write_back = True ):
        """Send a blue color sensor attached to the ray at *ray_id*"""
        return self.send_ray_sensor(ray_id, which_sense='b', write_back = write_back )

    # --------------- TOUCH SENSOR -----------------------------------
    def send_touch_sensor(self, body_id, write_back = True):
        """Attach a touch sensor to a body

        Parameters
        ----------
        body_id   : int
            The id tag of the body to attach the sensor to

        Returns
        -------
        int
            The id tag of the sensor
        write_back  : bool
            If True, write back to python after simulation. (Default True)
        """

        self._assert_body(body_id)

        return self._send_sensor('TouchSensor', body_id, int( write_back ) )


    # ------------- QUATERNION SENSOR --------------------------------
    def send_quaternion_sensor(self, body_id, which_sense='a', write_back = True):
        """Attach a vestibular sensor returning the quaternion of the body.

        Quaternions is 4 element vector which can represent the rotation of 
        a body. It's components are a, b, c, and d. You can also use
        w, x, y, and z. Read more here:
        https://en.wikipedia.org/wiki/Quaternion
        """
        if   which_sense == 'a' or which_sense == 'w':
            which_sense = 0
        elif which_sense == 'b' or which_sense == 'x':
            which_sense = 1
        elif which_sense == 'c' or which_sense == 'y':
            which_sense = 2
        elif which_sense == 'd' or which_sense == 'z':
            which_sense = 3

        assert (which_sense >= 0 and which_sense <= 3)
        self._assert_body(body_id)

        return self._send_sensor('QuaternionSensor', body_id, which_sense, int( write_back ))

    def send_quaternion_a_sensor(self, body_id, write_back = True ):
        return self.send_quaternion_sensor(body_id, 'a', write_back )

    def send_quaternion_b_sensor(self, body_id, write_back = True):
        return self.send_quaternion_sensor(body_id, 'b', write_back )

    def send_quaternion_c_sensor(self, body_id, write_back = True):
        return self.send_quaternion_sensor(body_id, 'c', write_back )

    def send_quaternion_d_sensor(self, body_id, write_back = True):
        return self.send_quaternion_sensor(body_id, 'd', write_back)

    def send_proprioceptive_sensor(self, joint_id, write_back = True ):
        """Attach a proprioceptive sensor to the joint.

        Proprioceptive sensors return the value of
        the joint at each time step. For example, a hinge
        joint will return the angle and a slider joint
        will return the position offset.

        .. note::
            Currently not implemented for universal or ball joints.
            Attached proprioceptive sensors will only return 0.

        Parameters
        ----------
        joint_id    : int
            The id tag of the joint to attach the sensor to
        write_back  : bool
            If True, write back to python after simulation. (Default True)
        Returns
        -------
        int
            The id tag of the sensor
        """

        self._assert_joint(joint_id)

        return self._send_sensor('ProprioceptiveSensor', joint_id, int( write_back ) )

    # ----- IS SEEN SENSOR ----------------------------------
    def send_is_seen_sensor( self, body_id, write_back = True ):
        """Attach a sensor to a body which reports a 1 when 'seen' by
        a ray sensor and a 0 when not 'seen' 
        
        Parameters
        ----------
        body_id    : int
            The id tag of the body to attach sensor to
        write_back : bool
            If True, write back to python after simulation. (Default True)

        Returns
        -------
        int
            The id tag of the sensor
        """

        self._assert_body( body_id )

        return self._send_sensor( 'IsSeenSensor', body_id, int( write_back ) )
