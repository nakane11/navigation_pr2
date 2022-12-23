import numpy as np
import rospy
import actionlib
import control_msgs.msg
import nav_msgs.msg

def _wrap_axis(axis):
    """Convert axis to float vector.
    Parameters
    ----------
    axis : list or numpy.ndarray or str or bool or None
        rotation axis indicated by number or string.
    Returns
    -------
    axis : numpy.ndarray
        conveted axis
    Examples
    --------
    >>> from skrobot.coordinates.math import _wrap_axis
    >>> _wrap_axis('x')
    array([1, 0, 0])
    >>> _wrap_axis('y')
    array([0, 1, 0])
    >>> _wrap_axis('z')
    array([0, 0, 1])
    >>> _wrap_axis('xy')
    array([1, 1, 0])
    >>> _wrap_axis([1, 1, 1])
    array([1, 1, 1])
    >>> _wrap_axis(True)
    array([0, 0, 0])
    >>> _wrap_axis(False)
    array([1, 1, 1])
    """
    if isinstance(axis, str):
        if axis in ['x', 'xx']:
            axis = np.array([1, 0, 0])
        elif axis in ['y', 'yy']:
            axis = np.array([0, 1, 0])
        elif axis in ['z', 'zz']:
            axis = np.array([0, 0, 1])
        elif axis == '-x':
            axis = np.array([-1, 0, 0])
        elif axis == '-y':
            axis = np.array([0, -1, 0])
        elif axis == '-z':
            axis = np.array([0, 0, -1])
        elif axis in ['xy', 'yx']:
            axis = np.array([1, 1, 0])
        elif axis in ['yz', 'zy']:
            axis = np.array([0, 1, 1])
        elif axis in ['zx', 'xz']:
            axis = np.array([1, 0, 1])
        else:
            raise NotImplementedError
    elif isinstance(axis, list):
        if not len(axis) == 3:
            raise ValueError
        axis = np.array(axis)
    elif isinstance(axis, np.ndarray):
        if not axis.shape == (3,):
            raise ValueError
    elif isinstance(axis, bool):
        if axis is True:
            return np.array([0, 0, 0])
        else:
            return np.array([1, 1, 1])
    elif axis is None:
        return np.array([1, 1, 1])
    else:
        raise ValueError
    return axis

def quaternion_norm(q):
    """Return the norm of quaternion.
    Parameters
    ----------
    q : list or numpy.ndarray
        [w, x, y, z] order
    Returns
    -------
    norm_q : float
        quaternion norm of q
    Examples
    --------
    >>> from skrobot.coordinates.math import quaternion_norm
    >>> q = [1, 1, 1, 1]
    >>> quaternion_norm(q)
    2.0
    >>> q = [0, 0.7071067811865476, 0, 0.7071067811865476]
    >>> quaternion_norm(q)
    1.0
    """
    q = np.array(q)
    if q.ndim == 1:
        norm_q = np.sqrt(np.dot(q.T, q))
    elif q.ndim == 2:
        norm_q = np.sqrt(np.sum(q * q, axis=1, keepdims=True))
    else:
        raise ValueError
    return norm_q

def quaternion_normalize(q):
    """Return the normalized quaternion.
    Parameters
    ----------
    q : list or numpy.ndarray
        [w, x, y, z] order
    Returns
    -------
    normalized_q : numpy.ndarray
        normalized quaternion
    Examples
    --------
    >>> from skrobot.coordinates.math import quaternion_normalize
    >>> from skrobot.coordinates.math import quaternion_norm
    >>> q = quaternion_normalize([1, 1, 1, 1])
    >>> quaternion_norm(q)
    1.0
    """
    q = np.array(q)
    normalized_q = q / quaternion_norm(q)
    return normalized_q

def quaternion2matrix(q, normalize=False):
    """Returns matrix of given quaternion.
    Parameters
    ----------
    quaternion : list or numpy.ndarray
        quaternion [w, x, y, z] order
    normalize : bool
        if normalize is True, input quaternion is normalized.
    Returns
    -------
    rot : numpy.ndarray
        3x3 rotation matrix
    Examples
    --------
    >>> import numpy
    >>> from skrobot.coordinates.math import quaternion2matrix
    >>> quaternion2matrix([1, 0, 0, 0])
    array([[1., 0., 0.],
           [0., 1., 0.],
           [0., 0., 1.]])
    """
    q = np.array(q)
    if normalize:
        q = quaternion_normalize(q)
    else:
        norm = quaternion_norm(q)
        if not np.allclose(norm, 1.0):
            raise ValueError("quaternion q's norm is not 1")
    if q.ndim == 1:
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]

        m = np.zeros((3, 3))
        m[0, 0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3
        m[0, 1] = 2 * (q1 * q2 - q0 * q3)
        m[0, 2] = 2 * (q1 * q3 + q0 * q2)

        m[1, 0] = 2 * (q1 * q2 + q0 * q3)
        m[1, 1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3
        m[1, 2] = 2 * (q2 * q3 - q0 * q1)

        m[2, 0] = 2 * (q1 * q3 - q0 * q2)
        m[2, 1] = 2 * (q2 * q3 + q0 * q1)
        m[2, 2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    elif q.ndim == 2:
        m = np.zeros((q.shape[0], 3, 3), dtype=np.float64)
        m[:, 0, 0] = q[:, 0] * q[:, 0] + \
            q[:, 1] * q[:, 1] - q[:, 2] * q[:, 2] - q[:, 3] * q[:, 3]
        m[:, 0, 1] = 2 * (q[:, 1] * q[:, 2] - q[:, 0] * q[:, 3])
        m[:, 0, 2] = 2 * (q[:, 1] * q[:, 3] + q[:, 0] * q[:, 2])

        m[:, 1, 0] = 2 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3])
        m[:, 1, 1] = q[:, 0] * q[:, 0] - \
            q[:, 1] * q[:, 1] + q[:, 2] * q[:, 2] - q[:, 3] * q[:, 3]
        m[:, 1, 2] = 2 * (q[:, 2] * q[:, 3] - q[:, 0] * q[:, 1])

        m[:, 2, 0] = 2 * (q[:, 1] * q[:, 3] - q[:, 0] * q[:, 2])
        m[:, 2, 1] = 2 * (q[:, 2] * q[:, 3] + q[:, 0] * q[:, 1])
        m[:, 2, 2] = q[:, 0] * q[:, 0] - \
            q[:, 1] * q[:, 1] - q[:, 2] * q[:, 2] + q[:, 3] * q[:, 3]
    return m

def rpy_angle(matrix):
    """Decomposing a rotation matrix to yaw-pitch-roll.
    Parameters
    ----------
    matrix : list or numpy.ndarray
        3x3 rotation matrix
    Returns
    -------
    rpy : tuple(numpy.ndarray, numpy.ndarray)
        pair of rpy in yaw-pitch-roll order.
    Examples
    --------
    >>> import numpy as np
    >>> from skrobot.coordinates.math import rpy_matrix
    >>> from skrobot.coordinates.math import rpy_angle
    >>> yaw = np.pi / 2.0
    >>> pitch = np.pi / 3.0
    >>> roll = np.pi / 6.0
    >>> rot = rpy_matrix(yaw, pitch, roll)
    >>> rpy_angle(rot)
    (array([1.57079633, 1.04719755, 0.52359878]),
     array([ 4.71238898,  2.0943951 , -2.61799388]))
    """
    if np.sqrt(matrix[1, 0] ** 2 + matrix[0, 0] ** 2) < _EPS:
        a = 0.0
    else:
        a = np.arctan2(matrix[1, 0], matrix[0, 0])
    sa = np.sin(a)
    ca = np.cos(a)
    b = np.arctan2(-matrix[2, 0], ca * matrix[0, 0] + sa * matrix[1, 0])
    c = np.arctan2(sa * matrix[0, 2] - ca * matrix[1, 2],
                   -sa * matrix[0, 1] + ca * matrix[1, 1])
    rpy = np.array([a, b, c])

    a = a + np.pi
    sa = np.sin(a)
    ca = np.cos(a)
    b = np.arctan2(-matrix[2, 0], ca * matrix[0, 0] + sa * matrix[1, 0])
    c = np.arctan2(sa * matrix[0, 2] - ca * matrix[1, 2],
                   -sa * matrix[0, 1] + ca * matrix[1, 1])
    return rpy, np.array([a, b, c])

class Coordinates(object):

    """Coordinates class to manipulate rotation and translation.
    Parameters
    ----------
    pos : list or numpy.ndarray or None
        shape of (3,) translation vector. or
        4x4 homogeneous transformation matrix.
        If the homogeneous transformation matrix is given,
        `rot` will be overwritten.
        If this value is `None`, set [0, 0, 0] vector as default.
    rot : list or numpy.ndarray or None
        we can take 3x3 rotation matrix or
        [yaw, pitch, roll] or
        quaternion [w, x, y, z] order
        If this value is `None`, set the identity matrix as default.
    name : str or None
        name of this coordinates
    check_validity : bool (optional)
        Default `True`.
        If this value is `True`, check whether an input rotation
        and an input translation are valid.
    """

    def __init__(self,
                 pos=None,
                 rot=None,
                 name=None,
                 hook=None,
                 check_validity=True):
        if check_validity:
            if (isinstance(pos, list) or isinstance(pos, np.ndarray)):
                T = np.array(pos, dtype=np.float64)
                if T.shape == (4, 4):
                    pos = T[:3, 3]
                    rot = T[:3, :3]
            if rot is None:
                self._rotation = np.eye(3)
            else:
                self.rotation = rot
            if pos is None:
                self._translation = np.array([0, 0, 0])
            else:
                self.translation = pos
        else:
            if rot is None:
                self._rotation = np.eye(3)
            else:
                self._rotation = rot
            if pos is None:
                self._translation = np.array([0, 0, 0])
            else:
                self._translation = pos
        if name is None:
            name = ''
        self.name = name
        self.parent = None
        self._hook = hook

    def inverse_transform_vector(self, vec):
            """Transform vector in world coordinates to local coordinates
            Parameters
            ----------
            vec : numpy.ndarray
                3d vector.
                We can take batch of vector like (batch_size, 3)
            Returns
            -------
            transformed_point : numpy.ndarray
                transformed point
            """
            q = np.array([q for q in self.rotation])
            if np.abs(np.linalg.norm(q) - 1.0) > 1e-3:
                raise ValueError('Invalid quaternion. Must be '
                                 'norm 1.0, get {}'.
                                 format(np.linalg.norm(q)))
            rotation = quaternion2matrix(q)
            vec = np.array(vec, dtype=np.float64)
            if vec.ndim == 2:
                return (np.matmul(rotation.T, vec.T)
                        - np.matmul(
                            rotation.T, self.translation).reshape(3, -1)).T
            return np.matmul(rotation.T, vec) - \
                np.matmul(rotation.T, self.translation)

    def difference_position(self, coords,
                            translation_axis=True):
        """Return differences in positoin of given coords.
        Parameters
        ----------
        coords : skrobot.coordinates.Coordinates
            given coordinates
        translation_axis : str or bool or None (optional)
            we can take 'x', 'y', 'z', 'xy', 'yz', 'zx', 'xx', 'yy', 'zz',
            True or False(None).
        Returns
        -------
        dif_pos : numpy.ndarray
            difference position of self coordinates and coords
            considering translation_axis.
        Examples
        --------
        >>> from skrobot.coordinates import Coordinates
        >>> from skrobot.coordinates import transform_coords
        >>> from numpy import pi
        >>> c1 = Coordinates().translate([0.1, 0.2, 0.3]).rotate(
        ...          pi / 3.0, 'x')
        >>> c2 = Coordinates().translate([0.3, -0.3, 0.1]).rotate(
        ...          pi / 2.0, 'y')
        >>> c1.difference_position(c2)
        array([ 0.2       , -0.42320508,  0.3330127 ])
        >>> c1 = Coordinates().translate([0.1, 0.2, 0.3]).rotate(0, 'x')
        >>> c2 = Coordinates().translate([0.3, -0.3, 0.1]).rotate(
        ...          pi / 3.0, 'x')
        >>> c1.difference_position(c2)
        array([ 0.2, -0.5, -0.2])
        """
        dif_pos = self.inverse_transform_vector(coords.worldpos())
        translation_axis = _wrap_axis(translation_axis)
        dif_pos[translation_axis == 1] = 0.0
        return dif_pos

    def rpy_angle(self):
        """Return a pair of rpy angles of this coordinates.
        Returns
        -------
        rpy_angle(self.rotation) : tuple(numpy.ndarray, numpy.ndarray)
            a pair of rpy angles. See also skrobot.coordinates.math.rpy_angle
        Examples
        --------
        >>> import numpy as np
        >>> from skrobot.coordinates import Coordinates
        >>> c = Coordinates().rotate(np.pi / 2.0, 'x').rotate(np.pi / 3.0, 'z')
        >>> r.rpy_angle()
        (array([ 3.84592537e-16, -1.04719755e+00,  1.57079633e+00]),
        array([ 3.14159265, -2.0943951 , -1.57079633]))
        """
        return rpy_angle(self.rotation)

    def worldpos(self):
        """Return translation of this coordinate
        See also skrobot.coordinates.Coordinates.translation
        Returns
        -------
        self.translation : numpy.ndarray
            translation of this coordinate
        """
        return self.translation
        
class ROSRobotMoveBaseInterface(object):
    def __init__(self, *args, **kwargs):
        self.base_controller_action_name = kwargs.pop(
            'base_controller_action_name',
            "/base_controller/follow_joint_trajectory")
        self.odom_topic = kwargs.pop('odom_topic', '/base_odometry/odom')
        self.move_base_trajectory_action = actionlib.SimpleActionClient(
                self.base_controller_action_name,
                control_msgs.msg.FollowJointTrajectoryAction)
        if self.move_base_trajectory_action.wait_for_server(
                rospy.Duration(10)) is False:
            rospy.logwarn('{} is not found'.format(
                self.base_controller_action_name))
            self.move_base_trajectory_action = None
        self.odom_msg = None
        self.odom_subscriber = rospy.Subscriber(
            self.odom_topic, nav_msgs.msg.Odometry,
            callback=self.odom_callback,
            queue_size=1)

    def odom(self):
        """Return Coordinates of this odom
        Returns
        -------
        odom_coords : skrobot.coordinates.Coordinates
            coordinates of odom.
        """
        if self.odom_msg is None:
            raise RuntimeError(
                'odom is not set. Please check odom topic {}'
                ' is published.'.format(self.odom_topic))
        pos = (self.odom_msg.pose.pose.position.x,
               self.odom_msg.pose.pose.position.y,
               self.odom_msg.pose.pose.position.z)
        q_wxyz = (self.odom_msg.pose.pose.orientation.w,
                  self.odom_msg.pose.pose.orientation.x,
                  self.odom_msg.pose.pose.orientation.y,
                  self.odom_msg.pose.pose.orientation.z)
        return Coordinates(pos=pos, rot=q_wxyz)
        
    def odom_callback(self, msg):
        """ROS's subscriber callback for odom.
        Parameters
        ----------
        msg : nav_msgs.msg.Odometry
            odometry message.
        """
        self.odom_msg = msg
        
    def go_pos_unsafe(self, x=0.0, y=0.0, yaw=0.0, wait=False):
        """Move Robot using MoveBase
        Parameters
        ----------
        x : float
            move distance with respect to x axis. unit is [m].
        y : float
            move distance with respect to y axis. unit is [m].
        yaw : float
            rotate angle. unit is [rad].
        wait : bool
            if wait is True, wait until stop go_pos_unsafe
        """
        self.go_pos_unsafe_no_wait(x=x, y=y, yaw=yaw)
        if wait is True:
            return self.go_pos_unsafe_wait()
        else:
            return True

    def go_pos_unsafe_no_wait(self, x=0.0, y=0.0, yaw=0.0):
        """Move Robot using MoveBase
        Parameters
        ----------
        x : float
            move distance with respect to x axis. unit is [m].
        y : float
            move distance with respect to y axis. unit is [m].
        yaw : float
            rotate angle. unit is [rad].
        """
        if self.move_base_trajectory_action is None:
            rospy.logwarn("go_pos_unsafe is disabled. "
                          'move_base_trajectory_action is not found')
            return True
        maxvel = 0.295
        maxrad = 0.495
        ratio = 0.8
        sec = max(np.linalg.norm([x, y]) / (maxvel * ratio),
                  abs(yaw) / (maxrad * ratio),
                  1.0)
        self.go_pos_unsafe_goal_msg = self.move_trajectory(
            x, y, yaw,
            sec, stop=True)
        return self.move_base_trajectory_action.send_goal(
            self.go_pos_unsafe_goal_msg.goal)

    def go_pos_unsafe_wait(self, wait_counts=3):
        maxvel = 0.295
        maxrad = 0.495
        ratio = 0.8
        counter = 0
        if self.go_pos_unsafe_goal_msg is None:
            return False
        if self.move_base_trajectory_action is None:
            rospy.logwarn("go_pos_unsafe_wait is disabled. "
                          'move_base_trajectory_action is not found')
            return True
        while counter < wait_counts:
            action_ret = self.move_base_trajectory_action.wait_for_result()
            if action_ret is False:
                return False

            goal_position = np.array(
                self.go_pos_unsafe_goal_msg.goal.
                trajectory.points[1].positions,
                dtype=np.float32)
            odom = self.odom
            odom_pos = odom.translation
            odom_angle = odom.rpy_angle()[0][0]
            diff_position = goal_position - (
                odom_pos + np.array((0, 0, odom_angle)))
            v = rotate_vector(
                np.array((diff_position[0], diff_position[1], 0.0)),
                -odom_angle, 'z') - np.array((0, 0, odom_angle))
            x = v[0]
            y = v[1]
            yaw = diff_position[2]
            if yaw > 2 * np.pi:
                yaw = yaw - 2 * np.pi
            if yaw < - 2 * np.pi:
                yaw = yaw + 2 * np.pi

            sec = max(np.linalg.norm([x, y]) / (maxvel * ratio),
                      abs(yaw) / (maxrad * ratio))
            sec = max(sec, 1.0)
            step = 1.0 / sec

            rospy.loginfo("                diff-pos {} {}, diff-angle {}".
                          format(x, y, yaw))
            if np.sqrt(x * x + y * y) <= 0.025 and \
               abs(yaw) <= np.deg2rad(2.5) and \
               counter > 0:  # try at least 1 time
                self.go_pos_unsafe_goal_msg = None
                return True

            self.go_pos_unsafe_goal_msg = self.move_trajectory(
                x * step, y * step, yaw * step, sec, stop=True)
            self.move_base_trajectory_action.send_goal(
                self.go_pos_unsafe_goal_msg.goal)
            counter += 1
        self.go_pos_unsafe_goal_msg = None
        return True
