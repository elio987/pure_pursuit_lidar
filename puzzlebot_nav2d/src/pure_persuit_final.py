#!/usr/bin/env python
import sys
import rospy
import numpy as np
from functools import partial
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
import py_trees

def go_to_point_controller(x, y, vmax, Kth, alpha):
    """
    Calculates the desired linear- and angular velocities to move the robot to a point.
    Used for the final step in the pure pursuit to approach and stay at the goal position.

    Arguments
    ---------
    x     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    y     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    vmax  :  float
      The maximum linear velocity, in m/s
    Kth   :  float
      Gain for the direction controller
    alpha :  float
      Parameter for calculating the linear velocity as it approaches the goal

    Returns
    ---------
    w   :  float
      The angular velocity in rad/s
    v   :  float
      The linear velocity in m/s
    d   :  float
      The distance to the goal

    """

    #---------------------------------------------------------
    # YOUR CODE HERE
    """
    x = np.round(x,2)
    y = np.round(y,2)
    if x == -0.0:
        x = 0.0
    if y == -0.0:
        y = 0.0"""
    w = Kth * np.arctan2(y,x)
    d = np.sqrt(x**2 + y**2)
    v = vmax*(1-np.exp(-alpha*(d**2)))
    #---------------------------------------------------------

    return w, v, d

def steer_towards_point_controller(x, y, v):
    """
    Given an intermediate goal point and a (constant) linear velocity, calculates the
    angular velocity that will steer the robot towards the goal such that the robot
    moves in a circular arc that passes through the intermediate goal point.

    Arguments
    ---------
    x     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    y     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    v     :  float
      The linear velocity in m/s

    Returns
    ---------
    w   :  float
      The angular velocity in rad/s

    Tests
    -----
    1) Goal point directly to the left of the robot. Move in half circle to get there
    >>> w = steer_towards_point_controller(0, 2, 1)
    >>> "w = %0.2f" %(w,)
    'w = 1.00'

    2) Goal at (1, -1). Move in quarter circle towards the right to get there.
    >>> w = steer_towards_point_controller(1, -1, 1)
    >>> "w = %0.2f" %(w,)
    'w = -1.00'


    """
    L = x**2 + y**2
    fi = (2*y)/L
    w = np.round(fi * v,2)
    #---------------------------------------------------------

    return w

def get_goal_point(p0, p1, L):
    """
    Returns the intermediate goal point for the pure pursuit algorithm. If no point
    on the line going through p0 and p1 is at distance L from the origin, then the
    returned beta should be a nan.

    Arguments
    ---------
    p0  :  array-like (2,)
         The current waypoint.
    p1  :  array-like (2,)
         The next waypoint.
    L   :  float\n",
         The look-ahead distance

    Returns\n",
    -------\n",
    pg   :  ndarray (2,)
          The intermediate goal point
    beta :  float
           The value giving the position of the goal point on the line connectin p0 and p1.
    """

    p0 = np.asarray(p0)
    p1 = np.asarray(p1)
    w = p1-p0

    a = np.dot(w, w)
    b = 2*np.dot(p0, w)
    c = np.dot(p0, p0) - L**2

    d = b**2 - 4*a*c
    if d < 0:
        pg = p0
        beta = np.nan
    else:
        #---------------------------------------------------------
        # YOUR CODE HERE
        coeff = [a,b,c]
        beta = np.round(max(np.roots(coeff)),2)
        if np.iscomplex(beta):
            beta = np.nan
            pg = p0
        elif beta >= 1:
            pg = p1
        elif (beta< 1)and(beta > 0):
            pg = np.round(p0 + beta*w,2)
        #---------------------------------------------------------

    return pg, beta

class Go2Point(py_trees.behaviour.Behaviour):
    """
    Takes a point to go to as input and will go towards that point until within
    given distance.
    """
    def __init__(self, name="Go2Point"):
        """
        Default construction.
        """
        super(Go2Point, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, goal, distance, cmd_vel_pub, tf_buffer, robot_frame='base_link',
              vel_max=0.6, K_theta=4.0, alpha=4.0):
        """
        Arguments
        ---------
          goal        :  PointStamped
             The goal point.
          distance    :  float
             When inside this distance, Success is returned.
          cmd_vel_pub :  Publisher
             Publisher that publishes to the /cmd_vel topic
          tf_buffer   :  tf2_ros.Buffer
             Transform buffer for transforming the waypoints.
          robot_frame :  String
             The name of reference frame of the robot.
          vel_max     :  float
             The maximum linear velocity to command.
          K_theta     :  float
             The gain for the angular part of the controller.
          alpha       :  float
             The parameter for setting the linear velocity.

        """
        self.goal = goal
        self.dist = distance
        self.cmd_vel_pub = cmd_vel_pub
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.vel_max = vel_max
        self.K_theta = K_theta
        self.alpha = alpha

        self.msg = geometry_msgs.msg.Twist()

        return True

    def initialise(self):
        """
        Since simple proportional control, no initialization needed. In case
        of PI, PD, or PID, the controller would have some state that should
        be initialized here.
        """
        pass

    def update(self):
        """
        Do the work:
        1) Transform waypoint to robot-centric frame
        2) Compute the control signal
        3) Publish the control signal
        """

        #----------------------------------------------------
        # YOUR CODE HERE
        # Transform the point to the robot reference frame
        self.goal.header.stamp = rospy.Time(0)
        print(self.goal.point.x,self.goal.point.y)
        goal_b = self.tf_buffer.transform(self.goal,self.robot_frame,timeout=rospy.Duration(10.0))
        #----------------------------------------------------

        w, v, dist = go_to_point_controller(goal_b.point.x, goal_b.point.y,
                                            self.vel_max, self.K_theta, self.alpha)

        if dist < self.dist:
            # Within the desired distance, so return success
            return py_trees.Status.SUCCESS
        else:
            #----------------------------------------------------
            # YOUR CODE HERE
            # Publish the velocity command to cmd_vel
            self.msg.linear.x = v # etc.
            self.msg.angular.z = w
            self.cmd_vel_pub.publish(self.msg)
            #----------------------------------------------------

            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        """
        Nothing to clean up
        """
        pass

class PursuitGoal(py_trees.behaviour.Behaviour):
    """
    Takes two waypoints (current and next) and goes toward a point on
    the line joining the two waypoints and at the look-ahead distance.
    Returns SUCCESS when the next intermediate goal point is past the
    next waypoint.
    Returns RUNNING when still moving along the trajectory between the
    two waypoints.
    Returns FAILURE if no point on the trajectory is at the
    look-ahead distance from the robot.
    """

    def __init__(self, name="PursuitGoal"):
        """
        Default construction.
        """
        super(PursuitGoal, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, wp0, wp1, look_ahead_distance,
              cmd_vel_pub, tf_buffer, robot_frame='base_link',
              vel_max=0.6):
        """
        Arguments
        ---------
          wp0        :  array-like (2,)
             The current waypoint.
          wp1        :  array-like (2,)
             The next waypoint.
          look_ahead_distance    :  float
             The main parameter of the pure pursuit algorithm
          cmd_vel_pub :  Publisher
             Publisher that publishes to the /cmd_vel topic
          tf_buffer   :  tf2_ros.Buffer
             Transform buffer for transforming the waypoints.
          robot_frame :  String
             The name of reference frame of the robot.
          vel_max     :  float
             The maximum linear velocity to command.
        """
        self.wp0 = wp0
        self.wp1 = wp1
        self.L = look_ahead_distance
        self.cmd_vel_pub = cmd_vel_pub
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.vel_max = vel_max

        self.msg = geometry_msgs.msg.Twist()

        return True

    def initialise(self):
        """
        Since simple proportional control, no initialization needed. In case
        of PI, PD, or PID, the controller would have some state that should
        be initialized here.
        """
        pass

    def update(self):
        """
        Do the work:
        1) Transform waypoints to robot-centric frame
        2) Compute the control signal
        3) Publish the control signal
        """

        #------------------------------------------------------------
        # YOUR CODE HERE
        # Transform the two waypoints to the robot reference frame
        self.wp0.header.stamp = rospy.Time(0)
        self.wp1.header.stamp = rospy.Time(0)
        wp0_b = self.tf_buffer.transform(self.wp0,self.robot_frame,timeout=rospy.Duration(10.0))
        wp1_b = self.tf_buffer.transform(self.wp1,self.robot_frame,timeout=rospy.Duration(10.0))
        #------------------------------------------------------------

        pg, beta = get_goal_point([wp0_b.point.x, wp0_b.point.y],
                                  [wp1_b.point.x, wp1_b.point.y],
                                  self.L)
        #Distance from the robot to the next waypoint
        error = np.sqrt(((pg[0]-wp1_b.point.x)**2)+((pg[1]-wp1_b.point.y)**2))
        if (beta > 1.0) and (error < 0.05):
            # Succeeded in moving along the trajectory from current to next waypoint
            # Time to move on to next pair of waypoints
            return py_trees.Status.SUCCESS

        elif (beta > 1.0) and (error > 0.05):
            # Succeeded in moving along the trajectory from current to next waypoint
            # Time to move on to next pair of waypoints
            return py_trees.Status.FAILURE

        elif np.isnan(beta):
            # Not able to find point on trajectory at look-ahead-distance
            # from the robot. Means failure.
            return py_trees.Status.FAILURE

        else:
            w = steer_towards_point_controller(pg[0], pg[1], self.vel_max)
            #----------------------------------------------------
            # YOUR CODE HERE
            # Publish the velocity command to cmd_vel
            self.msg.angular.z = w
            self.msg.linear.x = self.vel_max
            self.cmd_vel_pub.publish(self.msg)

            #----------------------------------------------------

            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        """
        Nothing to clean up
        """
        pass

class CarAtGoal(py_trees.behaviour.Behaviour):
    """
    Takes the last waypoints and gets the transformation from the current point to the last point,
    if we are close to the last point we stop the car and return SUCCESS, if we are not near the point we return
    FAILURE
    """

    def __init__(self, name="CarAtGoal"):
        super(CarAtGoal, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,wpf,tf_buffer,cmd_vel_pub, robot_frame='base_link'):
        #Lastwaypoint
        self.wpf = wpf
        self.tf_buffer = tf_buffer
        #Frame of the robot
        self.robot_frame = robot_frame
        #Velocity control
        self.cmd_vel_pub = cmd_vel_pub
        self.msg = geometry_msgs.msg.Twist()
        return True

    def initialise(self):
        pass

    def update(self):
        #We get the transformation from the robot to the last waypoint
        self.wpf.header.stamp = rospy.Time(0)
        error = self.tf_buffer.transform(self.wpf,self.robot_frame,timeout=rospy.Duration(10.0))
        #We get the euclidean from the robot to the last waypoint
        error = np.sqrt(((error.point.x)**2)+((error.point.y)**2))
        #If "error is bigger than the tolerance we are not at the current waypoint"
        if error > 0.45:
            return py_trees.Status.FAILURE
        #If we are near the last waypoint we stop the car and send SUCCESS
        else:
            self.msg.linear.x = 0
            self.msg.angular.z = 0
            self.cmd_vel_pub.publish(self.msg)
            return py_trees.Status.SUCCESS
    def terminate(self, new_status):
        pass

class Fallback(py_trees.behaviour.Behaviour):
    """
    If we are at the waypoint and the real distance from the car to the real waypoint
    is bigger than the tolerance we go backward until we are inside the tolerance.
    We get the next waypoint and frame from the robot.
    """

    def __init__(self, name="Fallback"):
        super(Fallback, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, wpf,cmd_vel_pub, tf_buffer, robot_frame='base_link',vel_max=0.6):
        #The next waypoint
        self.wpf = wpf
        self.cmd_vel_pub = cmd_vel_pub
        self.tf_buffer = tf_buffer
        #Frame from the robot
        self.robot_frame = robot_frame
        #Velicuty of the robot
        self.vel_max = vel_max

        self.msg = geometry_msgs.msg.Twist()

        return True

    def initialise(self):
        pass

    def update(self):
        self.wpf.header.stamp = rospy.Time(0)
        #The transformation from the robot to the next waypoint
        wpf = self.tf_buffer.transform(self.wpf,self.robot_frame,timeout=rospy.Duration(10.0))
        #The distance from the robot to the next waypoiny
        error = np.sqrt(((wpf.point.x)**2)+((wpf.point.y)**2))
        #While we are not inside the tolerance we go backward
        if error > 0.05:
            self.msg.angular.z = 0.0
            self.msg.linear.x = -1*vel_max
            self.cmd_vel_pub.publish(self.msg)
            return py_trees.Status.RUNNING
        #If we are inside the tolerance we go forward again
        else:
            self.msg.angular.z = 0.0
            self.msg.linear.x = vel_max
            self.cmd_vel_pub.publish(self.msg)
            return py_trees.Status.SUCCESS


    def terminate(self, new_status):
        pass

def create_behavior_tree(waypoints, frame, look_ahead_dist, vel_max,
                K_theta, alpha):
    """
    Constructs and returns the behavior tree.
    The tree has only one level with a sequence of nodes, all of which
    must succeed in order to perform the task.
    """
    # Setup stuff for ROS communication
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    cmd_vel_pub = rospy.Publisher('/cmd_vel' , geometry_msgs.msg.Twist, queue_size=1 )
    #We start the root of the tree with a Selector
    root = py_trees.composites.Selector("Root")
    #We create a root of Sequence to insert the Pure Pursuit points, fallbacks and Go2Point
    go_points = py_trees.composites.Sequence("PurePursuit")

    #We create the CarAtGoal node
    atGoal = CarAtGoal()
    #We insert the last waypoint
    atGoal.setup(waypoints[-1],tf_buffer,cmd_vel_pub)
    #We add the childrens
    root.add_children([atGoal,go_points])

    # Setup and add node handling the first waypoint. We should go towards this point
    # until it is at a distance within the look-ahead-distance from the
    # robot.
    g2p = Go2Point()
    g2p.setup(waypoints[0], look_ahead_dist, cmd_vel_pub, tf_buffer,
              vel_max=vel_max, K_theta=K_theta, alpha=alpha)

    go_points.add_child(g2p)

    # Add the nodes to handle pairs of waypoints
    contador = 1
    for cwp_, nwp_ in zip(waypoints[0:-2], waypoints[1:-1]):
        #We create a selector to insert the PurePursuit node and Fallback node
        nextPoint = py_trees.composites.Selector("Next_"+str(contador))
        pg = PursuitGoal()
        pg.setup(cwp_, nwp_, look_ahead_dist, cmd_vel_pub,
                 tf_buffer, vel_max=vel_max)
        fail = Fallback()
        fail.setup(nwp_,cmd_vel_pub, tf_buffer,vel_max=vel_max)
        #We insert selector
        go_points.add_child(nextPoint)
        #We insert the nodes Fallback and PurePursuit
        nextPoint.add_children([pg,fail])
        contador += 1


    #----------------------------------------------------------------------------
    # YOUR CODE HERE
    # Add the final node to go to the last waypoint, which is the final goal point
    g2p = Go2Point()
    g2p.setup(waypoints[-1], look_ahead_dist, cmd_vel_pub, tf_buffer,
              vel_max=vel_max, K_theta=K_theta, alpha=alpha)
    go_points.add_child(g2p)
    #
    #----------------------------------------------------------------------------

    return root


if __name__ == '__main__':
    #We read the waypoints file
    rospy.init_node('Pure_pursuit')
    L = rospy.get_param("/pure_pursuit/look_ahead_distance")
    vmax = rospy.get_param("/pure_pursuit/vel_lin_max")
    Kth = rospy.get_param("/pure_pursuit/K_theta")
    alpha = rospy.get_param("/pure_pursuit/alpha")
    frame = rospy.get_param("/pure_pursuit/frame")
    rate = rospy.get_param("/pure_pursuit/rate")
    waypoints = rospy.get_param("/waypoints")
    waypoints = np.reshape(waypoints, (-1, 2))
    waypoints_ps = []
    #We add the waypoints inside a list
    for wp_ in waypoints:
        p = PointStamped()
        p.header.frame_id = frame
        p.point.x = wp_[0]
        p.point.y = wp_[1]
        waypoints_ps.append(p)
    #We create the behaviour tree
    bt_tree = create_behavior_tree(waypoints_ps, frame, L, vmax,
                Kth, alpha)

    rosrate = rospy.Rate(rate)

    while not rospy.is_shutdown():
        bt_tree.tick_once()
        rosrate.sleep()
