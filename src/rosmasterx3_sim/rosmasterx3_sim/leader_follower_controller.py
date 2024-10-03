import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from typing import Optional
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class LeaderFollowerController(Node):

    def __init__(self):

        super().__init__('leader_follower_controller')

        # Controller parameters
        self.declare_parameter("robot1_initial_pose", [0.0,0.0,0.0])
        self.declare_parameter("robot2_initial_pose", [0.0,0.0,0.0])
        self.declare_parameter("d_goal", 0.1)
        self.declare_parameter('alpha_goal', 0.0)
        self.declare_parameter('theta_f_goal', 0.0)
        self.declare_parameter("K", [1.0,1.0,1.0])

        # Controller gains
        self.d_goal = self.get_parameter('d_goal').get_parameter_value().double_value
        self.alpha_goal = self.get_parameter('alpha_goal').get_parameter_value().double_value
        self.alpha_goal = ((self.alpha_goal + np.pi)%(2*np.pi)) - np.pi
        self.theta_f_goal = self.get_parameter('theta_f_goal').get_parameter_value().double_value
        self.theta_f_goal = ((self.theta_f_goal + np.pi)%(2*np.pi)) - np.pi
        self.K = np.array(self.get_parameter('K').get_parameter_value().double_array_value)

        self.q1 = np.array(self.get_parameter('robot1_initial_pose').get_parameter_value().double_array_value)
        self.q2 = np.array(self.get_parameter('robot2_initial_pose').get_parameter_value().double_array_value)
        self.u_l = np.array([[0,0,0]]).T
        self.u_f = np.array([0,0,0])

        # Publishers, subscribers and actions
        self._robot1_pose_subscription = self.create_subscription(Pose, 'robot1/pose', self.__robot1_pose_callback, 1)
        self._robot2_pose_subscription = self.create_subscription(Pose, 'robot2/pose', self.__robot2_pose_callback, 1)
        self._robot1_cmd_vel_subscription = self.create_subscription(Twist, 'robot1/cmd_vel', self.__robot1_cmd_vel_callback, 1)
        self._robot2_cmd_vel_publisher: Optional[Publisher] = None

        self.create_timer(0.01, self.__control_callback)

    def __robot1_pose_callback(self, msg):

        if msg.orientation.x == 0 and msg.orientation.y == 0 and msg.orientation.z == 0 and msg.orientation.w == 0:
            theta = self.q1[2]
        else:
            r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            theta = r.as_rotvec()[-1]

            self.q1 = np.array([msg.position.x, msg.position.y, theta])


    def __robot2_pose_callback(self, msg):

        if msg.orientation.x == 0 and msg.orientation.y == 0 and msg.orientation.z == 0 and msg.orientation.w == 0:
            theta = self.q2[2]
        else:
            r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            theta = r.as_rotvec()[-1]

            self.q2 = np.array([msg.position.x, msg.position.y, theta])


    def __robot1_cmd_vel_callback(self, msg):

        v_x = msg.linear.x
        v_y = msg.linear.y
        w_z = msg.angular.z

        self.u_l = np.array([[v_x,v_y,w_z]]).T

    def __control_callback(self):

        d = np.sqrt((self.q1[0] - self.q2[0])**2 + (self.q1[1] - self.q2[1])**2)
        alpha = self.q2[2] - np.arctan2((self.q1[1] - self.q2[1]), (self.q1[0] - self.q2[0]))
        alpha = ((alpha + np.pi)%(2*np.pi)) - np.pi ## NEW: Normalize alpha angle
        gamma = self.q1[2] - self.q2[2] + alpha
        gamma = ((gamma + np.pi)%(2*np.pi)) - np.pi ## NEW: Normalize gamma angle

        A = np.array([[np.cos(gamma),-np.sin(gamma),0],
                      [-(1/d)*np.sin(gamma),-(1/d)*np.cos(gamma),0],
                      [0,0,0]])
        B = np.array([[np.cos(alpha),-np.sin(alpha),0],
                      [-(1/d)*np.sin(alpha),-(1/d)*np.cos(alpha),-1],
                      [0,0,-1]]) 

        ## Caging
        alpha_error = alpha - self.alpha_goal ## NEW: Shortest rotation alpha angle
        if alpha_error > np.pi:
            alpha_error += -2*np.pi
        elif alpha_error < -np.pi:
            alpha_error += 2*np.pi

        theta_error = self.q2[2] - self.theta_f_goal ## NEW: Shortest rotation theta angle
        if theta_error > np.pi:
            theta_error += -2*np.pi
        elif theta_error < -np.pi:
            theta_error += 2*np.pi
        p_d = np.array([[-self.K[0]*(d - self.d_goal)],
                        [-self.K[1]*(alpha_error)],
                        [-self.K[2]*(theta_error)]]) 
    

        self.u_f = np.dot(np.linalg.inv(B), (np.dot(A, self.u_l) - p_d))
        u_f = self.u_f.squeeze()

        twist_msg = Twist()

        twist_msg.linear.x = float(u_f[0])
        twist_msg.linear.y = float(u_f[1])
        twist_msg.angular.z = float(u_f[2])

        if self._robot2_cmd_vel_publisher is not None:
            self._robot2_cmd_vel_publisher.publish(twist_msg)

    ##########################
    ## LifeCycle Callabacks ##
    ##########################

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.
        
        :return: The state machine either invokes a transition to the "inactive" state or stays
        in "unconfigured" depending on the return value.
        TransitionCallbackReturn.SUCCESS transitions to "inactive".
        TransitionCallbackReturn.FAILURE transitions to "unconfigured".
        TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self._robot2_cmd_vel_publisher = self.create_lifecycle_publisher(Twist, 'robot2/cmd_vel', 10)

        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the inactive and
        # enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks, and we don't
        # need to write on_activate()/on_deactivate() callbacks.

        # Log, only for demo purposes
        self.get_logger().info("on_activate() is called.")

        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        return super().on_activate(state)
  
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info("on_deactivate() is called.")
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested.

        on_cleanup callback is being called when the lifecycle node
        enters the "cleaning up" state.
        
        :return: The state machine either invokes a transition to the "unconfigured" state or stays
        in "inactive" depending on the return value.
        TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
        TransitionCallbackReturn.FAILURE transitions to "inactive".
        TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_publisher(self._robot2_cmd_vel_publisher)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.
        
        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
        TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
        TransitionCallbackReturn.FAILURE transitions to "inactive".
        TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_publisher(self._robot2_cmd_vel_publisher)

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS

def main(args=None):

    rclpy.init(args=args)

    leader_follower_controller_node = LeaderFollowerController()

    rclpy.spin(leader_follower_controller_node)

    leader_follower_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()