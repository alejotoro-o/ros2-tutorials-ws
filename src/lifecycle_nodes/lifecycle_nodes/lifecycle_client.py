
import rclpy
from rclpy.node import Node

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState, ChangeState

import threading
import time

class LifeCycleClient(Node):

    def __init__(self):

        super().__init__('lifecycle_client')

        self.talker1_get_state_client = self.create_client(GetState, "talker1/get_state")
        self.talker1_change_state_client = self.create_client(ChangeState, "talker1/change_state")

        self.talker2_get_state_client = self.create_client(GetState, "talker2/get_state")
        self.talker2_change_state_client = self.create_client(ChangeState, "talker2/change_state")

    def run(self):

        while not (self.talker1_get_state_client.wait_for_service() and self.talker2_get_state_client.wait_for_service() and self.talker1_change_state_client.wait_for_service() and self.talker2_change_state_client.wait_for_service()):
            self.get_logger().info("Waiting for lifecycle nodes services...")

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(1)

        self.get_logger().info("##### Configuring Talker 1 #####")
        transition = Transition.TRANSITION_CONFIGURE
        self.change_state(self.talker1_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(1)

        self.get_logger().info("##### Activating Talker 1 #####")
        transition = Transition.TRANSITION_ACTIVATE
        self.change_state(self.talker1_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(5)

        self.get_logger().info("##### Deactivating Talker 1 #####")
        transition = Transition.TRANSITION_DEACTIVATE
        self.change_state(self.talker1_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(1)

        self.get_logger().info("##### Cleaning Talker 1 #####")
        transition = Transition.TRANSITION_CLEANUP
        self.change_state(self.talker1_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(1)

        self.get_logger().info("##### Configuring Talker 2 #####")
        transition = Transition.TRANSITION_CONFIGURE
        self.change_state(self.talker2_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(1)

        self.get_logger().info("##### Activating Talker 2 #####")
        transition = Transition.TRANSITION_ACTIVATE
        self.change_state(self.talker2_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(5)

        self.get_logger().info("##### Deactivating Talker 2 #####")
        transition = Transition.TRANSITION_DEACTIVATE
        self.change_state(self.talker2_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

        time.sleep(1)

        self.get_logger().info("##### Cleaning Talker 2 #####")
        transition = Transition.TRANSITION_CLEANUP
        self.change_state(self.talker2_change_state_client, transition)

        self.get_logger().info("##### Talker 1 State #####")
        self.get_state(self.talker1_get_state_client)
        self.get_logger().info("##### Talker 2 State #####")
        self.get_state(self.talker2_get_state_client)

    ############################
    ## Manage Lifecycle Nodes ##
    ############################

    def get_state(self, client):

        get_state_request = GetState.Request()

        get_state_future = client.call_async(get_state_request)

        while not get_state_future.done():
            pass

        state = get_state_future.result().current_state.label
        self.get_logger().info(state)

    def change_state(self, client, transition):

        change_state_request = ChangeState.Request()
        change_state_request.transition.id = transition

        change_state_future = client.call_async(change_state_request)

        while not change_state_future.done():
            pass

        transition_result = change_state_future.result().success
        self.get_logger().info(str(transition_result))


def main(args=None):

    rclpy.init(args=args)

    lifecycle_client = LifeCycleClient()

    thread = threading.Thread(target=rclpy.spin, args=(lifecycle_client,))
    thread.start()
 
    lifecycle_client.run()

    lifecycle_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()