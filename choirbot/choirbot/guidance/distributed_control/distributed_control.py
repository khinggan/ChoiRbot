from geometry_msgs.msg import Vector3
from ..guidance import Guidance
import numpy as np
from time import sleep


class DistributedControlGuidance(Guidance):

    def __init__(self, update_frequency: float, pos_handler: str=None, pos_topic: str=None, input_topic: str = 'velocity'):
        super().__init__(pos_handler, pos_topic)
        self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.control)
        self.get_logger().info('Guidance {} started'.format(self.agent_id))

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(self.current_pose, self.in_neighbors, self.out_neighbors, False)

        # data: (agent 1's data)
        # {0: Pose(position=array([-2.24059137,  2.21454979,  0.00847838]), 
        #          orientation=array([ 1.17066147e-05,  1.96970411e-02, -1.72593029e-01,  9.84796259e-01]), 
        #          velocity=array([-1.47814542e-01,  9.91213875e-05,  0.00000000e+00]), 
        #          angular=array([ 0.       ,  0.       , -0.7126075])), 
        #  2: Pose(position=array([2.27493217, 1.53698393, 0.00900787]), 
        #          orientation=array([-3.20060384e-07, -1.71081119e-02,  1.13547882e-03,  9.99853001e-01]), 
        #          velocity=array([ 3.45902316e-01, -5.80375666e-05,  0.00000000e+00]), 
        #          angular=array([0.        , 0.        , 0.01666945])), 
        #  4: Pose(position=array([ 0.38370569, -3.33001209,  0.00864761]), 
        #          orientation=array([ 0.00420296, -0.00464518,  0.41310229,  0.91066309]), 
        #          velocity=array([ 0.3899216 , -0.00054371,  0.        ]), 
        #          angular=array([0.        , 0.        , 0.51398642]))
        # }
        # Cannot get all data at ones, so create another subscriber that subscribe all /odom and get data

        # compute input
        u = self.evaluate_input(data)

        # send input to planner/controller
        self.send_input(u)

    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.publisher_.publish(msg)

    def evaluate_input(self, neigh_data):
        raise NotImplementedError

