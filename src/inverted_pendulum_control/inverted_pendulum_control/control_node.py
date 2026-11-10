import time

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from inverted_pendulum_control.model import linear_model, discretize
from inverted_pendulum_control.mpc import MPCController


class InvertedPendulumController(Node):

    def __init__(self):
        super().__init__('inverted_pendulum_controller')

        # Los parámetros se declaran solo con su tipo, sin valor por defecto,
        # para que config/mpc_params.yaml sea la única fuente de los valores y
        # no haya que mantenerlos duplicados acá. Si el nodo se lanza sin el
        # yaml, get_parameter lanza ParameterUninitializedException y el nodo
        # no arranca, en vez de usar en silencio valores fijos en el código.
        self.declare_parameter('dt', Parameter.Type.DOUBLE)
        self.declare_parameter('horizon', Parameter.Type.INTEGER)
        self.declare_parameter('Q', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('R', Parameter.Type.DOUBLE)

        dt = self.get_parameter('dt').value
        N = self.get_parameter('horizon').value
        Q = np.diag(self.get_parameter('Q').value)
        R = np.array([[self.get_parameter('R').value]])

        A, B = linear_model()
        Ad, Bd = discretize(A, B, dt)
        self.controller = MPCController(Ad, Bd, Q, R, N)

        self.state = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64,
            '/cart_force_cmd',
            10
        )

        # La frecuencia de control tiene que coincidir con el dt usado para discretizar el modelo
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info(f'MPC started: dt={dt}, horizon={N}')

    def joint_state_callback(self, msg):
        cart_index = msg.name.index('cart_rail_joint')
        pendulum_index = msg.name.index('pendulum_cart_joint')

        # Vector de estado: [x, x_dot, theta, theta_dot]
        self.state = np.array([
            [msg.position[cart_index]],
            [msg.velocity[cart_index]],
            [msg.position[pendulum_index]],
            [msg.velocity[pendulum_index]]
        ])

    def control_loop(self):
        if self.state is None:
            return

        start = time.perf_counter()
        force = self.controller.solve(self.state)
        solve_ms = (time.perf_counter() - start) * 1000.0
        self.get_logger().debug(f'MPC solve time: {solve_ms:.2f} ms')

        force_msg = Float64()
        force_msg.data = force
        self.publisher.publish(force_msg)


def main(args=None):
    rclpy.init(args=args)

    node = InvertedPendulumController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
