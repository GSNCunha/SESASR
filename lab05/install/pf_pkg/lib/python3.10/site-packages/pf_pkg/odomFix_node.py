import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

class OdomFixNode(Node):
    def __init__(self):
        super().__init__('odom_fix_node')

        # Inicialização da posição e orientação
        self.position = np.array([0.0, 0.0, 0.0])  # Posição inicial (x, y, z)
        self.orientation = 0.0  # Orientação inicial (theta, em radianos)

        # Inicialização das velocidades (com valores padrão)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Variáveis para controlar o tempo de simulação
        self.dt = 0.01  # Intervalo de tempo para cada atualização (100 Hz)

        # Assinatura do tópico de velocidades
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publicação no novo tópico /odom_fixed
        self.publisher = self.create_publisher(Odometry, '/odom_fixed', 10)

        # Timer para executar a atualização a cada dt segundos
        self.create_timer(self.dt, self.update_odometry)

    def cmd_vel_callback(self, msg):
        """
        Recebe a velocidade linear (msg.linear.x) e angular (msg.angular.z) do robô.
        """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_odometry(self):
        """
        Atualiza a posição e a orientação com base nas velocidades e publica o tópico /odom_fixed.
        """
        # Calcular a nova posição usando o modelo cinemático simples
        delta_x = self.linear_velocity * np.cos(self.orientation) * self.dt
        delta_y = self.linear_velocity * np.sin(self.orientation) * self.dt
        delta_orientation = self.angular_velocity * self.dt

        # Atualizar a posição e a orientação
        self.position[0] += delta_x
        self.position[1] += delta_y
        self.orientation += delta_orientation

        # Garantir que a orientação fique dentro do intervalo -pi a pi
        if self.orientation > np.pi:
            self.orientation -= 2 * np.pi
        elif self.orientation < -np.pi:
            self.orientation += 2 * np.pi

        # Criar a mensagem de odometria
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        # Definir a posição
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = 0.0

        # Definir a orientação usando quaternions
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(self.orientation / 2)
        msg.pose.pose.orientation.w = np.cos(self.orientation / 2)

        # Publicar o tópico de odometria ajustada
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
