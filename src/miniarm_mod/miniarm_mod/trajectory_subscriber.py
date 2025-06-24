import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory

class MoveItTrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_subscriber')
        
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            100
        )
        self.publisher = self.create_publisher(JointTrajectory, '/Plan1_controller/joint_trajectory', 100)
        self.get_logger().info('⏳ starting trajectory subscriber, waiting for trajectories...')

    def trajectory_callback(self, msg):
        """ handle incoming trajectory message """
        if not msg.trajectory:
            self.get_logger().warn('⚠️ received empty trajectory message, ignoring...')
            return

        trajectory = msg.trajectory[0]
        self.publisher.publish(trajectory.joint_trajectory)
        joint_names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        
        self.get_logger().info(f'🎯 received new trajectory: {len(points)} points')
        self.get_logger().info(f'🔄 joints: {joint_names}')

        # 打印第一个和最后一个路径点信息
        if points:
            first_point = points[0]
            last_point = points[-1]
            self.get_logger().info('--- Trajectory information: ---')
            self.get_logger().info(f'🕒 start time: {first_point.time_from_start.sec} ')
            self.get_logger().info(f'📍 dest positions: {[round(x, 3) for x in last_point.positions]}')
            self.get_logger().info(f'⚡ dest velocities: {[round(x, 3) for x in last_point.velocities]}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = MoveItTrajectorySubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.get_logger().info('⏹ closing trajectory subscriber, cleaning up resources')
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()