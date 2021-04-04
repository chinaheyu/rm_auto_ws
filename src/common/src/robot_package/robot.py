import time
import math
import rospy
from common.msg import cmd_vel
from common.msg import cmd_action
from common.msg import cmd_belt
from common.msg import cmd_align
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class ROSBase(object):
    def __init__(self) -> None:
        super().__init__()
        rospy.Subscriber('/sign', Bool, self.__sign_callback, queue_size=10)
        self.__can_control = False

    def echo(self, message: str) -> None:
        rospy.loginfo(message)

    def __sign_callback(self, msg) -> None:
        self.__can_control = msg.data
    
    @property
    def can_control(self) -> bool:
        return self.__can_control

    @property
    def is_shutdown(self) -> bool:
        return rospy.is_shutdown()


class ActionBase(object):
    def __init__(self) -> None:
        super().__init__()
        self.__act_pub = rospy.Publisher('/cmd_action', cmd_action, queue_size=10)
        self.__grasp_state = False
        self.__push_state = False
        self.__flip_state = False
        self.__raise_body_state = False
        self.__raise_camera_state = False
        self.__act_msg = cmd_action()

    def __publish_act_msg(self) -> None:
        self.__act_msg.body_lift = self.__raise_body_state
        self.__act_msg.grasp = self.__grasp_state
        self.__act_msg.push = self.__push_state
        self.__act_msg.camera_lift = self.__raise_camera_state
        self.__act_msg.flip = self.__flip_state
        self.__act_pub.publish(self.__act_msg)

    def grasp(self, on: bool) -> None:
        self.__grasp_state = on
        self.__publish_act_msg()

    def grasp(self) -> None:
        self.__grasp_state = not self.__grasp_state
        self.__publish_act_msg()

    def push(self, on: bool) -> None:
        self.__push_state = on
        self.__publish_act_msg()

    def push(self) -> None:
        self.__push_state = not self.__push_state
        self.__publish_act_msg()

    def flip(self, on: bool) -> None:
        self.__flip_state = on
        self.__publish_act_msg()

    def flip(self) -> None:
        self.__flip_state = not self.__flip_state
        self.__publish_act_msg()

    def raise_body(self, on: bool) -> None:
        self.__raise_body_state = on
        self.__publish_act_msg()

    def raise_body(self) -> None:
        self.__raise_body_state = not self.__raise_body_state
        self.__publish_act_msg()

    def raise_camera(self, on: bool) -> None:
        self.__raise_camera_state = on
        self.__publish_act_msg()

    def raise_camera(self) -> None:
        self.__raise_camera_state = not self.__raise_camera_state
        self.__publish_act_msg()


class BeltBase(object):
    def __init__(self) -> None:
        super().__init__()
        self.__belt_position = 0
        self.__belt_low_boundary = 0
        self.__belt_high_boundary = 450
        self.__belt_pub = rospy.Publisher('/cmd_belt', cmd_belt, queue_size=10)
        self.__belt_msg = cmd_belt()

    def move_belt(self, position: int) -> None:
        self.__belt_position = min(max(position, self.__belt_low_boundary), self.__belt_high_boundary)
        self.__belt_msg.belt = self.__belt_position
        self.__belt_pub.publish(self.__belt_msg)

    @property
    def belt_position(self) -> int:
        return self.__belt_position

    @belt_position.setter
    def belt_position(self, value: int) -> None:
        self.move(value)


class MoveBase(object):
    def __init__(self) -> None:
        super().__init__()
        self.__vel_pub = rospy.Publisher("/cmd_vel", cmd_vel, tcp_nodelay=True, queue_size=10)
        rospy.Subscriber("/odometry", Odometry, self.__odom_callback, queue_size=10, tcp_nodelay=True)
        self.__vel_msg = cmd_vel()
        self.__max_line_speed = 4.0
        self.__max_rotate_speed = 20.0
        self.__velocity = {'vx': 0.0, 'vy': 0.0, 'vw': 0.0}
        self.__odometry = {'x': 0.0, 'y': 0.0, 'w': 0.0, 'vx': 0.0, 'vy': 0.0, 'vw': 0.0}

    def __odom_callback(self, data) -> None:
        self.__odometry['x'] = data.pose.pose.position.x
        self.__odometry['y'] = data.pose.pose.position.y
        self.__odometry['w'] = euler_from_quaternion((
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ))[2]
        self.__odometry['vx'] = data.twist.twist.linear.x
        self.__odometry['vy'] = data.twist.twist.linear.y
        self.__odometry['vw'] = data.twist.twist.angular.z

    @property
    def odometry(self) -> dict:
        return self.__odometry

    @property
    def velocity(self) -> dict:
        return self.__velocity

    def __publish_vel_msg(self):
        self.__vel_msg.vx = self.__velocity['vx']
        self.__vel_msg.vy = self.__velocity['vy']
        self.__vel_msg.vw = self.__velocity['vw']
        self.__vel_pub.publish(self.__vel_msg)

    def set_velocity(self, vx: float, vy: float, vw: float):
        self.__velocity['vx'] = max(min(vx, self.__max_line_speed), -self.__max_line_speed)
        self.__velocity['vy'] = max(min(vy, self.__max_line_speed), -self.__max_line_speed)
        self.__velocity['vw'] = max(min(vw, self.__max_rotate_speed), -self.__max_rotate_speed)
        self.__publish_vel_msg()

    def slide(self, dx: float, dy: float) -> None:
        target_x = self.__odometry['x'] + dx
        target_y = self.__odometry['y'] + dy
        vx = 0.0
        vy = 0.0
        while True:
            if abs(error := target_x - self.__odometry['x']) > 0.01:
                vx = 2.0 * error
            else: vx = 0.0
            if abs(error := target_y - self.__odometry['y']) > 0.01:
                vy = 2.0 * error
            else: vy = 0.0
            if vx == 0 and vy == 0:
                break
            self.set_velocity(vx, vy, 0)
        self.stop()

    def rotate(self, degree: float) -> None:
        begin_w = self.__odometry['w']
        target = begin_w + degree / 180.0 * math.pi
        while target > math.pi:
            target -= 2 * math.pi
        while target <= -math.pi:
            target += 2 * math.pi
        while True:
            error = target - self.__odometry['w']
            if error <= -math.pi:
                error += 2 * math.pi
            if error > math.pi:
                error -= 2 * math.pi
            if abs(error) < 0.01:
                break
            self.set_velocity(0, 0, 5.0 * error)
        self.stop()
    
    def stop(self) -> None:
        self.set_velocity(0, 0, 0)


class AutoAlignBase(object):
    def __init__(self) -> None:
        super().__init__()
        self.__align_state = False
        self.__align_pub = rospy.Publisher('/cmd_align', cmd_align, queue_size=2)
        self.__align_msg = cmd_align()
        rospy.Subscriber('/align_state', Bool, callback=self.__alignCallback)

    def __alignCallback(self, msg) -> None:
        self.__align_state = msg.data

    def alignToOre(self, on=True) -> None:
        self.__align_state = True
        self.__align_msg.do_align = on
        self.__align_pub.publish(self.__align_msg)

    def waitForAlign(self) -> None:
        while self.__align_state:
            time.sleep(0.1)

    @property
    def is_align(self) -> bool:
        return self.__align_state


class Robot(ROSBase, ActionBase, BeltBase, MoveBase, AutoAlignBase):
    def __init__(self, node_name: str):
        rospy.init_node(node_name)
        super().__init__()
