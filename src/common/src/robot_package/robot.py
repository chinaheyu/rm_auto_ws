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
    def __init__(self):
        super(ROSBase, self).__init__()
        rospy.Subscriber('/sign', Bool, self.__sign_callback, queue_size=10)
        self.__can_control = False

    def echo(self, message):
        rospy.loginfo(message)

    def __sign_callback(self, msg):
        self.__can_control = msg.data
    
    @property
    def can_control(self):
        return self.__can_control

    @property
    def is_shutdown(self):
        return rospy.is_shutdown()


class ActionBase(object):
    def __init__(self):
        super(ActionBase, self).__init__()
        self.__act_pub = rospy.Publisher('/cmd_action', cmd_action, queue_size=10)
        self.__grasp_state = False
        self.__push_state = False
        self.__flip_state = False
        self.__raise_body_state = False
        self.__raise_camera_state = False
        self.__act_msg = cmd_action()

    def __publish_act_msg(self):
        self.__act_msg.body_lift = self.__raise_body_state
        self.__act_msg.grasp = self.__grasp_state
        self.__act_msg.push = self.__push_state
        self.__act_msg.camera_lift = self.__raise_camera_state
        self.__act_msg.flip = self.__flip_state
        self.__act_pub.publish(self.__act_msg)

    def grasp(self, on):
        self.__grasp_state = on
        self.__publish_act_msg()

    def grasp(self):
        self.__grasp_state = not self.__grasp_state
        self.__publish_act_msg()

    def push(self, on):
        self.__push_state = on
        self.__publish_act_msg()

    def push(self):
        self.__push_state = not self.__push_state
        self.__publish_act_msg()

    def flip(self, on):
        self.__flip_state = on
        self.__publish_act_msg()

    def flip(self):
        self.__flip_state = not self.__flip_state
        self.__publish_act_msg()

    def raise_body(self, on):
        self.__raise_body_state = on
        self.__publish_act_msg()

    def raise_body(self):
        self.__raise_body_state = not self.__raise_body_state
        self.__publish_act_msg()

    def raise_camera(self, on):
        self.__raise_camera_state = on
        self.__publish_act_msg()

    def raise_camera(self):
        self.__raise_camera_state = not self.__raise_camera_state
        self.__publish_act_msg()


class BeltBase(object):
    def __init__(self):
        super(BeltBase, self).__init__()
        self.__belt_position = 0
        self.__belt_low_boundary = 0
        self.__belt_high_boundary = 450
        self.__belt_pub = rospy.Publisher('/cmd_belt', cmd_belt, queue_size=10)
        self.__belt_msg = cmd_belt()

    def move_belt(self, position):
        self.__belt_position = min(max(position, self.__belt_low_boundary), self.__belt_high_boundary)
        self.__belt_msg.belt = self.__belt_position
        self.__belt_pub.publish(self.__belt_msg)

    @property
    def belt_position(self):
        return self.__belt_position

    @belt_position.setter
    def belt_position(self, value):
        self.move(value)


class MoveBase(object):
    def __init__(self):
        super(MoveBase, self).__init__()
        self.__vel_pub = rospy.Publisher("/cmd_vel", cmd_vel, tcp_nodelay=True, queue_size=10)
        rospy.Subscriber("/odometry", Odometry, self.__odom_callback, queue_size=10, tcp_nodelay=True)
        self.__vel_msg = cmd_vel()
        self.__max_line_speed = 4.0
        self.__max_rotate_speed = 20.0
        self.__velocity = {'vx': 0.0, 'vy': 0.0, 'vw': 0.0}
        self.__odometry = {'x': 0.0, 'y': 0.0, 'w': 0.0, 'vx': 0.0, 'vy': 0.0, 'vw': 0.0}

    def __odom_callback(self, data):
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
    def odometry(self):
        return self.__odometry

    @property
    def velocity(self):
        return self.__velocity

    def __publish_vel_msg(self):
        self.__vel_msg.vx = self.__velocity['vx']
        self.__vel_msg.vy = self.__velocity['vy']
        self.__vel_msg.vw = self.__velocity['vw']
        self.__vel_pub.publish(self.__vel_msg)

    def set_velocity(self, vx, vy, vw):
        self.__velocity['vx'] = max(min(vx, self.__max_line_speed), -self.__max_line_speed)
        self.__velocity['vy'] = max(min(vy, self.__max_line_speed), -self.__max_line_speed)
        self.__velocity['vw'] = max(min(vw, self.__max_rotate_speed), -self.__max_rotate_speed)
        self.__publish_vel_msg()

    def slide(self, dx, dy):
        target_x = self.__odometry['x'] + dx
        target_y = self.__odometry['y'] + dy
        vx = 0.0
        vy = 0.0
        while True:

            error = target_x - self.__odometry['x']
            if abs(error) > 0.01:
                vx = 2.0 * error
            else: vx = 0.0

            error = target_y - self.__odometry['y']
            if abs(error) > 0.01:
                vy = 2.0 * error
            else: vy = 0.0

            if vx == 0 and vy == 0:
                break
            self.set_velocity(vx, vy, 0)
        self.stop()

    def rotate(self, degree):
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
    
    def stop(self):
        self.set_velocity(0, 0, 0)


class AutoAlignBase(object):
    def __init__(self):
        super(AutoAlignBase, self).__init__()
        self.__align_state = False
        self.__align_pub = rospy.Publisher('/cmd_align', cmd_align, queue_size=2)
        self.__align_msg = cmd_align()
        rospy.Subscriber('/align_state', Bool, callback=self.__alignCallback)

    def __alignCallback(self, msg):
        self.__align_state = msg.data

    def alignToOre(self, on=True):
        self.__align_state = True
        self.__align_msg.do_align = on
        self.__align_pub.publish(self.__align_msg)

    def waitForAlign(self):
        while self.__align_state:
            time.sleep(0.1)

    @property
    def is_align(self):
        return self.__align_state


class Robot(ROSBase, ActionBase, BeltBase, MoveBase, AutoAlignBase):
    def __init__(self, node_name):
        rospy.init_node(node_name)
        super(Robot, self).__init__()
