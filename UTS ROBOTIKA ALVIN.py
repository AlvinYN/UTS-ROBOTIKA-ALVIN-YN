# Import library dan package yang diperlukan
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import numpy as np
from heapq import heappush, heappop
from collections import deque

# Definisikan kelas "MazeSolver"
class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')

        # Inisialisasi variabel
        self.goal_x = 0
        self.goal_y = 0
        self.goal_reached = False
        self.robot_pos_x = 0
        self.robot_pos_y = 0
        self.robot_pos_theta = 0
        self.laser_scan = []
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(Twist, '/goal_reached', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

    # Definisikan prosedur "laser_callback" untuk membaca data dari sensor laser
    def laser_callback(self, data):
    self.laser_scan = data.ranges

# Definisikan fungsi "euclidean_distance" untuk menghitung jarak antara dua titik
def euclidean_distance(self, x1, y1, x2, y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# Definisikan fungsi "is_goal_reached" untuk mengecek apakah robot sudah mencapai tujuan
def is_goal_reached(self):
    distance = self.euclidean_distance(self.robot_pos_x, self.robot_pos_y, self.goal_x, self.goal_y)
    if distance < 0.2:
        self.goal_reached = True

# Definisikan fungsi "astar" untuk mencari jalur terpendek ke tujuan
def astar(self, start_x, start_y, goal_x, goal_y, map_data):
    queue = []
    heappush(queue, (0, start_x, start_y, []))
    visited = set()

    while queue:
        (cost, x, y, path) = heappop(queue)

        if (x, y) in visited:
            continue

        if x == goal_x and y == goal_y:
            return path + [(x, y)]

        visited.add((x, y))

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            next_x, next_y = x + dx, y + dy
            if next_x < 0 or next_y < 0 or next_x >= len(map_data) or next_y >= len(map_data[0]) or map_data[next_x][next_y] == 1:
                continue
            heappush(queue, (cost + 1 + self.euclidean_distance(next_x, next_y, goal_x, goal_y), next_x, next_y, path + [(x, y)]))

    return None

# Definisikan prosedur "move_robot" untuk menggerakkan robot ke tujuan
def move_robot(self):
    path = self.astar(int(self.robot_pos_x), int(self.robot_pos_y), self.goal_x, self.goal_y, self.map_data)
    if not path:
        return

    target_x, target_y = path[0]

    while not self.goal_reached:
        distance = self.euclidean_distance(self.robot_pos_x, self.robot_pos_y, target_x, target_y)
        if distance < 0.2:
            path = path[1:]
            if not path:
                break
            target_x, target_y = path[0]

        if target_x < self.robot_pos_x:
            vel_x = -0.2
        elif target_x > self.robot_pos_x:
            vel_x = 0.2
        else:
            vel_x = 0

        if target_y < self.robot_pos_y:
            vel_y = -0.2
        elif target_y > self.robot_pos_y:
            vel_y = 0.2
        else:
            vel_y = 0

        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.publisher.publish(twist)
        self.goal_publisher.publish(self.goal_reached)

        time.sleep(0.1)

# Definisikan prosedur "map_callback" untuk membaca data peta dari file
def map_callback(self, map_file):
    with open(map_file) as f:
        self.map_data = np.array([[int(i) for i in line.split
# Definisikan prosedur "run" untuk menjalankan simulasi dan menggerakkan robot ke tujuan
def run(self):
    # Inisialisasi ROS node dan subscriber
    rospy.init_node('simple_navigation_robot')
    rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
    rospy.Subscriber('/map_file', String, self.map_callback)

    # Inisialisasi ROS publisher
    self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.goal_publisher = rospy.Publisher('/goal_reached', Bool, queue_size=10)

    # Tunggu sampai data peta terbaca
    while not self.map_data.any():
        continue

    # Set posisi awal robot
    self.robot_pos_x = 5.0
    self.robot_pos_y = 5.0

    # Set tujuan robot
    self.goal_x = 10.0
    self.goal_y = 10.0

    # Gerakkan robot ke tujuan
    self.move_robot()

    # Berhenti saat goal tercapai
    self.publisher.publish(Twist())
    rospy.spin()


