import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import PoseStamped , Twist
import math
import scipy.interpolate as si
from rclpy.qos import QoSProfile

lookahead_distance = 0.15
v = 0.1
expansion_size = 2 #for the wall

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()

    came_from = {}

    gscore = {start:0}

    fscore = {start:heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))


    while oheap:

        current = heapq.heappop(oheap)[1]

        if current == goal:

            data = []

            while current in came_from:

                data.append(current)

                current = came_from[current]

            return data

        close_set.add(current)

        for i, j in neighbors:

            neighbor = current[0] + i, current[1] + j

            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:

                if 0 <= neighbor[1] < array.shape[1]:                

                    if array[neighbor[0]][neighbor[1]] == 1:

                        continue

                else:

                    # array bound y walls

                    continue

            else:

                # array bound x walls

                continue


            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):

                continue


            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:

                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))


    return False

def bspline_planning(x, y, sn):
    N = 2
    t = range(len(x))
    x_tup = si.splrep(t, x, k=N)
    y_tup = si.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)

    return rx, ry

def pure_pursuit(current_x, current_y, current_heading, path,lookahead_distance,index):
    closest_point = None
    v = 0.1
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index


class navigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.listener_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.info_callback,10)
        self.subscription = self.create_subscription(PoseStamped,'goal_pose',self.goal_pose_callback,QoSProfile(depth=10)) #rviz2 goal_pose Mesajına Abone
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10) # cmd_vel abone
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.flag = 0
        print("Hedef Bekleniyor...")

    def goal_pose_callback(self,msg):
        self.goal = (msg.pose.position.x,msg.pose.position.y)
        print("Hedef Konum: ",self.goal[0],self.goal[1])
        self.flag = 1

    def listener_callback(self,msg):
        if self.flag == 1:
            self.resolution = msg.info.resolution
            self.originX = msg.info.origin.position.x
            self.originY = msg.info.origin.position.y
            print("Robot Konum: ",self.originX,self.originY) #matrisin satır ve sütun değerleri robot icin
            column = int((self.x- msg.info.origin.position.x)/msg.info.resolution)
            row = int((self.y- msg.info.origin.position.y)/msg.info.resolution)
            columnH = int((self.goal[0]- msg.info.origin.position.x)/msg.info.resolution)
            rowH = int((self.goal[1]- msg.info.origin.position.y)/msg.info.resolution)
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data).reshape(height,width)
            wall = np.where(data == 100)
            for i in range(-expansion_size,expansion_size+1):
                for j in range(-expansion_size,expansion_size+1):
                    if i  == 0 and j == 0:
                        continue
                    x = wall[0]+i
                    y = wall[1]+j
                    x = np.clip(x,0,height-1)
                    y = np.clip(y,0,width-1)
                    data[x,y] = 100
            data = data*msg.info.resolution
            data[row][column] = 0 #Robot Anlık Konum
            data[data < 0] = 1 #-0.05 olanlar bilinmeyen yer
            data[data > 5] = 1 # 0 olanlar gidilebilir yer, 100 olanlar kesin engel
            #Elimde 0 , 1 olusan matris var. 0 olanlar gidilebilir yer, 1 olanlar engel
            #print("Robot Konum: ",row,column)
            path = astar(data,(row,column),(rowH,columnH))
            path = path + [(row,column)]
            path = path[::-1]
            path.pop(0)
            pathB = path
            pathB = [(p[1]*self.resolution+self.originX,p[0]*self.resolution+self.originY) for p in pathB]
            pathB = np.array(pathB)
            pathX = pathB[:,0]
            pathY = pathB[:,1]
            pathX,pathY = bspline_planning(pathX,pathY,len(pathX)*5)
            self.path = [(pathX[i],pathY[i]) for i in range(len(pathX))]
            print("Hedefe ilerleniyor...")
            self.i = 0
            self.flag = 2
    def timer_callback(self):
        if self.flag == 2:
            twist = Twist()
            try:
                twist.linear.x , twist.angular.z,self.i = pure_pursuit(self.x,self.y,self.yaw,self.path,lookahead_distance,self.i)
                if(abs(self.x - self.path[-1][0]) < 0.05 and abs(self.y - self.path[-1][1])< 0.05):
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.flag = 0
                    print("Hedefe Ulasildi.\n")
                    print("Yeni Hedef Bekleniyor..")
                self.publisher.publish(twist)
            except:
                pass

    def info_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)


def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
