import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import Odometry , OccupancyGrid
from geometry_msgs.msg import PoseStamped , Twist
from rclpy.qos import QoSProfile , qos_profile_sensor_data
import math
import matplotlib.pyplot as plt

K = 0.8 
Lfc = 2.0 
Kp = 1.0 
dt = 0.1 
L = 0.1 

def euler_from_quaternion(x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z # radyan

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

class VehicleState:

    def __init__(self,x=0.0,y=0.0,yaw=0.0,v=0.0):
        self.x = x
        self.y = y 
        self.yaw = yaw 
        self.v = v 
    
def update(state,a,delta):
    state.x = xC
    state.y = yC
    state.yaw = yaw_z
    state.v = state.v + a * dt 
    return state
    
def calc_target_index(state,cx,cy):
    dx = [state.x - icx for icx in cx] 
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx**2+idy**2)) for (idx,idy) in zip(dx,dy)]
    ind = d.index(min(d)) 
    L = 0.0
    Lf = K*state.v + Lfc
    while L < Lf and (ind+1) < len(cx):
        dx_t = cx[ind+1] - cx[ind]
        dy_t = cy[ind+1] - cx[ind]
        L += math.sqrt(dx_t**2 + dy_t ** 2)
        ind += 1
    return ind 

def PContorl(target,current):
    a = Kp * (target - current)
    return a

def pure_pursuit_control(state, cx, cy, pind):
    ind = calc_target_index(state, cx, cy)

    if pind >= ind: 
        ind = pind
    
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1] 
    alpha = math.atan2(ty-state.y,tx-state.x) - yaw_z

    if state.v < 0:
        alpha = math.pi - alpha
    Lf = (K * state.v + Lfc)
    delta = math.atan2(2.0*L*math.sin(alpha)/Lf, 1.0) 
    return delta, ind

def createM(height,width,arr):
    costmap_mat = np.ones([height,width])
    for i in range(0,height):
        for j in range(0,width):
            if(arr[(i*width)+j]==100):
                costmap_mat[i][j] = 1
                #Duvar Genisletme İslemi
                t = 2
                for k in range(2*t+1):
                    for l in range(2*t+1):
                        try:
                            costmap_mat[i+k-t][j+l-t]=1                   
                        except:
                            pass
                #Duvar Genisletme islemi
            else:
                costmap_mat[i][j] = 0
    return costmap_mat


class navigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.listener_callback,qos_profile_sensor_data) #Grid Mesajına Abone
        self.subscription = self.create_subscription(Odometry, 'odom',self.info_callback,QoSProfile(depth=10)) # Odometri Sensörüne Abone
        self.subscription = self.create_subscription(PoseStamped,'goal_pose',self.goal_pose_callback,QoSProfile(depth=10)) #rviz2 goal_pose Mesajına Abone
        self.publisher = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10)) # cmd_vel abone
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ratio = 20
        self.flag = 0
        print("Hedef Konum Bekleniyor..")

    def goal_pose_callback(self,msg):
        self.goal = (msg.pose.position.y,msg.pose.position.x)
        self.flag = 1

    def listener_callback(self, data):
        self.width = data.info.width
        self.height = data.info.height
        arr = data.data
        if(self.flag==1):
            self.start = (int((self.height/2)-(self.xA)*self.ratio),int((self.width/2)+(self.yA)*self.ratio))
            self.goal = (int((self.height/2)-self.goal[0]*self.ratio),int((self.width/2)+self.goal[1]*self.ratio))
            grid = createM(self.height,self.width,arr)
            self.route = astar(grid, self.start, self.goal)
            self.route = self.route + [self.start]
            self.route = self.route[::-1]
            self.cx = []
            self.cy = []  
            for i in range(len(self.route)):
                self.cx.append(self.route[i][0])
                self.cy.append(self.route[i][1]) 
            self.flag = 2
            
    def info_callback(self,msg):
        self.xA = msg.pose.pose.position.y
        self.yA = msg.pose.pose.position.x
        global yaw_z
        yaw_z = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w) + math.pi/2
        if(self.flag == 2):
            global xC,yC
            xC = (self.height/2)-(self.xA)*self.ratio
            yC = (self.width/2)+(self.yA)*self.ratio 
            self.state = VehicleState(x =xC, y =yC,yaw = yaw_z, v = 0.0)
            self.target_speed = 0.08
            self.target_ind = calc_target_index(self.state,self.cx,self.cy)
            self.x = [self.state.x]
            self.y = [self.state.y]
            self.yaw = [self.state.yaw]
            self.v = [self.state.v]
            self.flag = 3

        if(self.flag == 3):
            xC = (self.height/2)-(self.xA)*self.ratio
            yC = (self.width/2)+(self.yA)*self.ratio

    def timer_callback(self):
        twist = Twist()
        err = 1
        if(self.flag==3):
            ai = PContorl(self.target_speed,self.state.v)
            di, self.target_ind = pure_pursuit_control(self.state,self.cx,self.cy,self.target_ind)
            self.state = update(self.state,ai,di)
            twist.linear.x = self.state.v
            twist.angular.z = di/self.state.v
            self.publisher.publish(twist)
            self.x.append(self.state.x)
            self.y.append(self.state.y)
            self.yaw.append(self.state.yaw)
            self.v.append(self.state.v)
            plt.cla()
            plt.plot(self.cx,self.cy,".r",label="course")
            plt.plot(self.x,self.y,"-b",label="trajectory")
            plt.plot(self.cx[self.target_ind],self.cy[self.target_ind],"go",label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

        if(self.flag == 3 and xC-err < self.goal[0] and xC+err > self.goal[0] and yC-err < self.goal[1] and yC+err > self.goal[1]):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.flag=0
            print("Hedefe Ulasildi.\n")
            print("Yeni Konum Bekleniyor..")
        

def main(args=None):

  rclpy.init(args=args)
  navigation_control = navigationControl()
  rclpy.spin(navigation_control)
  navigation_control.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()