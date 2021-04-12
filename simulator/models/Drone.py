from models.CostomThread import ExitableThread
from time import sleep
from simple_pid import PID
from models.Sensors import Wifi
import math

class Drone:
    Kp = 0.04      # quao longe estamos do destino
    Ki = 0.0001     # aproximar do valor verdaeiro
    Kd = 0.004      # compensar o overshoot do p

    DRONE_SAMPLE_TIME = 0.008
    ERROR_THRESHOLD = 10
    MAX_RADIUS_CONNECTION = 50
    RADIUS_CONNECTION_THRESHOLD = 30
    OPTIMAL_DISTANCE_CONNECTION = 20

    def __init__(self, coords, drone_type=None):
        self.initial_coords = coords
        self.x, self.y, self.z = (coords[0], coords[1], coords[2] if len(coords)>2 else 5)
        self.state = "OFFLINE"
        self.connection = None
        self.task = None
        self.type = drone_type
        self.pid_x, self.pid_y, self.pid_z = (PID(self.Kp, self.Ki, self.Kd), PID(self.Kp, self.Ki, self.Kd), PID(self.Kp, self.Ki, self.Kd))
        self.pid_x.sample_time = self.pid_y.sample_time = self.pid_z.sample_time = self.DRONE_SAMPLE_TIME
        self.is_access_point = False
    
    @property
    def coords(self):
        return (self.x, self.y, self.z)

    def addTask(self, func, args=None):
        if func == "moveTo":
            self.state = "ARMED"
            self.startTask(self.moveTo, args)
        elif func == 'returnBase':
            self.startTask(self.moveTo, self.initial_coords)
            self.state == "OFFLINE"

    def startTask(self, func, args):
        if self.task != None:
            self.task.exit()
            sleep(0.05)
        self.task = ExitableThread(target=func, args=(args[0], args[1], args[2] if len(args)>2 else self.z), daemon=True)
        self.task.start()

    def moveTo(self, x, y, z):
        self.pid_x.setpoint, self.pid_y.setpoint, self.pid_z.setpoint = (x,y,z)
        while(not self.task.exited()):
            len_x, len_y, len_z = (self.pid_x(self.x), self.pid_y(self.y), self.pid_z(self.z))
            size = math.sqrt(len_x**2 + len_y**2 + len_z**2)
            if size == 0:
                self.x, self.y, self.z = (x, y, z)
                self.task = None
                break
            self.x += len_x/size
            self.y += len_y/size
            self.z += len_z/size
            variation = math.sqrt((self.x-x)**2 + (self.y-y)**2 + (self.z-z)**2)
            if variation < self.ERROR_THRESHOLD:
                self.x, self.y, self.z = (x, y, z)
                self.task = None
                break
            sleep(self.DRONE_SAMPLE_TIME)
        
    def is_free(self):
        return self.task == None
    
    def wait_until_free(self):
        sleep(0.1)
        while self.task != None:
            sleep(0.1)
    
    def connectedTo(self, drone, mbs=None):
        self.connection = Wifi(drone, mbs)

    def draw(self, pygame, screen):
        if self.coords != self.initial_coords or self.is_access_point:
            if self.is_access_point: color = (0, 255, 0)
            else: color = (255, 255, 0)
            pygame.draw.circle(screen, color, (self.x, self.y), self.MAX_RADIUS_CONNECTION, width=1)
            if self.type == "relay": pygame.draw.circle(screen, (0, 150, 150), (self.x, self.y), (self.z+10)/5)
            else: pygame.draw.circle(screen, (200, 200, 200), (self.x, self.y), (self.z+10)/5)

    def distance(self, drone):
        return math.sqrt((self.x-drone.x)**2 + (self.y-drone.y)**2 + (self.z-drone.z)**2)

    def __str__(self):
        return "Drone: " + str(self.coords)