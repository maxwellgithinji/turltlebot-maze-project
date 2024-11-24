
from robot_control_class import RobotControl
import rospy

class Maze(RobotControl):
    def __init__(self, route=1):
        super().__init__()
        self.route = route
        self.front_barrier_proximity = 1
        self.front_laser = self.get_front_laser()
        self.front_left_laser = self.get_laser(240)
        self.front_right_laser = self.get_laser(480)
        

    def navigate(self):
        # go straight until the the fornt laser detects a barrier
        while self.can_move_staright():
            self.set_front_lasers() #update front laser information each loop (takes ~1sec)
            self.move_straight()
        
        self.stop_robot()
        self.set_front_lasers()
        rospy.loginfo("Stopping robot, barrier %fm away", self.front_laser)

    def set_front_lasers(self):
        full_laser = self.get_laser_full()
        self.front_laser =  full_laser[360]
        self.front_left_laser = full_laser[240]
        self.front_right_laser = full_laser[480]
    
    def can_move_staright(self):
        return self.front_laser >self.front_barrier_proximity and self.front_left_laser > self.front_barrier_proximity and self.front_right_laser > self.front_barrier_proximity


m = Maze()

m.navigate()
