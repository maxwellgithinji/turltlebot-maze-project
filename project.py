
from robot_control_class import RobotControl
import rospy

class Maze(RobotControl):
    def __init__(self, route=1):
        super().__init__()
        self.route = route
        self.front_laser = self.get_front_laser()
        self.front_left_laser = self.get_laser(200)
        self.front_right_laser = self.get_laser(440)
        

    def navigate(self):
        # go straight until the the fornt laser detects a barrier
        while self.can_move_staright():
            self.set_front_lasers() #update front laser information each loop (takes ~1sec)
            self.move_straight()
        
        self.stop_robot()
        self.set_front_lasers()
        rospy.logwarn("Stopping robot, barrier %fm away", self.front_laser)
        print(self.front_left_laser, self.front_laser, self.front_right_laser)

        # try to turn the robot after stopping
        self.make_turn()

        self.navigate()

    def set_front_lasers(self):
        full_laser = self.get_laser_full()
        self.front_laser =  full_laser[360]
        self.front_left_laser = full_laser[200]
        self.front_right_laser = full_laser[440]
    
    def can_move_staright(self):
        front_barrier_proximity = 1
        return self.front_laser > front_barrier_proximity and self.front_left_laser > front_barrier_proximity and self.front_right_laser > front_barrier_proximity

    def make_turn(self):
        if self.should_turn_left():
            rospy.loginfo("turning left, front left laser proximity, %fm", self.front_left_laser)
            initial_reading = self.front_left_laser
            while not self.left_oversteer(initial_reading):
                self.rotate(-5)
                self.set_front_lasers()
        elif self.should_turn_right():
            rospy.loginfo("turning right, front right laser proximity %fm", self.front_right_laser)
            initial_reading = self.front_right_laser 
            while not self.right_oversteer(initial_reading):
                self.rotate(5)
                self.set_front_lasers()

    def should_turn_left(self):
        return self.front_right_laser < 1
    def should_turn_right(self):
        return self.front_left_laser < 1
    
    def left_oversteer(self, initial_reading):
        rospy.logwarn("left oversteer, front left proximity %fm away",self.front_left_laser)
        return self.front_left_laser < initial_reading
        
    def right_oversteer(self, initial_reading):
        rospy.logwarn("right oversteer, front right proximiyt %fm away",self.front_right_laser)
        return self.front_right_laser < initial_reading
    
m = Maze()

m.navigate()
