
from cmath import pi
from robot_control_class import RobotControl
import rospy
# Create a protective field
# - focuses on front lasers when moving forward
# focuses on side lasers when turning
class Maze(RobotControl):
    def __init__(self, route=1):
        super().__init__()
        self.route = route
        self.front_laser = self.get_front_laser()
        self.front_left_laser = self.get_laser(200)
        self.front_right_laser = self.get_laser(440)
        self.BARRIER_PROXIMITY = 1.0 
        self.SCAN_RANGE = 30
        

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

        # recursively navigate until a barrier is encountered
        self.navigate()

    def set_front_lasers(self):
        full_laser = self.get_laser_full()
        self.front_laser =  min(full_laser[360-15:360+15])
        self.front_left_laser = min(full_laser[200-self.SCAN_RANGE:200+self.SCAN_RANGE])
        self.front_right_laser = min(full_laser[440-self.SCAN_RANGE:440+self.SCAN_RANGE])
    
    def can_move_staright(self):
        front_barrier_proximity = 1
        return self.front_laser > front_barrier_proximity and self.front_left_laser > front_barrier_proximity and self.front_right_laser > front_barrier_proximity

    def make_turn(self):
        turn_direction = self.set_turn_direction()
        circle_radians = 2*pi
        max_rotation = 0
        angle = 5

        # avaoid infinite turns
        while max_rotation < circle_radians:
            self.rotate(angle * turn_direction)
            self.set_front_lasers()

            if self.can_move_staright():
                rospy.loginfo("found clear path")
                return

            max_rotation += abs(angle * pi/180) #converts the turn angle to radians
        
        rospy.logwarn("Could not find clear path after full rotation")
        

    def set_turn_direction(self):
        if self.front_left_laser > self.front_right_laser:
             return -1 #left
        else:
            return 1 #right
    
m = Maze()

m.navigate()
