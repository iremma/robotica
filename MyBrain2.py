# ===============================================
# Seminar 12 Assignment
# Authors: Aida Munoz Monjas
#          Irene Maria Marban Alvarez  
# ===============================================

from pyrobot.brain import Brain
import math

class MyBrain2(Brain):

    # Default var. values
    threshold = 0.3
    goal_x = 4
    goal_y = 0
    state = 'GOAL_SEEKING'
    fronts = []
    left_fronts = []
    right_fronts = []
    
    def toGoal(self, goal_angle):
        if math.fabs(goal_angle) < math.pi/10:
            return 0.01
        else:
            return goal_angle

    def computeTranslation(self, fronts, left_fronts, right_fronts):
        min_front = min(fronts)
        min_left = min(left_fronts)
        min_right = min(right_fronts)
        min_sonar = min(min_front,min_left,min_right)
        if min_sonar < 0.3:
            return 0
        if min_sonar < 1:
            return 0.1 
        return 0.5

    def obstacleInWay(self, fronts, left_fronts, right_fronts):
        min_front = min(fronts)
        min_left = min(left_fronts)
        min_right = min(right_fronts)
        min_sonar = min(min_front, min_left, min_right)
        return min_sonar < 1
    
    def wallFollow(self, fronts, left_fronts, right_fronts):
        if min(fronts) < 1:
            return 1.5
        if min(left_fronts) <1:
            return -1.5
        if min(right_fronts) <1:
            return 1.5
        return 0

    def fifo(self, lista, new_value):
        head = lista[1]
        middle = lista[2]
        lista[0] = head
        lista[1] = middle
        lista[2] = new_value
        return lista


    def step(self):
        goal_angle = (math.atan2(self.goal_y - self.robot.y, self.goal_x - self.robot.x) - self.robot.thr) # % (2 * math.pi)
        
        #sonars
        front = min([s.distance() for s in self.robot.range["front"]])
        left_front = min([s.distance() for s in self.robot.range["left-front"]])
        right_front = min([s.distance() for s in self.robot.range["right-front"]])
        left = min([s.distance() for s in self.robot.range["left"]])
        right = min([s.distance() for s in self.robot.range["right"]])

        if len(self.fronts) < 3:
        	self.fronts.append(front)
        	self.left_fronts.append(left_front)
        	self.right_fronts.append(right_front)
        else:
        	self.fifo(self.fronts,front)
        	self.fifo(self.left_fronts,left_front)
        	self.fifo(self.right_fronts,right_front)
        
        translation = self.computeTranslation(self.fronts,self.left_fronts,self.right_fronts)
        	
        # if at target, stop
        print(self.state)
        if math.fabs(self.robot.x - self.goal_x) < self.threshold and math.fabs(self.robot.y-self.goal_y) < self.threshold :
            print("me=[",self.robot.x,",",self.robot.y,"] goal=[",self.goal_x,",",self.goal_y,"]\tgoal reached!")
            self.robot.move(0, 0)

        elif self.state == 'GOAL_SEEKING':
            rotation = self.toGoal(goal_angle)
            if (self.obstacleInWay(self.fronts,self.left_fronts,self.right_fronts)):
                self.state= 'WALL_FOLLOW'
                
        elif self.state == 'WALL_FOLLOW':
            rotation = self.wallFollow(self.fronts,self.left_fronts,self.right_fronts)
            if not self.obstacleInWay(self.fronts,self.left_fronts,self.right_fronts):
                self.state= 'GOAL_SEEKING'
           
        print(rotation)
        print(goal_angle)
        
        self.robot.move(translation, rotation)
def INIT(engine):
	assert engine.robot.requires("range-sensor") and engine.robot.requires(
        "continuous-movement"
    )
	return MyBrain2("MyBrain2", engine)

