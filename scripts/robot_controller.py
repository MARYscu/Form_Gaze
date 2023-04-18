import sys
import motion
import time
from naoqi import ALProxy
import rospy
from std_msgs.msg import String
from nao_robot_study.msg import GameState
from nao_robot_study.msg import TimeState
from robot_behaviors import idleBehavior, StiffnessOff, HandControl_R, ArmControl_R_Waving, idleWrist

from naoqi import ALProxy

gameInstructions = ["Hello my name is Nao!",  "The game instructions are on the tablet screen.", "When you are ready, press the start game button to begin."]

naoTips = ["Here is a helpful tip for the game.", "Try to associate the pattern of the numbers with something easier to remember, \
    such as a letter or a shape."]

class RobotBehavior:
    def __init__(self, host, port):
        self.speechProxy = None
        self.postureProxy = None
        self.motionProxy = None
        self.host = host
        self.port = port
        self.instructions = True
        self.stiffnessoff = StiffnessOff
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("game_state", GameState, self.gamescore_msg_callback)
        rospy.Subscriber("time_state", TimeState, self.timestate_msg_callback)

        self.connectNao()

    def connectNao(self):
        try:
            self.speechProxy = ALProxy("ALTextToSpeech", self.host, self.port)
        except Exception, e:
            print "Error when creating speech device proxy:" + str(e)
            exit(1)

        try:
            self.postureProxy = ALProxy("ALRobotPosture", self.host, 9559)
        except Exception as e:
            print("Could not create proxy to ALRobotPosture", e)
            exit(1)

        try:
            self.motionProxy = ALProxy("ALMotion", self.host, 9559)
        except Exception as e:
            print("Could not create proxy to ALMotion")
            exit(1)

    def Nao_initial(self):
        self.stiffnessoff(self.motionProxy)
        self.motionProxy.openHand('LHand')
        self.motionProxy.openHand('RHand')
        time.sleep(1)

    def idleNaoBehavior(self, event=None):
        idleBehavior(self.motionProxy)

    def gamescore_msg_callback(self, data):
        rospy.loginfo("Game ended.")
        score = data.roundsComplete
        high_score = data.highScore
        losses = data.losses
        gameInProgress = data.gameStart

        if not gameInProgress:
            naoLine = "Your final highest score is " + str(high_score) + "numbers. Good job!"
            self.speechProxy.say(naoLine)
            rospy.sleep(1)
            self.speechProxy.say("Thank you for taking part in this study.")
            rospy.sleep(1)
            self.speechProxy.say("Please find Kelly or Mary to fill out the post study survey.")
        elif self.instructions and data.gameStart:
            idleWrist(self.motionProxy.post)
            for instruction in gameInstructions:
                self.speechProxy.say(instruction)
                rospy.sleep(1)
            self.instructions = False
        else:
            naoLine = "You completed " + str(score) + "rounds."
            self.speechProxy.say(naoLine)
            rospy.sleep(1)
            self.speechProxy.say("Your current high score is " + str(high_score) + "numbers.")
            if losses == 1:
                rospy.sleep(3)
                self.speechProxy.say("It may be helpful to take more time to memorize the positions before pressing the first number.")

    def timestate_msg_callback(self, data):
        minutesRemaining = int(round(data.minutesLeft / 60))
        rospy.loginfo("time update: %d minutes remaining", minutesRemaining)
        if minutesRemaining % 2 == 1:
            naoLine = "You have " + str(minutesRemaining) + "minutes remaining."
            self.speechProxy.say(naoLine)
        elif minutesRemaining == 6:
            HandControl_R(self.motionProxy)
        elif minutesRemaining == 4:
            rospy.sleep(3)
            for tip in naoTips:
                self.speechProxy.say(tip)
                rospy.sleep(1)
        elif minutesRemaining == 2:
            ArmControl_R_Waving(self.motionProxy)

    def releaseNao(self):
        self.speechProxy.say("Ending here")
        self.stiffnessoff(self.motionProxy)

    def run(self):
        rospy.loginfo("initialized.")
        self.Nao_initial()
        timer = rospy.Timer(rospy.Duration(20), self.idleNaoBehavior)
        rospy.spin()
        timer.shutdown()

def start_robot(host, port):
    robot = RobotBehavior(host, port)
    rospy.sleep(1)
    robot.run()


if __name__=='__main__':
    try:
        start_robot("192.168.2.117", 9559)
    except rospy.ROSInterruptException:
        pass

