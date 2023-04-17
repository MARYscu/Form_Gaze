import rospy
import sys
from std_msgs.msg import String
from nao_robot_study.msg import GameState
from nao_robot_study.msg import TimeState
from robot_behaviors import idleBehavior

from naoqi import ALProxy

gameInstructions = "Hello! Today you will be playing a memory test game. You will have \
    10 total minutes from the time you press start game to play the game. \
        To play the game, press the number tiles in order. Starting from the second round, the numbers on the tiles will \
            disappear after you press the first number. Try to get past as many rounds as you can!"

naoTips = "Nice job! Here are some helpful tips for the game. Try to associate the pattern of the numbers with something easier to remember, \
    for example, a letter or a shape. Additionally, you might not need to memorize the position of the last number \
        as it will be the only tile left on the screen once you click all the other numbers."

class RobotBehavior:
    def __init__(self, host, port):
        self.speechProxy = None
        self.postureProxy = None
        self.motionProxy = None
        self.host = host
        self.port = port
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
            self.postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
        except Exception as e:
            print("Could not create proxy to ALRobotPosture", e)
            exit(1)

        try:
            self.motionProxy = ALProxy("ALMotion", robotIP, 9559)
        except Exception as e:
            print("Could not create proxy to ALMotion")
            exit(1)

    def gamescore_msg_callback(self, data):
        rospy.loginfo("Game ended.")
        score = data.roundsComplete
        high_score = data.highScore
        losses = data.losses
        naoLine = "You completed " + str(score) + "rounds. Your current high score is " + str(high_score) + "numbers."
        self.speechProxy.say(naoLine)
        if losses == 1:
            self.speechProxy.say(naoTips)

    def timestate_msg_callback(self, data):
        minutesRemaining = round(data.minutesLeft / 60)
        rospy.loginfo("time update: %d minutes remaining", minutesRemaining)
        naoLine = "You have " + str(minutesRemaining) + "minutes remaining."
        self.speechProxy.say(naoLine)

    def idleBehavior(self):
        pass

    def releaseNao(self):
        self.speechProxy.say("Ending here")

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.idleBehavior()
            except KeyboardInterrupt:
                self.releaseNao()


def start_robot(host, port):
    robot = RobotBehavior(host, port)
    robot.speechProxy.say(gameInstructions)
    robot.run()


if __name__=='__main__':
    try:
        start_robot("192.168.2.117", 9559)
    except rospy.ROSInterruptException:
        pass
