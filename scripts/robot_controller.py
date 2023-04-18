import sys
import motion
import time
from naoqi import ALProxy
import rospy
from std_msgs.msg import String
from nao_robot_study.msg import GameState
from nao_robot_study.msg import TimeState
from nao_robot_study.msg import GameUpdate
from robot_behaviors import idleBehavior, StiffnessOff, ArmControl_R_Waving, idleWrist, \
    Scratching_Head, Pointing, Raisingbothhands, FacetrackingOff, FacetrackingOn

from naoqi import ALProxy

gameInstructions = ["Hello my name is Nao!",  "The game instructions are on the tablet screen.", "When you are ready, press the start game button to begin."]

naoTips = ["Here is a helpful tip for the game.", "Try to associate the pattern of the numbers with something easier to remember, \
    such as a letter or a shape."]

roundEncouragement = ["You're doing great!", "Nice job so far!", "You've gotten past so many rounds!", "Keep it up!", "Amazing!", 
                      "Wow!", "I'm in awe!"]

sameOrLower = ["You can do it! Let's go again.", "Maybe not your highest score, but you're getting the hang of this!"]

higherScore = ["This time was your highest score yet! Let's keep going!", "You made an improvement on your high score again! You're doing great!"]

class RobotBehavior:
    def __init__(self, host, port):
        self.speechProxy = None
        self.postureProxy = None
        self.motionProxy = None
        self.faceProxy = None
        self.faceTracker = None
        self.host = host
        self.port = port
        self.instructions = True
        self.stiffnessoff = StiffnessOff
        self.roundEncTrack = 0
        self.timer = None
        self.callbackNum = 0
        self.sameLowTrack = 0
        self.higherScoreTrack = 0
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("game_state", GameState, self.gamescore_msg_callback)
        rospy.Subscriber("time_state", TimeState, self.timestate_msg_callback)
        rospy.Subscriber("update_state", GameUpdate, self.gameupdate_msg_callback)

        self.connectNao()

    def connectNao(self):
        try:
            self.speechProxy = ALProxy("ALTextToSpeech", self.host, self.port)
        except Exception, e:
            print "Error when creating speech device proxy:" + str(e)
            exit(1)

        try:
            self.postureProxy = ALProxy("ALRobotPosture", self.host, self.port)
        except Exception as e:
            print("Could not create proxy to ALRobotPosture", e)
            exit(1)

        try:
            self.motionProxy = ALProxy("ALMotion", self.host, self.port)
        except Exception as e:
            print("Could not create proxy to ALMotion")
            exit(1)
            
        try:
            self.faceProxy = ALProxy("ALFaceDetection", self.host, self.port)
        except Exception as e:
            print("Error when creating face detection proxy:" + str(e))
            exit(1)

        try:
            self.faceTracker = ALProxy("ALFaceTracker", self.host, self.port)
        except Exception as e:
            print("Error when creating face tracker proxy: " + str(e))
            exit(1)
         
        try:
            self.memoryProxy = ALProxy("ALMemory", self.host, 9559)
        except Exception as e:
            print("Error when creating memory proxy:")
            exit(1)

        self.timer = rospy.Timer(rospy.Duration(20), self.idleNaoBehavior)

    def Nao_initial(self):
        self.stiffnessoff(self.motionProxy)
        self.motionProxy.openHand('LHand')
        self.motionProxy.openHand('RHand')
        time.sleep(1)

    def idleNaoBehavior(self, event=None):
        idleBehavior(self.motionProxy)

    def gameupdate_msg_callback(self, data):
        self.timer.shutdown()
        FacetrackingOn(self.faceTracker, self.motionProxy)
        self.speechProxy.say(roundEncouragement[self.roundEncTrack])
        self.roundEncTrack += 1
        if self.roundEncTrack == len(roundEncouragement):
            self.roundEncTrack = 0
        FacetrackingOff(self.faceTracker, self.motionProxy)
        self.timer = rospy.Timer(rospy.Duration(20), self.idleNaoBehavior)

    def gamescore_msg_callback(self, data):
        self.timer.shutdown()
        FacetrackingOn(self.faceTracker, self.motionProxy)
        rospy.loginfo("Game ended.")
        score = data.roundsComplete
        high_score = data.highScore
        losses = data.losses
        gameInProgress = data.gameStart
        scoreDiff = data.highScoreDiff
        
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
            if self.callbackNum % 4 == 0:
                naoLine = "You completed " + str(score) + "rounds."
                self.speechProxy.say(naoLine)
                rospy.sleep(1)
                self.speechProxy.say("Your current high score is " + str(high_score) + "numbers.")
                self.callbackNum += 1
            elif scoreDiff > 0:
                self.speechProxy.say(higherScore[self.higherScoreTrack])
                self.higherScoreTrack += 1
                if self.higherScoreTrack == len(higherScore):
                    self.higherScoreTrack = 0
            elif scoreDiff <= 0:
                self.speechProxy.say(sameOrLower[self.sameLowTrack])
                self.sameLowTrack += 1
                if self.sameLowTrack == len(sameOrLower):
                    self.sameLowTrack = 0


            if losses == 1:
                rospy.sleep(3)
                self.speechProxy.say("It may be helpful to take more time to memorize the positions before pressing the first number.")
        FacetrackingOff(self.faceTracker, self.motionProxy)
        self.timer = rospy.Timer(rospy.Duration(20), self.idleNaoBehavior)
    
    def timestate_msg_callback(self, data):
        self.timer.shutdown()
        minutesRemaining = int(round(data.minutesLeft / 60))
        rospy.loginfo("time update: %d minutes remaining", minutesRemaining)
        if minutesRemaining == 1:
            FacetrackingOn(self.faceTracker, self.motionProxy)
            naoLine = "You have 1 minute remaining."
            self.speechProxy.say(naoLine)
        elif minutesRemaining == 6:
            self.timer.shutdown()
            Scratching_Head(self.motionProxy)
        elif minutesRemaining == 5:
            ArmControl_R_Waving(self.motionProxy)
        elif minutesRemaining == 4:
            FacetrackingOn(self.faceTracker, self.motionProxy)
            self.speechProxy.say("You have 4 minutes remaining.")
            rospy.sleep(2)
            for tip in naoTips:
                self.speechProxy.say(tip)
                rospy.sleep(1)
        elif minutesRemaining == 3:
            FacetrackingOn(self.faceTracker, self.motionProxy)
            Pointing(self.motionProxy, self.speechProxy) 
        elif minutesRemaining == 2:
            Raisingbothhands(self.motionProxy)
        FacetrackingOff(self.faceTracker, self.motionProxy)
        self.timer = rospy.Timer(rospy.Duration(20), self.idleNaoBehavior)

    def releaseNao(self):
        self.speechProxy.say("Ending here")
        self.stiffnessoff(self.motionProxy)

    def run(self):
        rospy.loginfo("initialized.")
        self.Nao_initial()
        Pointing(self.motionProxy, self.speechProxy)
        rospy.spin()
        

def start_robot(host, port):
    robot = RobotBehavior(host, port)
    rospy.sleep(1)
    robot.run()


if __name__=='__main__':
    try:
        start_robot("192.168.2.117", 9559)
    except rospy.ROSInterruptException:
        pass

