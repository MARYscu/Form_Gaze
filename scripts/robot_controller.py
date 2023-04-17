import sys
import motion
import time
from naoqi import ALProxy
import rospy
from std_msgs.msg import String
from nao_robot_study.msg import GameState
from nao_robot_study.msg import TimeState

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

    def Nao_initial(self):
        self.StiffnessOff(self.motionProxy)
        self.motionProxy.openHand('LHand')
        self.motionProxy.openHand('RHand')
        time.sleep(1)

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

    def StiffnessOn(self, proxy):
        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"
        pStiffnessLists = 1.0
        pTimeLists = 1.0
        proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    def StiffnessOff(self, proxy):
        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"
        pStiffnessLists = 0.0
        pTimeLists = 1.0
        proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    # NAO idle behavior
    def idleHeadR(self, motionProxy):
        # 20s idle behavior
        # Set stiffness on for Head motors
        motionProxy.setStiffnesses("Head", 0.8)
        names= ["HeadYaw", "HeadPitch"]
        targetAngles = [[0.0, -0.4], [0.0, -0.4]]
        timeList = [[1.0, 4.0], [1.0, 4.0]]
        isAbsolute = True
        motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
        # time.sleep(10.0)
        targetAngles = [[-0.4, 0], [-0.4, 0]]
        timeList = [[1.0, 4.0], [1.0, 4.0]]
        motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
        motionProxy.setStiffnesses("Head", 0)

    def idleHeadL(self, motionProxy):
        # 20s idle behavior
        # Set stiffness on for Head motors
        motionProxy.setStiffnesses("Head", 0.8)
        names= ["HeadYaw", "HeadPitch"]
        targetAngles = [[0.0, 0.4], [0.0, -0.4]]
        timeList = [[1.0, 4.0], [1.0, 4.0]]
        isAbsolute = True
        motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
        # time.sleep(10.0)
        targetAngles = [[0.4, 0], [-0.4, 0]]
        timeList = [[1.0, 4.0], [1.0, 4.0]]
        motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
        motionProxy.setStiffnesses("Head", 0)

    def idleWrist(self, motionProxy):
        motionProxy.setStiffnesses("RWristYaw", 0.8)
        motionProxy.angleInterpolation("RWristYaw", [0.3, 0.7, 1.29], [1.0, 3.0, 5.0], True)
        # time.sleep(5)
        motionProxy.angleInterpolation("RWristYaw", -0.46, 1.0, True)
        motionProxy.setStiffnesses("RWristYaw", 0)

    def ArmControl_R_Waving(self, motionProxy):
        # Set stiffness on for Head motors

        print("ARM Control")
        # Initiate the Joints
        joints_R = ["RShoulderPitch", "RElbowRoll", "RShoulderRoll", "RElbowYaw", "RWristYaw"]
        default = {"RShoulderPitch": 1.54, "RWristYaw": -0.46, "RElbowYaw": 0.86, "RShoulderRoll": -0.4, "RElbowRoll": 1}
        isAbsolute = True
        for joint in joints_R:
            motionProxy.setStiffnesses(joint, 0.8)

        for k,v in default.items():
            motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

        # Raise hand and Wave
        names= ["RShoulderPitch", "RElbowRoll"]
        targetAngles = [[0, -0.78], [0.2, 1.3]]
        timeList = [[1.0, 2.0], [2.0, 4.0]]

        motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)

        # back to default
        for k,v in default.items():
            motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)
        for joint in joints_R:
            motionProxy.setStiffnesses(joint, 0)
        print("ARM Control End")

    def HandControl_R(self, motionProxy):

        print("RHAND CONTROL")
        # Initiate the Joints
        joints_R = ["RShoulderPitch", "RElbowRoll", "RShoulderRoll", "RElbowYaw", "RWristYaw"]
        default_R = {"RShoulderPitch": 1.54, "RWristYaw": -0.46, "RElbowYaw": 0.86, "RShoulderRoll": -0.4, "RElbowRoll": 1}
        isAbsolute = True
        for joint in joints_R:
            motionProxy.setStiffnesses(joint, 0.8)

        for k,v in default_R.items():
            motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

        motionProxy.angleInterpolation(["RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"], [-0.59, 1, 1.7, 1.5], 1, isAbsolute)


        motionProxy.openHand('RHand')
        time.sleep(0.5)
        motionProxy.closeHand('RHand')

        for k,v in default_R.items():
            motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

        for joint in joints_R:
            motionProxy.setStiffnesses(joint, 0)


        print("RHAND CONTROL END")

    # Left Arm is Broken
    def HandControl_L(self, motionProxy):

        print("LHAND CONTROL")
        # Initiate the Joints
        joints_L = ["LShoulderPitch", "LElbowRoll", "LShoulderRoll", "LElbowYaw", "LWristYaw"]
        default_L = {"LShoulderPitch": 1, "LWristYaw": -0.6, "LElbowYaw": -0.8, "LShoulderRoll": 0.35, "LElbowRoll": -0.6}
        isAbsolute = True
        for joint in joints_L:
            motionProxy.setStiffnesses(joint, 0.8)

        for k,v in default_L.items():
            motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

        motionProxy.angleInterpolation(["LShoulderPitch", "LWristYaw", "LElbowRoll", "RElbowYaw"], [1.3, -1.8, -1.4, -1.7], 1, isAbsolute)


        motionProxy.openHand('LHand')

        time.sleep(0.5)
        motionProxy.closeHand('LHand')


        for k,v in default_L.items():
            motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

        for joint in joints_L:
            motionProxy.setStiffnesses(joint, 0)
        print("LHAND CONTROL END")

    def releaseNao(self):
        self.speechProxy.say("Ending here")
        self.StiffnessOff(self.motionProxy)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.Nao_initial()
                start = time.time()
                time_passed = time.time() - start

                idle_index = -1
                while time_passed < 60:
                    time_passed = time.time() - start
                    if  15 < time_passed < 18:
                        self.HandControl_R(self.motionProxy)

                    elif 50 <time_passed < 60:
                        self.ArmControl_R_Waving(self.motionProxy)

                    else:
                        print("idle")
                        idle_index = (idle_index + 1) % 2
                    if (idle_index == 1):
                        # 20s
                        print("idle1")
                        self.idleWrist(self.motionProxy)
                        # time.sleep(10)
                    else:
                    # 60s
                        print("idle0")
                        self.idleHeadR(self.motionProxy)
                        # time.sleep(10)
                        self.idleHeadL(self.motionProxy)
                        # time.sleep(10)
                    self.releaseNao()

            except KeyboardInterrupt:
                self.releaseNao()



def start_robot(host, port):
    robot = RobotBehavior(host, port)
    time.sleep(1)
    robot.speechProxy.say(gameInstructions)
    robot.run()


if __name__=='__main__':
    try:
        start_robot("192.168.2.117", 9559)
    except rospy.ROSInterruptException:
        pass
