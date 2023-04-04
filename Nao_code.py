import sys
import motion
import time
from naoqi import ALProxy

#reference
# http://www.cs.cmu.edu/~cga/nao/doc/reference-documentation/nao/hardware/kinematics/nao-joints-32.html#hardware-kin-v3-2-left-arm-joints

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def StiffnessOff(proxy):
    # make sure the robot sits down
    proxy.goToPosture("Sit", 1.0)
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 0.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


# NAO Speaking Functions
def greeting(greetingProxy):
    greeting_script = "greeting script"
    greetingProxy.say(greeting_script)


def game_rule(greetingProxy):
    game_rule = "game rule script"
    greetingProxy.say(game_rule)


def human_control_response(greetingProxy):
    response = raw_input("response: ")
    greetingProxy.say(response)


# NAO idle behavior
def idleHead(motionProxy):
    # Set stiffness on for Head motors
    motionProxy.setStiffnesses("Head", 0.8)
    names= ["HeadYaw", "HeadPitch"]
    targetAngles = [[0.0, -1.0, 0.0], [0.0, -1.0, 0.0]]
    timeList = [[1.0, 4.0, 7.0], [1.0, 4.0, 7.0]]
    isAbsolute = True
    motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
    time.sleep(4.0)
    names= ["HeadYaw", "HeadPitch"]
    targetAngles = [[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]]
    timeList = [[1.0, 4.0, 7.0], [1.0, 3.0, 7.0]]
    isAbsolute = True
    motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
    time.sleep(4.0)
    motionProxy.setStiffnesses("Head", 0)

# NAO Distraction

def arm_move(motionProxy, effectorName):

    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)

    space = motion.FRAME_ROBOT
    useSensor = False

    effectorInit = motionProxy.getPosition(effectorName, space, useSensor)

    # Active LArm tracking
    isEnabled = True
    motionProxy.wbEnableEffectorControl(effectorName, isEnabled)

    # Example showing how to set position target for LArm
    # The 3 coordinates are absolute LArm position in NAO_SPACE
    # Position in meter in x, y and z axis.

    # X Axis LArm Position feasible movement = [ +0.00, +0.12] meter
    # Y Axis LArm Position feasible movement = [ -0.05, +0.10] meter
    # Y Axis RArm Position feasible movement = [ -0.10, +0.05] meter
    # Z Axis LArm Position feasible movement = [ -0.10, +0.10] meter

    coef = 1.0
    if (effectorName == "LArm"):
        coef = +1.0
    elif (effectorName == "RArm"):
        coef = -1.0

    targetCoordinateList = [ [ +0.12, +0.00*coef, +0.00], [ +0.12, +0.00*coef, -0.10]]
    # [ +0.12, +0.00*coef, +0.00],  # target 0
    # [ +0.12, +0.00*coef, -0.10], # target 1
    # [ +0.12, +0.05*coef, -0.10], # target 1
    # [ +0.12, +0.05*coef, +0.10], # target 2
    # [ +0.12, -0.10*coef, +0.10], # target 3
    # [ +0.12, -0.10*coef, -0.10], # target 4
    # [ +0.12, +0.00*coef, -0.10], # target 5
    # [ +0.12, +0.00*coef, +0.00], # target 6
    # [ +0.00, +0.00*coef, +0.00], # target 7

    # wbSetEffectorControl is a non blocking function
    # time.sleep allow head go to his target
    # The recommended minimum period between two successives set commands is
    # 0.2 s.
    for targetCoordinate in targetCoordinateList:
        targetCoordinate = [targetCoordinate[i] + effectorInit[i] for i in range(3)]
        motionProxy.wbSetEffectorControl(effectorName, targetCoordinate)
        time.sleep(4.0)

    # Deactivate Head tracking
    isEnabled    = False
    motionProxy.wbEnableEffectorControl(effectorName, isEnabled)

def sit(postureProxy):
    postureProxy.goToPosture("SitRelax", 1.0)
    # postureProxy.goToPosture("StandZero", 1.0)
    # postureProxy.goToPosture("LyingBelly", 1.0)
    # postureProxy.goToPosture("LyingBack", 1.0)
    # postureProxy.goToPosture("Stand", 1.0)
    # postureProxy.goToPosture("Crouch", 1.0)
    # postureProxy.goToPosture("Sit", 1.0)


def main(robotIP):

    # initiating all the proxy
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception as e:
        print("Could not create proxy to ALRobotPosture", e)

    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception as e:
        print("Could not create proxy to ALMotion")
        print("Error was: ", e)

    try:
        greetingProxy = ALProxy("ALTextToSpeech", robotIP, 9559)
    except Exception as e:
        print("Could not create proxy to ALTextToSpeech", e)

    start = time.time()
    time_passed = time.time() - start

    while time_passed < 600:
        time_passed = time.time() - start
        if time_passed == 20:
            greeting(greetingProxy)
            game_rule(greetingProxy)
        elif 200 >= time_passed >= 180:
            arm_move(motionProxy, "LArm")
            motionProxy.openHand('LHand')
        elif 500 >= time_passed >= 480:
            postureProxy.goToPosture("Stand", 1.0)
            arm_move(motionProxy, "RArm")
        else:
            StiffnessOff(robotIp)
            idleHead(motionProxy)

    StiffnessOff(robotIp)



if __name__ == "__main__":
    robotIp = "192.168.2.113"

    if len(sys.argv) <= 1:
        print("Usage python alrobotposture.py robotIP (optional default: 192.168.2.113)")
    else:
        robotIp = sys.argv[1]


    # testing
    while True:
        NAObehavior = int(input("Please enter the behavior:\n\
            (0: exit 1: greeting 2: game rule 3: human control response 4:move left arm\
            5: move right arm): "))
        try:
            if NAObehavior == 1:
                greeting(robotIp)
            if NAObehavior == 2:
                game_rule(robotIp)
            if NAObehavior == 3:
                human_control_response(robotIp)
            if NAObehavior == 4:
                arm_move(robotIp, "LArm")
            if NAObehavior == 5:
                arm_move(robotIp, "RArm")
            if NAObehavior == 0:
                StiffnessOff(robotIp)
                break
        except ValueError:
            print("Wrong Input")
            continue
