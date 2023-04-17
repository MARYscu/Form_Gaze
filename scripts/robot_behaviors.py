import sys
import time
import random
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
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 0.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


# NAO Speaking Functions
def greeting(greetingProxy):
    greeting_script = "Hello"
    greetingProxy.say(greeting_script)
    game_rule = "game rule script"
    greetingProxy.say(game_rule)


# def human_control_response(greetingProxy):
#     response = raw_input("response: ")
#     greetingProxy.say(response)


# NAO idle behavior
def idleHeadR(motionProxy):
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



def idleHeadL(motionProxy):
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


def idleWrist(motionProxy):
    motionProxy.setStiffnesses("RWristYaw", 0.8)
    motionProxy.setStiffnesses("RElbowYaw", 0.8)
    motionProxy.angleInterpolation("RElbowYaw", [1, 1.3, 2], [1.0, 2.0, 3.0], True)
    motionProxy.angleInterpolation("RWristYaw", [0.3, 0.7, 1.29], [1.0, 3.0, 5.0], True)
    # time.sleep(5)

    motionProxy.angleInterpolation("RWristYaw", -0.46, 1.0, True)
    motionProxy.angleInterpolation("RElbowYaw", 0.86, 1.0, True)
    motionProxy.setStiffnesses("RWristYaw", 0)
    motionProxy.setStiffnesses("RElbowYaw", 0)




def ArmControl_R_Waving(motionProxy):
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


def HandControl_R(motionProxy):

    print("RHAND CONTROL")
    # Initiate the Joints
    joints_R = ["RShoulderPitch", "RElbowRoll", "RShoulderRoll", "RElbowYaw", "RWristYaw"]
    default_R = {"RShoulderPitch": 1.54, "RWristYaw": -0.46, "RElbowYaw": 0.86, "RShoulderRoll": -0.4, "RElbowRoll": 1}
    isAbsolute = True
    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0.8)

    for k,v in default_R.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

    motionProxy.angleInterpolation("RShoulderPitch", 0.89, 1, isAbsolute)
    motionProxy.angleInterpolation("RShoulderRoll", -0.61, 1, isAbsolute)
    motionProxy.angleInterpolation("RElbowRoll", 0.5, 1, isAbsolute)
    motionProxy.angleInterpolation("RElbowYaw", 1 , 1, isAbsolute)
    motionProxy.angleInterpolation("RWristYaw", 1.7, 1, isAbsolute)


    motionProxy.openHand('RHand')
    time.sleep(0.5)
    motionProxy.closeHand('RHand')

    for k,v in default_R.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0)


    print("RHAND CONTROL END")



# Left Arm is Broken
def HandControl_L(motionProxy):

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


def idleBehavior(motionProxy):
    rand_int = random.randint(0, 1)
    if (rand_int%2 == 0):
        # 20s
        print("idle1")
        idleWrist(motionProxy)
        # time.sleep(10)
    else:
        # 60s
        print("idle0")
        idleHeadR(motionProxy)
        # time.sleep(10)
        idleHeadL(motionProxy)
        # time.sleep(10)
