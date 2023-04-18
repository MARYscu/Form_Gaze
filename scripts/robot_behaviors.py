import sys
import time
import random
from naoqi import ALProxy



#reference
# http://www.cs.cmu.edu/~cga/nao/doc/reference-documentation/nao/hardware/kinematics/nao-joints-32.html#hardware-kin-v3-2-left-arm-joints

def set_Facetracking_on(faceProxy, motionProxy):
    """Make a proxy to nao's ALFaceDetection and enable/disable tracking.

    """
    motionProxy.setStiffnesses("Head", 1)
    # Enable or disable tracking.
    faceProxy.enableTracking(True)
    # Just to make sure correct option is set.
    print("Is tracking now enabled on the robot?", faceProxy.isTrackingEnabled())
def set_Facetracking_off(faceProxy, motionProxy):
    # Enable or disable tracking.
    faceProxy.enableTracking(False)
    motionProxy.setStiffnesses("Head", 0)
    # Just to make sure correct option is set.
    print("Is tracking now enabled on the robot?", faceProxy.isTrackingEnabled())


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
def vocal_distraction(greetingProxy):
    greeting_script = "Can you pass that bag to me?"
    greetingProxy.say(greeting_script)


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
    motionProxy.angleInterpolation("RShoulderPitch", 0.78, 1, True)
    motionProxy.angleInterpolation("RElbowYaw", [1, 1.3, 2], [1.0, 2.0, 3.0], True)
    motionProxy.angleInterpolation("RWristYaw", [0.3, 0.7, 1.29], [1.0, 3.0, 5.0], True)
    # time.sleep(5)

    motionProxy.angleInterpolation("RWristYaw", -0.46, 1.0, True)
    motionProxy.angleInterpolation("RElbowYaw", 0.86, 1.0, True)
    motionProxy.angleInterpolation("RShoulderPitch", 1.54, 1, True)
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


    motionProxy.angleInterpolation("RShoulderRoll", -0.6, 1, isAbsolute)
    # Raise hand and Wave

    motionProxy.angleInterpolation("RShoulderPitch", [0, -0.78], [1.0, 2.0], isAbsolute)

    for _ in range(3):
        names= ["RElbowRoll", "RShoulderRoll"]
        targetAngles = [[0.2, 1.3], [0, -0.8]]
        timeList = [[1.0, 2.0], [1.0, 2.0]]

        motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)


    # back to default
    for k,v in default.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)
    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0)
    print("ARM Control End")

def Head_Position(motionProxy, position):
    # Set stiffness on for Head motors
    motionProxy.setStiffnesses("Head", 0.8)
    isAbsolute = True
    motionProxy.angleInterpolation("HeadPitch", 0, 1, isAbsolute)
    motionProxy.angleInterpolation("HeadYaw", position, 1, isAbsolute)
    motionProxy.setStiffnesses("Head", 0)

def Scratching_Head(motionProxy):
    # Initiate the Joints
    print("Scratching Head")
    joints_R = ["RShoulderRoll", "RWristYaw", "RElbowYaw", "RElbowRoll", "RShoulderPitch"]
    default = {"RShoulderRoll": -0.4, "RWristYaw": -0.46, "RElbowYaw": 0.86,  "RElbowRoll": 1, "RShoulderPitch": 1.54 }
    isAbsolute = True
    motionProxy.setStiffnesses("Head", 0.8)
    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0.8)

    for k,v in default.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

    # Raise hand and Wave
    names= ["RShoulderPitch", "RElbowYaw", "RElbowRoll", "RShoulderRoll"]
    targetAngles = [[0, -0.30], [0.7, 0.94], [1, 1.48], [-0.35, -0.08]]
    timeList = [[1.0, 3.0], [4, 5], [4, 5], [3, 4]]
    motionProxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
    motionProxy.angleInterpolation("HeadPitch", 0.19, 1, isAbsolute)

    names= ["RHand", "RWristYaw"]
    targetAngles = [[0.57, 1.3]]
    timeList = [[1.0, 3.0, 4.0, 7.0]]
    motionProxy.angleInterpolation("RWristYaw", [0.57, 1.4], [1.0, 3.0], isAbsolute)



    for _ in range(3):
        motionProxy.openHand('RHand')
        motionProxy.closeHand('RHand')

    motionProxy.angleInterpolation("RWristYaw", [0.57, 1.4], [1.0, 3.0], isAbsolute)

    for _ in range(2):
        motionProxy.openHand('RHand')
        motionProxy.closeHand('RHand')

    # back to default
    motionProxy.angleInterpolation("HeadPitch", 0, 1, isAbsolute)
    for k,v in default.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)
    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0)
    motionProxy.setStiffnesses("Head", 0)
    print("Scratching End")



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

    motionProxy.angleInterpolation("RShoulderPitch", 0.55, 1, isAbsolute)
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
def Raisingbothhands(motionProxy):
    print("Raisingbothhands")
    joints_R = ["RShoulderRoll", "RWristYaw", "RElbowYaw", "RElbowRoll", "RShoulderPitch"]
    default_R = {"RShoulderRoll": -0.4, "RWristYaw": -0.46, "RElbowYaw": 0.86,  "RElbowRoll": 1, "RShoulderPitch": 1.54 }
    isAbsolute = True
    joints_L = ["LShoulderRoll", "LShoulderPitch"]
    # motionProxy.angleInterpolation("LShoulderPitch", [0, 0.8, -0.75, -0.8], [1, 2, 3, 4], isAbsolute)
    default_L = {"LShoulderRoll": 0.38, "LShoulderPitch": 0.75}

    isAbsolute = True
    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0.8)
    for k,v in default_R.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

    for joint in joints_L:
        motionProxy.setStiffnesses(joint, 0.8)
    for k,v in default_L.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

    motionProxy.angleInterpolation(["RShoulderPitch", "LShoulderPitch"], [0, -0.62], 1, isAbsolute)
    # motionProxy.angleInterpolation("LShoulderPitch", -0.62, 1, isAbsolute)
    motionProxy.angleInterpolation(["RShoulderRoll", "LShoulderRoll"], [[-0.47, 0.08],[-0.2, 1.1]], [[1, 2], [1, 2]], isAbsolute)
    motionProxy.angleInterpolation(["RShoulderRoll", "LShoulderRoll"], [[0.08, -0.47],[1.1, -0.2]], [[1, 2], [1, 2]], isAbsolute)
    motionProxy.angleInterpolation("RElbowRoll", 0.5, 1, isAbsolute)
    motionProxy.angleInterpolation("RElbowYaw", 1 , 1, isAbsolute)

    for k,v in default_R.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)
    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0)

    for k,v in default_L.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)
    for joint in joints_L:
        motionProxy.setStiffnesses(joint, 0)

def AnkleLift(motionProxy):
    print("AnkleLift")

    motionProxy.setStiffnesses("LAnklePitch", 0.8)
    isAbsolute = True
    for _ in range(3):
        motionProxy.angleInterpolation("LAnklePitch", [-0.8, -0.4], [1, 2], isAbsolute)
    motionProxy.setStiffnesses("LAnklePitch", 0)



def Pointing(motionProxy, greetingProxy):
    print("Pointing")
    joints_R = ["RShoulderPitch", "RElbowRoll", "RShoulderRoll", "RElbowYaw", "RWristYaw"]
    default_R = {"RShoulderPitch": 1.54, "RWristYaw": -0.46, "RElbowYaw": 0.86, "RShoulderRoll": -0.4, "RElbowRoll": 1}
    isAbsolute = True
    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0.8)


    joints_move = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw",  "RElbowRoll",  "RWristYaw"]
    move_position = [0.21, -0.2,  0.57,  0.41, -0.9]

    motionProxy.angleInterpolation(joints_move, move_position, [[1], [1], [2], [2], [2]], isAbsolute)
    time.sleep(5)
    vocal_distraction(greetingProxy)
    time.sleep(5)

    for k,v in default_R.items():
        motionProxy.angleInterpolation(k, v, 1.0, isAbsolute)

    for joint in joints_R:
        motionProxy.setStiffnesses(joint, 0)

    print("Pointing End")







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

def eye_tracking(memoryProxy, faceProxy, motionProxy):
    motionProxy.angleInterpolation("HeadPitch", 0, 1, True)
    period = 500
    faceProxy.subscribe("Test_Face", period, 0.0 )
    val = memoryProxy.getData("FaceDetected")
    increment = 0
    # faceProxy.enableTracking(True)
    # Just to make sure correct option is set.
    if (val and isinstance(val, list) and len(val) >= 2):
        print("face")
    else:

        while (isinstance(val, list) == False or len(val) < 2):
            val = memoryProxy.getData("FaceDetected")
            print("no face ")
            motionProxy.setStiffnesses("Head", 1.0)
            # motionProxy.setAngles("HeadYaw", 0.0, 0.1)
            increment -= 0.15
            motionProxy.angleInterpolation("HeadYaw",increment , 1, True)
    return increment

