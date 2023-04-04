import sys
import motion
import time
from naoqi import ALProxy

# def idleBehavior(proxy):

def getSensors(proxy):
    names = ["RShoulderRoll", "RElbowRoll", "RElbowYaw", "RShoulderPitch",
             "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LShoulderPitch"]
    useSensors = False
    commandAngles = proxy.getAngles(names, useSensors)
    print "Command Angles"
    print str(commandAngles)
    print ""

    useSensors = True
    sensorAngles = proxy.getAngles(names, useSensors)
    print "Sensor Angles"
    print str(sensorAngles)
    print ""

    errors = []
    for i in range(0, len(commandAngles)):
        errors.append(commandAngles[i] - sensorAngles[i])
    print "Errors"
    print errors

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

def initialPosition(proxy):
    try: 
        names = ["RShoulderRoll", "RElbowRoll", "RElbowYaw", "RShoulderPitch"]
        angles = [[0.088], [0.66], [0.83], [0.5, 0.97]]
        timeList = [[5.0], [5.0], [5.0], [3.0, 7.0]]
        # , [0.22], [-0.035], [0.0], [0.5, 0.97]
        # , [2.0], [2.0], [2.0], [1.0, 3.0]
        # "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LShoulderPitch"
        isAbsolute = True
        proxy.angleInterpolation(names, angles, timeList, isAbsolute)
        names = ["LShoulderRoll", "LElbowRoll", "LElbowYaw", "LShoulderPitch"]
        angles = [[0.22], [-0.035], [0.0], [0.5, 0.97]]
        proxy.angleInterpolation(names, angles, timeList, isAbsolute)
    except Exception, e:
        print "Could not move"
        print "Error was: ", e

def idleHead(proxy):
    # Set stiffness on for Head motors
    proxy.setStiffnesses("Head", 0.8)

    # turn head idly?
    names= ["HeadYaw", "HeadPitch"]
    targetAngles = [[0.0, -1.0, 0.0], [0.0, -1.0, 0.0]]
    timeList = [[1.0, 4.0, 7.0], [1.0, 4.0, 7.0]]
    isAbsolute = True
    proxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
    time.sleep(4.0)
    names= ["HeadYaw", "HeadPitch"]
    targetAngles = [[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]]
    timeList = [[1.0, 4.0, 7.0], [1.0, 3.0, 7.0]]
    isAbsolute = True
    proxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
    time.sleep(4.0)
    
def idleBehavior(mproxy, lproxy):
    names = "RWristYaw"
    targetAngles = [-1.0, 1.0, 0.0]
    timeList = [0.5, 1.0, 1.5]
    isAbsolute = True
    mproxy.angleInterpolation(names, targetAngles, timeList, isAbsolute)
    lproxy.randomEyes(2.0)
    time.sleep(3.0)

def pointBehavior(proxy):
    pass

def waveDistract(proxy):
    pass

def standDistract(proxy):
    pass

def helpBehavior(mproxy, ttsproxy):
    ttsproxy.say("Do you need some help? I can give you some tips and tricks!")


def main(robotIP):


    motionProxy = ALProxy("ALMotion", robotIP, 9559)
    ledProxy = ALProxy("ALLeds", robotIP, 9559)

    StiffnessOn(motionProxy)

    try:
        tts = ALProxy("ALTextToSpeech", "192.168.2.113", 9559)
    except:
        print("Could not create proxy to tts")
        print("Error was: ", e)

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
        #postureProxy.goToPosture("Sit", 0.4)
        
        
        # initialPosition(motionProxy)
        # Send NAO to Pose Init
        # postureProxy.goToPosture("SitRelax", 0.4)
        #time.sleep(10.0)
        # StiffnessOff(motionProxy)
        
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    time.sleep(1.0)

    idleHead(motionProxy)
    time.sleep(5.0)
    idleBehavior(motionProxy, ledProxy)
    time.sleep(5.0)
    helpBehavior(motionProxy, tts)
    StiffnessOff(motionProxy)
    

    #waveDistract(motionProxy)
    #time.sleep(5.0)

    #idleBehavior(motionProxy)
    

    #helpBehavior(motionProxy, tts)



    
if __name__ == "__main__":
    robotIp = "192.168.2.113"

    main(robotIp)
