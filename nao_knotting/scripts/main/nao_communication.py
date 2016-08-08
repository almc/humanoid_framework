# -*- encoding: UTF-8 -*-

from naoqi import ALProxy

def init_proxies(robotIP, robotPORT):
    try:
        motionProxy = ALProxy("ALMotion", robotIP, robotPORT)
    except Exception, e:
        print "Could not create proxy to ALMotion due to:", e
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, robotPORT)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture due to:", e
    try:
        memoryProxy = ALProxy("ALMemory", robotIP, robotPORT)
    except Exception, e:
        print "Could not create proxy to ALMemory due to:", e
    try:
        speechProxy = ALProxy("ALTextToSpeech", robotIP, robotPORT)
        #lang = speechProxy.getAvailableLanguages();
        #print "Available languages: " + str(lang)
    except Exception,e:
        print "Could not create proxy to ALTextToSpeech due to:", e

    return motionProxy, postureProxy, memoryProxy, speechProxy
