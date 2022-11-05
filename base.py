#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
import tf
import _thread
import azure.cognitiveservices.speech as speechsdk
class receptionist:
    def __init__(self):
        #voice_init
        speech_key, service_region = "80c72f7522eb4105aecaa9766104bd53", "eastus"
        speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
        speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config)
        speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"
        speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)

        #subsciber
        #publisher
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

# ----------Navigation-------------------------------------------------------------------------
    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return False
        euler = transformations.euler_from_quaternion(rot)
        x = trans[0]
        y = trans[1]
        th = euler[2] / pi * 180
        return (x, y, th)

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("goal reached! ")

    def _active_cb(self):
        rospy.loginfo("navigation has be actived")

    def _feedback_cb(self, feedback):
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)
        pass
    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
    def goto(self, p):
        global xc, yc, zc
        rospy.loginfo("[Navi] goto %s"%p)
        x, y, th = p
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.canavigation_demo
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True
    
    # ----------Computer Vision--------------------------------------------------------------- 
    
    
    
    
    
    
    
    #main part
    def main(self):
        # say task begin
        _thread.start_new_thread( text_to_speech, ("task begin!",) )
        #nav
        self.goto(0,0,0)
        
        pass
# ----------Voice-------------------------------------------------------------------------
def text_to_speech(self,text):
    result = self.speech_synthesizer.speak_text_async(text).get()

    if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
        print("Speech synthesized to speaker for text [{}]".format(text))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech synthesis canceled: {}".format(cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            if cancellation_details.error_details:
                print("Error details: {}".format(cancellation_details.error_details))
        print("Did you update the subscription info?")
        
def speech_to_text(self):
    result = self.speech_recognizer.recognize_once()

    # Checks result.
    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print("Recognized: {}".format(result.text))
    elif result.reason == speechsdk.ResultReason.NoMatch:
        print("No speech could be recognized: {}".format(result.no_match_details))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech Recognition canceled: {}".format(cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(cancellation_details.error_details))
    return result.text







if __name__ == "__main__":
    #Voice init
    speech_key, service_region = "80c72f7522eb4105aecaa9766104bd53", "eastus"
    speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
    speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"
    speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)
    #ROS 
    rospy.init_node('receptionist',anonymous=True)

    receptionist_buct = receptionist()


    
    rospy.spin()
    receptionist_buct.main()
#    for goal in goals:
#        receptionist.goto(goal)
#    while not rospy.is_shutdown():
#        r.sleep()
