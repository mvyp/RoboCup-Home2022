#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
import math
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from geometry_msgs.msg import Twist
from math import pi
import tf
import _thread
import azure.cognitiveservices.speech as speechsdk
from pan_tilt_msgs.msg import PanTiltCmdDeg
class receptionist:
    def __init__(self):
     
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.object_list=['water','bottle']
        self.find_object= False
        self.arrive_object= False
        #subsciber
        self.tf_listener = tf.TransformListener()
        self.arm_listener = rospy.Subscriber("topic", String)
        


        #publisher
        self.pub_pan_tilt = rospy.Publisher('/pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=1)

        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)

        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
        self.pan_tilt_up=PanTiltCmdDeg()
        self.pan_tilt_up.pitch=40
        self.pan_tilt_up.speed=20
        self.pub_pan_tilt.publish(self.pan_tilt_up)

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
        for i in self.object_list:
            try:
                trans = self.tfBuffer.lookup_transform("base_link", i, rospy.Time())
                if(trans.transform.translation.z<-0.05 and not self.find_object):
                    self.cancel()
                    self.find_object=True
                    print("find the {}".format(i))
                    angle=to_euler_angles(feedback.base_position.pose.orientation.w,feedback.base_position.pose.orientation.x,feedback.base_position.pose.orientation.y,feedback.base_position.pose.orientation.z)
                    object_positon = self.tfBuffer.lookup_transform("map", i, rospy.Time())
                    self.goto([object_positon.transform.translation.x,object_positon.transform.translation.y,angle+math.atan2(trans.transform.translation.y,trans.transform.translation.x)])
                    break
                if(trans.transform.translation.z<-0.05 and (math.sqrt(math.pow(trans.transform.translation.x,2) +math.pow(trans.transform.translation.y,2))<0.75)):
                    cmd_msg=Twist()
                    self.cmd_pub.publish(cmd_msg)
                    self.cancel()
                    if(abs(trans.transform.translation.y)<0.1):
                        print("right position")
                        break
                    elif(trans.transform.translation.y<0.2 or trans.transform.translation.y>-0.2):
                        cmd_msg.angular.z=trans.transform.translation.z
                        self.cmd_pub.publish(cmd_msg)
                        break
                    #TODO
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
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
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True
    
    # ----------Computer Vision--------------------------------------------------------------- 
    
    
    
    
    
    
    
    #main part
    def main(self):
        # say task begin
        text_to_speech("hi, which room should I clean ?")
        #  ~<the answer>
        answer = speech_to_text()
        goroom = message_proc(answer)
        print(goroom)
        if(goroom == 'kitchen'):
            #first point
            self.goto([1,3,0])
            print("hhh")

            if(self.arrive_object):
                data = None
                while data is None:
                    try:
                        data = rospy.wait_for_message("test", String, timeout=1)
                    except:
                        pass

            #second point
            self.find_object=0
            self.goto([0,0,0])

            if(self.arrive_object):
                data = None
                while data is None:
                    try:
                        data = rospy.wait_for_message("test", String, timeout=1)
                    except:
                        pass

            # #third point
            # self.find_object=0
            # result = self.goto([0,2,0])

            # if(self.arrive_object):
            #     data = None
            #     while data is None:
            #         try:
            #             data = rospy.wait_for_message("test", String, timeout=1)
            #         except:
            #             pass

            # if(result):
            #     exit()


        
# ----------Voice-------------------------------------------------------------------------
def text_to_speech(text):
    result = speech_synthesizer.speak_text_async(text).get()

    if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
        print("Speech synthesized to speaker for text [{}]".format(text))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech synthesis canceled: {}".format(cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            if cancellation_details.error_details:
                print("Error details: {}".format(cancellation_details.error_details))
        print("Did you update the subscription info?")
    
def speech_to_text():
    result = speech_recognizer.recognize_once()

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
def message_proc(string):
    #print(string.split())
    list=string.split()

    for i in list:
        if(i=="living"):
            room = 'living room.'
            break
        elif(i=="bedroom."):
            room = 'bedroom'
            break
        elif(i=="kitchen."):
            room = 'kitchen'
            break
        elif(i=="TODO."):
            room = 'TODO'
            break

    return room
def to_euler_angles(w, x, y, z):
    """w、x、y、z to euler angles"""
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    yaw = y*180/math.pi

    return yaw






if __name__ == "__main__":
    #Voice init
    speech_key, service_region = "80c72f7522eb4105aecaa9766104bd53", "eastus"
    speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
    speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"
    speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config)

    #ROS 
    rospy.init_node('receptionist',anonymous=True)

    receptionist_buct = receptionist()


    receptionist_buct.main()
    print("s")

    rospy.spin()
