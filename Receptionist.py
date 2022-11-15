#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf_conversions import transformations
from math import pi
import tf
import _thread
import azure.cognitiveservices.speech as speechsdk
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2 as cv
import requests
from io import BytesIO
import base64
import PIL.Image
from cv_bridge import CvBridge, CvBridgeError


class receptionist:

    def __init__(self):
        #Kinova
        self.pub_cmd = rospy.Publisher('Command', String, queue_size=1)

        #voice_init
        speech_key, service_region = "80c72f7522eb4105aecaa9766104bd53", "eastus"
        speech_config = speechsdk.SpeechConfig(subscription=speech_key,
                                               region=service_region)
        speech_recognizer = speechsdk.SpeechRecognizer(
            speech_config=speech_config)
        speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"
        speech_synthesizer = speechsdk.SpeechSynthesizer(
            speech_config=speech_config)

        #subsciber
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.img_callback, queue_size=1)

        #publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.set_pose_pub = rospy.Publisher('/initialpose',
                                            PoseWithCovarianceStamped,
                                            queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/map',
                                              '/base_link', rospy.Time(),
                                              rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

        #variable TODO
        self.master_name = 'jack'
        self.master_drink = 'orange'
        self.master_cloth = 'unknow'

        self.guest1_name = 'unknow'
        self.guest1_drink = 'unknow'
        self.guest1_upper_cloth = 'unknow'
        self.guest1_lower_cloth = 'unknow'
        self.guest1_age = 'unknow'
        self.guest1_glass = 'unknow'
        self.guest1_sex = 'unknow'


        self.guest2_name = 'unknow'
        self.guest2_drink = 'unknow'
        self.guest2_upper_cloth = 'unknow'
        self.guest2_lower_cloth = 'unknow'
        self.guest2_age = 'unknow'
        self.guest2_glass = 'unknow'
        self.guest2_sex = 'unknow'

        self.move_cmd = Twist()
        self.bridge_ros2cv = CvBridge()


# ----------Navigation-------------------------------------------------------------------------

    def get_pos(self):
        try:
            (trans,
             rot) = self.tf_listener.lookupTransform('/map', '/base_link',
                                                     rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
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
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
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
        rospy.loginfo("[Navi] goto %s" % p)
        x, y, th = p
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self._done_cb, self._active_cb,
                                 self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True
        return True

    # ----------Computer Vision---------------------------------------------------------------
    # def img_callback(self,img_msg):
    #     try:
    #         self.frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    #     except CvBridgeError as e:
    #         print (e)
    def frame2base64(self,frame):                       # 将格式转换成base64格式函数
        img = PIL.Image.fromarray(frame)               # 将每一帧转为Image
        output_buffer = BytesIO()                  # 创建一个BytesIO
        img.save(output_buffer, format='JPEG')     # 写入output_buffer
        byte_data = output_buffer.getvalue()       # 在内存中读取
        base64_data = base64.b64encode(byte_data)  # 转为BASE64
        return base64_data                         # 转码成功 返回base64编码


    def human_show(self,frame):                        # 人物特征检测函数
        request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/body_attr"
        img = self.frame2base64(frame)                   # 载入数据

        # 以下这一段是为了匹配相关数据头，固定格式，无需更改
        params = {"image": img}
        access_token = '24.cbba6b2fdce77a40ab866cde8e0550f2.2592000.1670079955.282335-28234814'
        request_url = request_url + "?access_token=" + access_token
        headers = {'content-type': 'application/x-www-form-urlencoded'}
        ########################################################

        # print('send data to baidu')
        response = requests.post(request_url, data=params, headers=headers)  # 读出API读回识别数据
        
        if response:                                                         # 如果回应就继续执行，说明API有响应
            print('API已相应')
            if 'person_num' in response.json():                              # 如果response.json()有person_num这个键值，说明响应正常无error响应（response.json()是一个字典里放了一个列表，列表里还有字典····）
                print('数据正常')                                              # 这一段大家可以直接print(response.json())，查找自己想要的信息，然后打印
                if response.json()['person_num'] != 0:                       # 如果有人才打印相关信息，即检测人数不为零（person_num键值对应的返回信息是图像中的人数）
                    print('人数：', response.json()['person_num'])
                    print('性别：', response.json()['person_info'][0]['attributes']['gender']['name'])
                    print('上身：', response.json()['person_info'][0]['attributes']['upper_color']['name'], '色',
                        response.json()['person_info'][0]['attributes']['upper_wear_fg']['name'])
                    print('下身：', response.json()['person_info'][0]['attributes']['lower_color']['name'], '色',
                        response.json()['person_info'][0]['attributes']['lower_wear']['name'])
                    print('年纪：', response.json()['person_info'][0]['attributes']['age']['name'])
                    print('眼镜：', response.json()['person_info'][0]['attributes']['glasses']['name'])
                    print('位置：', response.json()['person_info'][0]['location'])
                    return response.json()['person_info'][0]['attributes']['upper_color']['name'],response.json()['person_info'][0]['attributes']['lower_color']['name'],response.json()['person_info'][0]['attributes']['gender']['name'], response.json()['person_info'][0]['attributes']['glasses']['name'],response.json()['person_info'][0]['location']
                else:
                    print('当前无人')
            else:
                print('数据异常')
        else:
            print('API未响应') 




    #main part
    def main(self):
        # say task begin
        _thread.start_new_thread(text_to_speech, ("task begin!", ))
        # 导航到门口
        self.goto(1.653, -0.451, -90)
        #（第一个客人）
        # 机械臂开门
        self.pub_cmd.publish("Open_door")
        # 识别门口人的特征
        try:
            rosImg = rospy.wait_for_message('/rgb/image_raw/compressed', Image)  # 获取相机节点的ros图像
        except:
          print("cannot find picture")
        
        frame = self.bridge_ros2cv(rosImg, 'bgr8')  # 转化img
        self.guest1_upper_cloth,self.guest1_lower_cloth, self.guest1_glass,self.guest1_sex,loc=self.human_show(frame)
        
        text_to_speech("hi, what is your name and your favorate drink ?")
        #  ~<the answer>
        guest1_answer = speech_to_text()
        print(guest1_answer)
        self.guest1_name,self.guest1_drink= message_proc(guest1_answer)
        text_to_speech(
            "So, your name is {},and your favorate drink is {}.".format(
                self.guest1_name, self.guest1_drink))
        text_to_speech("follow me if possible, behind my body")

        # 导航到客厅
        self.goto(0, 0, 0)

        # 到客厅 TODO
        text_to_speech(
            "Please stand on my right side. And I will point to a seat you can take."
        )

        # 转向空座位

        # 机械臂向前指   3s 收回
        self.pub_cmd.publish("Front")
        #3s 收回
        rospy.sleep(3)
        self.pub_cmd.publish("Wait")

        text_to_speech("You can sit there.")

        # （第二个人）

        # 导航到门口
        self.goto(0, 0, 0)
        # 机械臂开门
        self.pub_cmd.publish("Open_door")
        # 识别门口人的特征
        self.guest2_upper_cloth,self.guest2_lower_cloth, self.guest2_glass,self.guest2_sex,loc=self.human_show()

        text_to_speech("hi, what is your name and your favorate drink ?")
        #  ~<the answer>
        guest2_answer = speech_to_text()
        self.guest2_name,self.guest2_drink= message_proc(guest2_answer)
        #process the answer TODO
        text_to_speech(
            "So, your name is {},and your favorate drink is {}.".format(
                self.guest2_name, self.guest2_drink))
        text_to_speech("follow me if possible, behind my body")

        # 导航到客厅
        self.goto(0, 0, 0)

        text_to_speech("Please stand on my right side.")
        #转向右边
        text_to_speech(
            "dear {} ,they are {} and {} , and their favorate drink are {} and{}."
            .format(self.guest2_name, self.master_name, self.guest1_name,
                    self.master_drink, self.guest1_drink))

        #转向大家 (Naming at least 4 characteristics of the first guest to the second guest)TODO

        text_to_speech(
            "dear {} and {}  , this is {} , and his favorate drink is {}.".
            format(self.guest1_name, self.master_name, self.guest2_name,
                   self.guest2_drink))

        text_to_speech(" I will point to a seat you can take.")

        # 转向空座位

        # 机械臂向前指   3s 收回
        self.pub_cmd.publish("Front")
        #3s 收回
        rospy.sleep(3)
        self.pub_cmd.publish("Wait")
        text_to_speech("You can sit there.")


# ----------Voice-------------------------------------------------------------------------
def text_to_speech(self, text):
    result = self.speech_synthesizer.speak_text_async(text).get()

    if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
        print("Speech synthesized to speaker for text [{}]".format(text))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech synthesis canceled: {}".format(
            cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            if cancellation_details.error_details:
                print("Error details: {}".format(
                    cancellation_details.error_details))
        print("Did you update the subscription info?")


def speech_to_text(self):
    result = self.speech_recognizer.recognize_once()

    # Checks result.
    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print("Recognized: {}".format(result.text))
    elif result.reason == speechsdk.ResultReason.NoMatch:
        print("No speech could be recognized: {}".format(
            result.no_match_details))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech Recognition canceled: {}".format(
            cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(
                cancellation_details.error_details))
    return result.text
def message_proc(string):
    #string="My name is Hua and my favoriate drink is orange juice."
    more_string=1
    #print(string.split())
    list=string.split()

    name=list[list.index("is")+1]
    print("name is "+name)

    list2=list
    del list2[list.index("is")]

    drink = list2[list2.index("is")+1]
    for i in drink:
        if( i =="."):
            more_string=0
    if(more_string):
        drink+=" "
        drink+=list2[list2.index("is")+2]
    drink =drink.strip('.')
    print("drink is "+drink)
    return name,drink

if __name__ == "__main__":
    #Voice init
    speech_key, service_region = "80c72f7522eb4105aecaa9766104bd53", "eastus"
    speech_config = speechsdk.SpeechConfig(subscription=speech_key,
                                           region=service_region)
    speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"
    speech_synthesizer = speechsdk.SpeechSynthesizer(
        speech_config=speech_config)
    #ROS
    rospy.init_node('receptionist', anonymous=True)

    receptionist_buct = receptionist()

    rospy.spin()
    receptionist_buct.main()
