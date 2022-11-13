from human_tools import *
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge



"""
调用 human_tools 里面的函数

调用ros
写个循环
订阅 ros话题中深度相机的画面

进行\检测人脸\判断sofa\可视化


"""


# 主程序
if __name__ == "__main__":

    # ros话题
    # 创建发布者
    rospy.init_node('human_state')    
    pub = rospy.Publisher('human_state_detect', String, queue_size=10)
    bridge_ros2cv = CvBridge()

    is_visual = True  # 是否进行可视化

    while 1:
        # 接受ros图片
        try:
            rosImg = rospy.wait_for_message('/rgb/image_raw', Image)  # 获取相机节点的ros图像
        except:
            continue  # 如果没有图片就跳过这次循环,继续接受
        
        frame = bridge_ros2cv.imgmsg_to_cv2(rosImg, 'bgr8')  # 转化img

        # 沙发是否有人检测
        is_people, seat_nmm, x, y, w, h = detect_show(frame)

        # 把信息进行 ros发送
        result_message = str(is_people) + '-' + str(seat_nmm) + '-' + str(x) + str(y) + str(w) + str(h)
        pub.publish(result_message)

        if is_people: # 如果有人,再调用百度的api
            human_show(frame) 

        # 然后如果要进行可视化的话
        if is_visual:
            img = visual(frame, seat_nmm, x, y, w, h)
            cv.imshow('result', img)  # 将标注好的图片标注出来
            if ord('q') == cv.waitKey(1):       # 按下q键退出
                break
    
    if is_visual:
        cv.destroyAllWindows()
