import cv2 as cv
import requests
from io import BytesIO
import base64
import PIL.Image


# 以后直接在这里改 沙发的值

OBJ_X = 250  # 检测沙发左上角X坐标（目前只是自己拟值模拟，模型添加后直接替换该参数）
OBJ_Y = 370  # 检测沙发左上角Y坐标
OBJ_W = 150  # 检测沙发的宽
OBJ_H = 60   # 检测沙发的高


def frame2base64(frame):                       # 将格式转换成base64格式函数
    img = PIL.Image.fromarray(frame)               # 将每一帧转为Image
    output_buffer = BytesIO()                  # 创建一个BytesIO
    img.save(output_buffer, format='JPEG')     # 写入output_buffer
    byte_data = output_buffer.getvalue()       # 在内存中读取
    base64_data = base64.b64encode(byte_data)  # 转为BASE64
    return base64_data                         # 转码成功 返回base64编码


def human_show(frame):                        # 人物特征检测函数
    request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/body_attr"
    img = frame2base64(frame)                   # 载入数据

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
            else:
                print('当前无人')
        else:
            print('数据异常')
    else:
        print('API未响应')


def detect_show(img):                                                   # 人脸识别+沙发是否有人检测
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)                          # 转灰度图

    # 这个检测器用来识别人脸,效果比较差
    face_detect = cv.CascadeClassifier('haarcascade_frontalface_alt.xml')          # 人脸识别模型载入（这是一个.xml文件，要对应自己电脑中该文件的位置）
    # 不要写绝对路径
    # eye_detect = cv.CascadeClassifier('E:/Anaconda/Anaconda/envs/face1/Library/etc/haarcascades/haarcascade_eye_tree_eyeglasses.xml')     # 眼睛识别模型载入
    face = face_detect.detectMultiScale(gray)       # 人脸识别函数，返回人脸在图像中的位置，返回参数分别为x, y, w, h（左上角坐标和宽高）
    # eye = eye_detect.detectMultiScale(gray)
    seat_nmm = 0             # 定义座位标志位
    is_people = len(face)  # 定义是否有人  无人0 有人则>0

    for x, y, w, h in face:                                                    # 沙发上是否有人判别（运用状态穷举法，逻辑可能啰嗦，可优化）
        # cv.rectangle(img, (x, y), (x+w, y+h), color=(0, 255, 0), thickness=2)  # 用矩形方框框出人脸区域

        # 判断沙发上有无人: 我把你们的注释掉重写了
        # if ((x+w) > OBJ_X) and ((x+w) < (OBJ_X+OBJ_W)):                        # 如果人脸右边界在沙发右边界外，左脸边界在沙发右边界内，人在沙发上，座位标志位置1
        #     seat_nmm = 1
        # elif (x < (OBJ_X+OBJ_W)) and (x > OBJ_X):                              # 如果人脸左边界在沙发左边界外，右脸边界在沙发左边界内，人在沙发上，座位标志位置1
        #     seat_nmm = 1
        # elif ((x+x+w)/2 < (OBJ_X+OBJ_W)) and ((x+x+w)/2 > OBJ_X):              # 如果人脸中间位置左边在沙发左右边界内，人在沙发上，座位标志位置1
        #     seat_nmm = 1
        # else:
        #     seat_nmm = 0
        center_x, center_x_sofa = int(x + w/2), int(OBJ_X + OBJ_W/2)
        if abs(center_x - center_x_sofa) < OBJ_W*0.8:
            seat_nmm = 1
        else:
            seat_nmm = 0

    [x, y, w, h] = face[0] if is_people else [0, 0, 0, 0]     # 如果没有人的话就是0，0，0，0, 有人的话就是真实数值
    return is_people, seat_nmm, x, y, w, h


# 可视化部分我就是注释掉了 并重写了个函数
def visual(img, seat_nmm, x, y, w, h):
    cv.rectangle(img, (x, y), (x+w, y+h), color=(0, 255, 0), thickness=2)  # 用矩形方框框出人脸区域
    if seat_nmm == 1:                       # 如果标志位置1，说明有人，将沙发框选区域标注
        cv.rectangle(img, (OBJ_X, OBJ_Y), (OBJ_X+OBJ_W, OBJ_Y + OBJ_H), color=(0, 255, 0), thickness=2)      # 框住沙发
        cv.putText(img, 'Full', (OBJ_X, OBJ_Y), cv.FONT_HERSHEY_SIMPLEX, 1, (200, 255, 155), 2, cv.LINE_AA)  # 在图像上写“full seat”（满座）
        cv.putText(img, 'Seat', (OBJ_X, OBJ_Y+30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)
    else:
        cv.rectangle(img, (OBJ_X, OBJ_Y), (OBJ_X+OBJ_W, OBJ_Y + OBJ_H), color=(0, 0, 255), thickness=2)     # 框住沙发
        cv.putText(img, 'None', (OBJ_X, OBJ_Y), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)     # 在图像上写“none seat”（无人）
        cv.putText(img, 'Seat', (OBJ_X, OBJ_Y+30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)
    return img

    # cv.imshow('result', img)  # 将标注好的图片标注出来


# ######################################### 主函数开启 ################################ #

# img = cv.VideoCapture(0)    # 摄像头读取

# while True:                     # 进入主循环
#     flag, frame = img.read()    # 读取信息
#     if not flag:                # 如果信息为空，跳出循环
#         break
#     detect_show(frame)          # 沙发是否有人检测
#     rval, frame = img.read()    # 再读一次信息(可能冗余，待优化)
#     human_show(frame)           # 人体特征检测

#     if ord('q') == cv.waitKey(1):       # 按下q键退出
#         break

# cv.destroyAllWindows()
# img.release()


