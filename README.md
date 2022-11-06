# teamwork
base.py是一个任务流程大框架

定义了导航和语音的基本函数

class中main函数为 任务流程

主要看 mian 及 类的init

添加新功能则写新函数

导航去某一个点 self.goto(0,0,0)

语音转文字 
多线程调用不会中断    _thread.start_new_thread( text_to_speech, ("task begin!",) )
也可直接调用 text_to_speech("hurry")

文字转语音  speech_to_text  会return text

用github的目的是可以查看提交的修改记录，找到每次不同

更新代码 git pull

上传代码 git push 
