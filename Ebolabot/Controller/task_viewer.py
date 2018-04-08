from klampt import *
from klampt import so3,se3,vectorops
from klampt.glprogram import *
from klampt import gldraw
import numpy as np
from sspp.topic import TopicServiceBase
import threading
import traceback

currentTaskLock = threading.Lock()
currentTask = None

class TaskViewerService(TopicServiceBase):
    def __init__(self,addr):
        TopicServiceBase.__init__(self,topic='.controller.task')
        self.open(addr)
    def onMessage(self,msg):
        global currentTask,currentTaskLock
        print msg
        currentTaskLock.acquire()
        currentTask = msg
        currentTaskLock.release()

class TaskViewer(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self,"Task Viewer")

    def display(self):
        global currentTask,currentTaskLock
        glEnable(GL_LIGHTING)
        currentTaskLock.acquire()
        task = currentTask
        if task != None:
            task = currentTask.copy()
        currentTaskLock.release() 
        if task != None:
            if task['type'] == 'multi_ik':
                for ikgoal in task['components']:
                    T = (so3.from_moment(ikgoal['endRotation']),ikgoal['endPosition'])
                    width = 0.02
                    length = 0.2                   
                    gldraw.xform_widget(T,length,width)

class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.service = TaskViewerService(('localhost',4568))
    def run(self):
        self.service.run()
    def kill(self):
        self.service.kill = True

if __name__ == "__main__":
    st = ServiceThread()
    st.start()
    try:
        program = TaskViewer()
        program.run()
    except Exception,AttributeError:
        print "Exception called in TaskViewer"
        traceback.format_exc()
    finally:
        print "Killing service thread"
        st.kill()


