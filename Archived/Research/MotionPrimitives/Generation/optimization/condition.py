import time

class CountCondition:
    def __init__(self,maxCount):
        self.maxCount = maxCount
        self.count =0
    def start(self):
        self.count = 0
    def __call__(self):
        if self.count >= self.maxCount: return False
        self.count += 1
        return True

class TimeCondition:
    def __init__(self,seconds):
        self.seconds = seconds
        self.endTime = time.time()+seconds
    def start(self):
        self.endTime = time.time()+self.seconds
    def __call__(self):
        return time.time() <= self.endTime
