from campero_ur10_msgs.msg import ImgPoint, ImgTrace, ImageDraw

class MyImageDraw:
    def __init__(self, w, h, xoffset, yoffset):
        self.W = w
        self.H = h
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.traces = []

    def okey(self):
        return self.W > 0 and self.H > 0
        
    def clear(self):
        del self.traces[:]
    
    def addTrace(self):
        self.traces.insert(len(self.traces), [])

    def addPoint(self, x, y):
        xx = abs(x + self.xoffset)
        yy = abs(y + self.yoffset)
        pts = self.traces[len(self.traces) - 1]
        pts.insert(len(pts), (xx,yy))

    def processTrace(self, trace):
        if len(trace) == 0:
            return None
        
        _trace = ImgTrace()
        i = 0
        for pt in trace:
            x,y = pt
            if x >= 0 and x < self.W and y >= 0 and y < self.H:
                imgPoint = ImgPoint()
                imgPoint.x = x
                imgPoint.y = y
                _trace.points.insert(i, imgPoint)
                i += 1
        
        return _trace

    def getImgDraw(self):
        if len(self.traces) == 0:
            return None

        img = ImageDraw()
        img.type = ImageDraw.BOARD_POINTS
        i = 0
        for trace in self.traces:
            _trace = self.processTrace(trace)
            if _trace is not None:
                img.traces.insert(i, _trace)
                i += 1
        
        img.W = self.W
        img.H = self.H

        return img
