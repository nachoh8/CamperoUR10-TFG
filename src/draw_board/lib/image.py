from campero_ur10_msgs.msg import ImgPoint, ImgTrace, ImageDraw

SIZE_DEFAULT = 512
XOFFSET_DEFAULT = 0
YOFFSET_DEFAULT = 0

class MyImageDraw:
    def __init__(self, size=SIZE_DEFAULT, xoffset = XOFFSET_DEFAULT, yoffset = YOFFSET_DEFAULT):
        self.size = SIZE_DEFAULT if size < 0 else size
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.traces = []

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
            if x >= 0 and x < self.size and y >= 0 and y < self.size:
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
        i = 0
        for trace in self.traces:
            _trace = self.processTrace(trace)
            if _trace is not None:
                img.traces.insert(i, _trace)
                i += 1
        
        img.size = self.size

        return img