"""Bounded latest-frame raster work. No widgets, transport or heartbeat here."""
from collections import OrderedDict
from types import SimpleNamespace
import threading
import time
from PySide6.QtCore import QRectF
from PySide6.QtGui import QImage,QPainter


class RasterFrame(SimpleNamespace):
    def width(self):return self.size[0]
    def height(self):return self.size[1]
    def rect(self):return QRectF(0,0,*self.size)
    def plot_rect(self):return QRectF(55,41,self.width()-73,self.height()-(86 if self.marker_times else 65))


class ChartRaster:
    def __init__(self):
        self.condition=threading.Condition();self.jobs=OrderedDict();self.results={}
        self.closed=False;self.thread=None;self.completed=0;self.max_wall_s=0.

    def submit(self,key,frame,paint):
        with self.condition:
            if self.closed:return
            self.jobs[key]=(frame,paint)
            if self.thread is None:
                self.thread=threading.Thread(target=self.run,name='chart raster only',daemon=True);self.thread.start()
            self.condition.notify()

    def take(self,key):
        with self.condition:return self.results.pop(key,None)

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.closed or self.jobs)
                if self.closed:return
                key,(frame,paint)=self.jobs.popitem(last=False)
            started=time.monotonic()
            try:
                image=QImage(round(frame.width()*frame.dpr),round(frame.height()*frame.dpr),QImage.Format.Format_ARGB32_Premultiplied)
                image.setDevicePixelRatio(frame.dpr);image.fill(0)
                painter=QPainter(image)
                try:paint(frame,painter)
                finally:painter.end()
                result=(frame.signature,image,None,frame.created)
            except Exception as exc:result=(frame.signature,None,str(exc),frame.created)
            with self.condition:
                self.completed+=1;self.max_wall_s=max(self.max_wall_s,time.monotonic()-started)
                if not self.closed:self.results[key]=result

    def close(self):
        with self.condition:
            self.closed=True;self.jobs.clear();self.results.clear();self.condition.notify_all()
        if self.thread:self.thread.join(timeout=1.)
