"""Adapted from reference/legacy_furuta/pc_host/furuta_pc/widgets.py.
Theme-aware charts and pose; reference sources remain unchanged.
"""
from .theme import PALETTES
from .presentation import axis_ticks, tick_label
from .angle_display import trace_segments, visible_trace_segments
from .ui_timing import timed_ui
from .command_display import marker_caption,marker_detail
from collections import deque
import math
import time
from .chart_raster import RasterFrame
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPalette, QPolygonF
from PySide6.QtWidgets import QWidget, QComboBox, QListView, QToolTip

TEAL = '#44dec0'
BLUE = '#6eacff'
AMBER = '#f5bf62'
MUTED = '#8596ae'


class ThemedComboBox(QComboBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setView(QListView())
        self.view().setUniformItemSizes(True)
        self.theme="dark"

    def set_theme(self,name):
        self.theme=name;colors=PALETTES[name];palette=QPalette(self.palette())
        roles={'Window':'panel','Base':'panel','AlternateBase':'field','Text':'text','WindowText':'text','ButtonText':'text','Highlight':'hover','HighlightedText':'text'}
        for group in (QPalette.ColorGroup.Active,QPalette.ColorGroup.Inactive,QPalette.ColorGroup.Disabled):
            for role,key in roles.items():palette.setColor(group,getattr(QPalette.ColorRole,role),QColor(colors[key]))
        for widget in (self,self.view(),self.view().viewport(),self.view().window()):
            widget.setPalette(palette);widget.setAutoFillBackground(True)
        self.view().setStyleSheet('QListView, QListView QWidget { background: '+colors['panel']+'; color: '+colors['text']+'; } QListView::item { min-height: 24px; padding: 5px 10px; } QListView::item:selected { background: '+colors['hover']+'; color: '+colors['text']+'; }')

    def showPopup(self):
        self.set_theme(self.theme)
        super().showPopup()

    def paintEvent(self, event):
        super().paintEvent(event)
        p=QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(QPen(self.palette().color(QPalette.ColorRole.Text),1.4))
        x,y=self.width()-16,self.height()/2
        p.drawLine(QPointF(x-4,y-2),QPointF(x,y+2))
        p.drawLine(QPointF(x,y+2),QPointF(x+4,y-2))
        p.end()


class TraceChart(QWidget):
    def __init__(self, title, series, bounds=None, parent=None, periodic=False, fast=False):
        super().__init__(parent)
        self.title, self.series, self.bounds = title, series, bounds
        self.periodic = periodic
        self.fast = fast
        self.samples = deque(maxlen=3000)
        self.markers = []
        self.marker_labels = False
        self.marker_times = False
        self.setMouseTracking(True)
        self.window = 20
        self.theme = "dark"
        self.setMinimumHeight(120)
        self.setMinimumWidth(300)
        self.frame_renderer=None;self.raster_image=None;self.raster_signature=None
        self.raster_requested=None;self.raster_at=-float('inf');self.raster_error=None
        self.raster_generation=0;self.raster_display_at=0.

    def clear(self):
        self.samples.clear();self.raster_generation+=1
        self.raster_image=None;self.raster_requested=None

    def append(self, sample, redraw=True):
        self.samples.append(sample)
        if redraw:
            self.update()

    def plot_rect(self):
        return QRectF(55, 41, self.width()-73, self.height()-(86 if self.marker_times else 65))

    @timed_ui('paint.chart')
    def paintEvent(self, event):
        if self.frame_renderer is not None and self.samples:
            signature=(self.raster_generation,self.width(),self.height(),self.devicePixelRatioF(),self.theme,self.window,tuple(self.series),self.bounds,self.periodic,self.fast,self.marker_labels,self.marker_times)
            result=self.frame_renderer.take(id(self))
            if result:
                saved,image,error,created=result;self.raster_error=error
                if saved==signature and image is not None:self.raster_image=image;self.raster_signature=saved;self.raster_display_at=created
            now=time.monotonic()
            if self.raster_requested!=signature or now-self.raster_at>=.1:
                frame=RasterFrame(size=(self.width(),self.height()),dpr=self.devicePixelRatioF(),signature=signature,created=now,
                    title=self.title,series=tuple(self.series),bounds=self.bounds,periodic=self.periodic,fast=self.fast,
                    samples=tuple(self.samples),markers=tuple(dict(m) for m in self.markers),marker_labels=self.marker_labels,
                    marker_times=self.marker_times,window=self.window,theme=self.theme)
                self.frame_renderer.submit(id(self),frame,TraceChart.paint_frame)
                self.raster_requested=signature;self.raster_at=now
            p=QPainter(self)
            if self.raster_image is not None and self.raster_signature==signature:p.drawImage(0,0,self.raster_image)
            else:
                p.fillRect(self.rect(),QColor(PALETTES[self.theme]['panel']));p.setPen(QColor(PALETTES[self.theme]['muted']))
                p.drawText(self.rect(),Qt.AlignmentFlag.AlignCenter,'图表绘制失败：'+self.raster_error if self.raster_error else '正在绘制完整曲线…')
            if self.raster_image is not None and (self.raster_error or now-self.raster_display_at>.25):
                p.setPen(QColor(PALETTES[self.theme]['amber']))
                p.drawText(58,38,'图表绘制失败：'+self.raster_error if self.raster_error else '曲线画面更新滞后；设备新鲜度见状态栏')
            p.end();return
        p=QPainter(self)
        try:self.paint_frame(p)
        finally:p.end()

    def paint_frame(self,p):
        colors = PALETTES[self.theme]
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(colors['panel']))
        p.setPen(QColor(colors['text']))
        p.setFont(QFont('Microsoft YaHei UI', 10, QFont.Weight.DemiBold))
        p.drawText(16, 24, self.title)
        x = self.width() - 18
        p.setFont(QFont('Microsoft YaHei UI', 8))
        for key, label, color in reversed(self.series):
            width = p.fontMetrics().horizontalAdvance(label) + 34
            x -= width
            p.setPen(QPen(QColor(colors.get(color, color)), 2))
            p.drawLine(x, 20, x+14, 20)
            p.setPen(QColor(colors['muted']))
            p.drawText(x+20, 24, label)
        rect = self.plot_rect()
        end = max(self.window, self.samples[-1]['t']) if self.samples else self.window
        begin = end-self.window
        rows = [s for s in self.samples if s['t'] >= begin]
        values = [s[k] for s in rows for k, _, _ in self.series
                  if math.isfinite(s[k]) and (k not in ('alpha', 'beta') or s['valid'])]
        if self.bounds:
            low, high = self.bounds
        elif values:
            low, high = min(min(values), 0), max(max(values), .05)
            pad = max(.01, (high-low)*.12)
            low, high = low-pad, high+pad
        else:
            low, high = -90, 210
        p.setFont(QFont('Segoe UI', 8))
        ticks=axis_ticks(low,high,angle=any(k in ('theta','alpha','beta') for k,_,_ in self.series))
        if self.bounds==(-200,200):ticks=[-200,-100,0,100,200]
        while len(ticks)>2 and (ticks[1]-ticks[0])*rect.height()/(high-low)<16:
            ticks=ticks[::2]
        for tick in ticks:
            y = rect.bottom()-(tick-low)*rect.height()/(high-low)
            p.setPen(QPen(QColor(colors['line']), 1))
            p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            p.setPen(QColor(colors['muted']))
            p.drawText(QRectF(1, y-9, 46, 18), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       tick_label(tick))
        # Anchor ticks to elapsed time, so labels and grid lines move with the
        # waveform rather than staying at fixed viewport fractions.
        for tick in axis_ticks(begin, end, angle=False):
            x = rect.left()+(tick-begin)*rect.width()/self.window
            p.setPen(QPen(QColor(colors['line']), 1, Qt.PenStyle.DashLine))
            p.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            p.setPen(QColor(colors['muted']))
            text = f'{tick_label(tick)}s'
            width = p.fontMetrics().horizontalAdvance(text) + 8
            left = max(0, min(x-width/2, self.width()-width))
            p.drawText(QRectF(left, rect.bottom()+5, width, 18), Qt.AlignmentFlag.AlignCenter, text)
        p.save()
        p.setClipRect(rect)
        if self.fast:
            # Only the speed panel uses cosmetic polylines. Original signal
            # and pose charts retain their antialiasing, width and seam paths.
            p.setRenderHint(QPainter.RenderHint.Antialiasing,False)
            for key, _, color in self.series:
                pen=QPen(QColor(colors.get(color,color)),1);pen.setCosmetic(True);p.setPen(pen)
                for points in trace_segments(rows,key,False):
                    polygon=QPolygonF([QPointF(rect.left()+(t-begin)*rect.width()/self.window,
                                             rect.bottom()-(value-low)*rect.height()/(high-low)) for t,value in points])
                    if len(points)>1:p.drawPolyline(polygon)
                    elif points:p.drawPoint(polygon[0])
        else:
            for key, _, color in self.series:
                path = QPainterPath()
                p.setPen(QPen(QColor(colors.get(color, color)), 1.7))
                for points in trace_segments(rows,key,self.periodic):
                    # Clip outside the viewport by more than half the pen plus
                    # AA coverage; artificial endpoints stay behind Qt's clip.
                    margin=3*(high-low)/max(1,rect.height())
                    for visible in visible_trace_segments(points,low-margin,high+margin,self.periodic):
                        for i,(t,value) in enumerate(visible):
                            point=QPointF(rect.left()+(t-begin)*rect.width()/self.window,
                                          rect.bottom()-(value-low)*rect.height()/(high-low))
                            if i==0:
                                path.moveTo(point)
                                p.drawPoint(point)
                            else:path.lineTo(point)
                p.drawPath(path)
        occupied = [-float('inf')] * 3
        for marker in self.markers:
            if not begin <= marker['t'] <= end:
                continue
            x = rect.left() + (marker['t']-begin)*rect.width()/self.window
            marker_color = QColor(colors['amber']);marker_color.setAlphaF(.45)
            p.setPen(QPen(marker_color, 1, Qt.PenStyle.DotLine))
            p.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            if self.marker_labels:
                text = marker_caption(marker)
                p.setFont(QFont('Microsoft YaHei UI', 8))
                width = min(rect.width(), p.fontMetrics().horizontalAdvance(text) + 8)
                left = max(rect.left(), min(x+4, rect.right()-width))
                lane = next((i for i, edge in enumerate(occupied) if left > edge+4), None)
                if lane is not None:
                    box = QRectF(left, rect.top()+3+lane*20, width, 18)
                    text_color = QColor(colors['amber']);text_color.setAlphaF(.58)
                    p.setPen(text_color)
                    p.drawText(box, Qt.AlignmentFlag.AlignCenter, text)
                    occupied[lane] = box.right()
        p.restore()
        if self.marker_times:
            p.setFont(QFont('Segoe UI', 8))
            edge = -float('inf')
            for marker in self.markers:
                if not begin <= marker['t'] <= end:
                    continue
                x = rect.left()+(marker['t']-begin)*rect.width()/self.window
                color = QColor(colors['amber']);color.setAlphaF(.75)
                p.setPen(QPen(color, 1))
                p.drawLine(QPointF(x,rect.bottom()),QPointF(x,rect.bottom()+4))
                text = f"{marker['t']:.2f}s"
                width = p.fontMetrics().horizontalAdvance(text)+4
                left = max(rect.left(), min(x-width/2,rect.right()-width))
                if left > edge+4:
                    p.drawText(QRectF(left,rect.bottom()+22,width,18),Qt.AlignmentFlag.AlignCenter,text)
                    edge = left+width
        if not rows:
            p.setPen(QColor(colors['muted']))
            p.drawText(rect, Qt.AlignmentFlag.AlignCenter, '连接设备或启动演示后显示曲线')

    def mouseMoveEvent(self, event):
        end = max(self.window, self.samples[-1]['t']) if self.samples else self.window
        begin = end-self.window
        matches = [m for m in self.markers if begin <= m['t'] <= end
                   and abs(55+(m['t']-begin)*(self.width()-73)/self.window-event.position().x()) < 7]
        if matches:
            # Rich text is disabled by escaping imported label text.
            from html import escape
            text = '<br>'.join(escape(marker_detail(m)) for m in matches)
            QToolTip.showText(event.globalPosition().toPoint(), text, self)
        else:
            QToolTip.hideText()
        super().mouseMoveEvent(event)


class PendulumView(QWidget):
    def __init__(self):
        super().__init__()
        self.sample = None
        self.theme = "dark"
        self.setMinimumHeight(180)

    @timed_ui('paint.pose')
    def paintEvent(self, event):
        colors = PALETTES[self.theme]
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(colors['panel']))
        p.setPen(QColor(colors['text']))
        p.setFont(QFont('Microsoft YaHei UI', 10, QFont.Weight.DemiBold))
        p.drawText(18, 27, '倒立摆姿态')
        theta = math.radians(self.sample['theta']) if self.sample else 0
        alpha = math.radians(self.sample['alpha']) if self.sample else 0
        scale = min(w/300, (h-70)/250)
        cx, cy = w*.44, 48+(h-100)*.4
        radius = 92*scale
        p.setPen(QPen(QColor('#2c3d54'), 1, Qt.PenStyle.DashLine))
        p.drawEllipse(QRectF(cx-radius, cy-radius*.3, radius*2, radius*.6))
        p.drawLine(QPointF(cx-radius-25, cy), QPointF(cx+radius+25, cy))
        p.setPen(QPen(QColor('#526881'), 13*scale))
        p.drawLine(QPointF(cx, cy+4), QPointF(cx, cy+53*scale))
        p.setPen(QPen(QColor('#6a86a7'), 4*scale))
        p.drawLine(QPointF(cx-30*scale, cy+55*scale), QPointF(cx+30*scale, cy+55*scale))
        pivot = QPointF(cx+radius*math.cos(theta), cy-radius*.3*math.sin(theta))
        tip = QPointF(pivot.x()+90*scale*math.sin(alpha)*.65,
                      pivot.y()+90*scale*math.cos(alpha))
        p.setPen(QPen(QColor(colors['accent']), 7*scale, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(cx, cy), pivot)
        p.setPen(QPen(QColor(colors['teal'] if self.sample and self.sample['valid'] else colors['muted']), 5*scale,
                      Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(pivot, tip)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor('#d7e8fc'))
        for point in (QPointF(cx, cy), pivot):
            p.drawEllipse(point, 5*scale, 5*scale)
        p.setBrush(QColor(colors['teal'] if self.sample else colors['muted']))
        p.drawEllipse(tip, 7*scale, 7*scale)
        p.setPen(QColor(colors['muted']))
        p.setFont(QFont('Microsoft YaHei UI', 8))
        p.drawText(12, h-28, 'θ 旋臂角 · α 下垂为 0°')
        p.drawText(12, h-10, getattr(self,'source_caption','实测姿态示意') if self.sample else '等待有效测量')
        p.end()
