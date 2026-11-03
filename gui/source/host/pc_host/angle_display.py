"""Display-only circular coordinates and measured-pose selection, in degrees."""
import math


class PoseSelector:
    def __init__(self):
        self.reset()

    def reset(self):
        self.pose = self.pending = self.since = self.last_t = None

    def update(self, row):
        t, alpha = row['t'], row['alpha']
        if not row.get('valid') or not math.isfinite(alpha) or not math.isfinite(t):
            self.pending = self.since = self.last_t = None
            return self.pose
        if self.last_t is not None and (t <= self.last_t or t-self.last_t > .3):
            self.pending = self.since = None
        self.last_t = t
        distance = abs(math.remainder(alpha, 360))
        if self.pose is None:
            self.pose = 'down' if distance <= 90 else 'upright'
        target = 'down' if distance <= 60 else 'upright' if distance >= 120 else self.pose
        if target == self.pose:
            self.pending = self.since = None
        elif target != self.pending:
            self.pending, self.since = target, t
        elif t-self.since >= .15:
            self.pose = target
            self.pending = self.since = None
        return self.pose


def trace_segments(rows, key, periodic=False):
    """Unwrap each valid continuous run; never interpolate missing measurements."""
    segments, points = [], []
    previous_t = previous_value = None
    for row in rows:
        t, value = row['t'], row[key]
        valid = math.isfinite(value) and math.isfinite(t) and (key not in ('alpha', 'beta') or row['valid'])
        gap = previous_t is not None and (t-previous_t > .3 or t <= previous_t)
        if not valid or gap:
            if points:segments.append(points)
            points = []
            previous_t = previous_value = None
        if not valid:continue
        y = value
        if periodic:
            y = math.remainder(value,360) if previous_value is None else points[-1][1]+math.remainder(value-previous_value,360)
        points.append((t,y));previous_t,previous_value=t,value
    if points:segments.append(points)
    return segments


def visible_offsets(points, low, high, periodic=False):
    if not periodic:return [0]
    bottom, top = min(y for _,y in points), max(y for _,y in points)
    return [360*k for k in range(math.ceil((low-top)/360),math.floor((high-bottom)/360)+1)]


def visible_trace_segments(points, low, high, periodic=False):
    """Clip each edge before building Qt paths; work does not grow with turns.

    Intersections retain the original linear interpolation. Separate input
    runs stay separate; raw samples and circular unwrapping are unchanged.
    """
    if not periodic:
        return [points]
    if len(points) < 2:
        return [[(t,y+offset) for t,y in points]
                for offset in visible_offsets(points,low,high,True)] if points else []
    runs, tails = [], {}
    for a,b in zip(points,points[1:]):
        t0,y0=a;t1,y1=b
        for k in range(math.ceil((low-max(y0,y1))/360),
                       math.floor((high-min(y0,y1))/360)+1):
            v0,v1=y0+360*k,y1+360*k
            if v0==v1:
                start,end=(t0,v0),(t1,v1)
            else:
                enter,leave=sorted(((low-v0)/(v1-v0),(high-v0)/(v1-v0)))
                enter,leave=max(0.,enter),min(1.,leave)
                start=(t0+(t1-t0)*enter,v0+(v1-v0)*enter)
                end=(t0+(t1-t0)*leave,v0+(v1-v0)*leave)
            tail=tails.get(k)
            if tail is None or tail[-1]!=start:
                tail=[start];runs.append(tail);tails[k]=tail
            tail.append(end)
    return runs
