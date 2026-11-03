"""Bounded UI-thread timing only: no IO, callbacks, timers or heartbeat writes."""
from collections import deque
from functools import wraps
import threading
import time


class UiTiming:
    def __init__(self,limit=256,slow_s=.01):
        self.limit=limit;self.slow_s=slow_s;self.slow=deque(maxlen=limit)
        self.stats={};self.active=[];self.total=0;self.slow_total=0
        self.phase='unmarked';self.phase_stats={};self.phase_marks=deque(maxlen=16)
        self.live={}

    def mark_phase(self,phase):
        if phase not in ('startup','replay','shutdown'):raise ValueError('Unknown timing phase')
        if self.active:raise ValueError('Cannot change timing phase inside an observed callback')
        self.phase=phase
        mark=dict(phase=phase,at=time.monotonic());self.phase_marks.append(mark)
        return dict(mark)

    def begin(self,stage):
        item=dict(stage=stage,phase=self.phase,started=time.monotonic(),cpu_started=time.thread_time(),thread=threading.get_ident())
        self.active.append(item)
        self.live=dict(stage=stage,started=item['started'],thread=item['thread'])
        return item

    def end(self,item):
        wall=time.monotonic()-item['started'];cpu=time.thread_time()-item['cpu_started']
        self.active.pop();self.total+=1
        self.live=dict(stage=self.active[-1]['stage'] if self.active else 'idle',last_stage=item['stage'],last_wall_s=wall,last_cpu_s=cpu,at=item['started']+wall)
        event=dict(stage=item['stage'],phase=item['phase'],started=item['started'],ended=item['started']+wall,
                   wall_s=wall,cpu_s=cpu,thread=item['thread'],context=dict(item.get('context',{})))
        for table in (self.stats,self.phase_stats.setdefault(item['phase'],{})):
            stat=table.setdefault(item['stage'],dict(count=0,total_wall_s=0.,max_wall_s=0.,max_cpu_s=0.))
            stat['count']+=1;stat['total_wall_s']+=wall
            if 'worst_wall' not in stat or wall>stat['max_wall_s']:stat['worst_wall']=event
            if 'worst_cpu' not in stat or cpu>stat['max_cpu_s']:stat['worst_cpu']=event
            stat['max_wall_s']=max(stat['max_wall_s'],wall);stat['max_cpu_s']=max(stat['max_cpu_s'],cpu)
        if wall>=self.slow_s:
            self.slow_total+=1
            self.slow.append(event)

    def snapshot(self):
        # Called by the UI-thread diagnostic at an explicit boundary, not every
        # pump. Deep copies also isolate retained worst-call context dictionaries.
        from copy import deepcopy
        return dict(clock='time.monotonic / time.thread_time',total=self.total,slow_total=self.slow_total,
                    retained=len(self.slow),limit=self.limit,slow_threshold_s=self.slow_s,
                    stats=deepcopy(self.stats),phase=self.phase,phase_stats=deepcopy(self.phase_stats),
                    phase_marks=deepcopy(list(self.phase_marks)),
                    active=deepcopy(self.active),slow=deepcopy(list(self.slow)))


def timed_ui(stage):
    def decorate(function):
        @wraps(function)
        def measured(self,*args,**kwargs):
            timing=getattr(self,'ui_timing',None)
            if timing is None:return function(self,*args,**kwargs)
            name=stage+getattr(self,'ui_timing_name','')
            item=timing.begin(name)
            if stage=='paint.chart':
                item['context']=dict(samples=len(self.samples),window_s=self.window,series=len(self.series),
                                     width=self.width(),height=self.height(),periodic=self.periodic,fast=self.fast)
            if stage=='poll':item['context']=dict(queued_before=self.events.qsize())
            try:return function(self,*args,**kwargs)
            finally:
                if stage=='poll':item['context'].update(processed=getattr(self,'poll_event_count',0),queued_after=self.events.qsize())
                timing.end(item)
        return measured
    return decorate
