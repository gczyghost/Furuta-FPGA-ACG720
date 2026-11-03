"""Display-only command captions. Sending is not device acceptance."""
import math
import re

def marker_caption(marker):
    match=re.fullmatch(r'F1 ([0-9A-Fa-f]+):(\d+) (-?\d+)',marker.get('command',''))
    if not match:return marker.get('label','指令').replace(' · 已发送','').replace('已发送','').strip() or '指令'
    op,addr,value=int(match[1],16),int(match[2]),int(match[3])
    if value>=2**31:value-=2**32
    if op==2:return 'STOP'
    if op==0x26:return '退出伺服' if addr==1 else f'{value}rpm'
    if op==0x2b:return f'{value}rpm轨迹'
    if op in (0x24,0x27):return '回零' if value==0 else f'定位{math.degrees(value/65536):g}°'
    if op==0x25:return f'{math.degrees(value/65536):+.1f}°'
    return {3:'清故障',0x11:'写标定',0x20:'使能',0x21:'校准',0x22:'落摆',0x29:'起摆',0x2a:'复位'}.get(op,'指令')

def marker_detail(marker):
    return f"{marker['t']:.2f}s · 请求 {marker_caption(marker)} · {marker.get('command','')}"
