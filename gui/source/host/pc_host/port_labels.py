"""Friendly labels from present exact USB instances; COM values stay unchanged."""
import sys

KNOWN={r'USB\VID_1A86&PID_7523\5&6E75075&0&9':'FPGA 通信口',
       r'USB\VID_2E3C&PID_5740\1149B7C24055':'驱动电源（非通信口）'}

def present_port_identities():
    if sys.platform!='win32':return {}
    import ctypes as c
    from serial.tools import list_ports_windows as w
    guids=(w.GUID*8)();count=w.DWORD();result={}
    if not w.SetupDiClassGuidsFromName('Ports',guids,c.sizeof(guids),c.byref(count)):return result
    for guid in guids[:min(count.value,8)]:
        handle=w.SetupDiGetClassDevs(c.byref(guid),None,w.NULL,w.DIGCF_PRESENT)
        try:
            info=w.SP_DEVINFO_DATA();info.cbSize=c.sizeof(info);index=0
            while w.SetupDiEnumDeviceInfo(handle,index,c.byref(info)):
                index+=1;identity=c.create_unicode_buffer(512)
                if not w.SetupDiGetDeviceInstanceId(handle,c.byref(info),identity,len(identity),None):continue
                key=w.SetupDiOpenDevRegKey(handle,c.byref(info),w.DICS_FLAG_GLOBAL,0,w.DIREG_DEV,w.KEY_READ)
                try:
                    port=c.create_unicode_buffer(256);size=w.ULONG(c.sizeof(port))
                    if w.RegQueryValueEx(key,'PortName',None,None,c.byref(port),c.byref(size))==0:
                        result.setdefault(port.value.upper(),[]).append(identity.value.upper())
                finally:w.RegCloseKey(key)
        finally:w.SetupDiDestroyDeviceInfoList(handle)
    return result

def friendly_ports(ports,identities):
    counts={p.device:sum(q.device==p.device for q in ports) for p in ports}
    for port in ports:
        ids=identities.get(port.device.upper(),[])
        role=KNOWN.get(ids[0].upper()) if len(ids)==1 and counts[port.device]==1 else None
        yield (port.device, f'{port.device} · {role or port.description or "串口"}',role)

def preferred_port(labelled,previous=''):
    """Keep a present user choice; otherwise select one exact FPGA instance."""
    allowed=[(device,role) for device,label,role in labelled if role!='驱动电源（非通信口）']
    if previous and sum(device.upper()==previous.upper() for device,role in allowed)==1:
        return next(device for device,role in allowed if device.upper()==previous.upper())
    matches=[device for device,role in allowed if role=='FPGA 通信口']
    return matches[0] if len(matches)==1 else ''
