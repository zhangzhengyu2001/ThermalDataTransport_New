"""
MFC.py
气体流量阀（MFC, Mass Flow Controller）通讯协议相关函数集合。
实现了MFC通讯协议中的CRC校验、数据打包与解析、波特率设置、地址设置、流量设置、阀门状态设置、即时流量读取等功能。
"""

def getCRC(data):
    """
    计算数据的CRC16校验码。
    参数：
        data (list): 需要校验的数据列表。
    返回：
        int: 计算得到的CRC16校验码。
    """
    CRC = 0xffff
    dataLength = len(data)
    for i in range(dataLength):
        CRC = CRC ^ (data[i] & 0x00ff)
        for j in range(8):
            carry = CRC & 0x0001
            CRC = CRC >> 1
            if carry == 0x0001:
                CRC = CRC ^ 0xA001
    return CRC

def _mergeData(data):
    """
    将字节数据合并为一个整数。
    参数：
        data (list): 字节数据列表。
    返回：
        int: 合并后的整数。
    """
    dataLength = len(data)
    message = 0
    for i in range(dataLength):
        message = (message << 8) + data[i]
    return message

def mergeData2Bytes(data):
    """
    将数据列表转换为bytes类型。
    参数：
        data (list): 数据列表。
    返回：
        bytes: 转换后的字节流。
    """
    return bytes(data)

def parseBytes2Data(returnBytes):
    """
    将bytes类型数据转换为列表。
    参数：
        returnBytes (bytes): 字节流。
    返回：
        list: 转换后的数据列表。
    """
    datalist = list(returnBytes)
    return datalist

def addCRC(data):
    """
    为数据添加CRC校验码（低字节在前，高字节在后）。
    参数：
        data (list): 原始数据列表。
    返回：
        list: 添加CRC后的数据列表。
    """
    CRC = getCRC(data)
    data.append(CRC&0x00ff)
    data.append((CRC&0xff00)>>8)
    return data

def setBaudRateSend(addr, baudRate):
    """
    生成设置波特率的命令数据。
    参数：
        addr (int): 设备地址。
        baudRate (int): 波特率（支持1200, 2400, 4800, 9600, 19200）。
    返回：
        list: 命令数据列表。
    """
    baudRates = [1200,2400,4800,9600,19200]
    assert baudRate in baudRates
    if baudRate == 1200:
        para = 0x04B0
    elif baudRate == 2400:
        para = 0x0960
    elif baudRate == 4800:
        para = 0x12C0
    elif baudRate == 9600:
        para = 0x2580
    elif baudRate == 19200:
        para = 0x4B00
    data = [addr, 0x06,0x00,0x01,(para&0xff00)>>8, para&0x00ff]
    return data

def setAddrSend(addr, newAddr):
    """
    生成设置新地址的命令数据。
    参数：
        addr (int): 当前设备地址。
        newAddr (int): 新设备地址（32~94）。
    返回：
        list: 命令数据列表。
    """
    assert newAddr >= 32
    assert newAddr <= 94
    data = [addr, 0x06,0x00,0x00,0x00, newAddr]
    return data

def setValveFlowRateSend(addr, flowRate):
    """
    生成设置阀门流量的命令数据。
    参数：
        addr (int): 设备地址。
        flowRate (float): 目标流量（0~1）。
    返回：
        list: 命令数据列表。
    """
    assert flowRate >= 0
    assert flowRate <= 1
    hexValue = 0x4000 + int(flowRate * (0xC000 - 0x4000))
    data = [addr, 0x06,0x00,0x02,(hexValue&0xff00)>>8, hexValue&0x00ff]

    return data

def setValveStatusSend(addr, status):
    """
    生成设置阀门状态的命令数据。
    参数：
        addr (int): 设备地址。
        status (int): 阀门状态（0: 关闭, 1: 打开, 2: 保持）。
    返回：
        list: 命令数据列表。
    """
    assert status in [0,1,2]
    data = [addr, 0x06,0x00,0x05,0x00, status]
    return data

def getInstantFlowRateSend(addr):
    """
    生成读取即时流量的命令数据。
    参数：
        addr (int): 设备地址。
    返回：
        list: 命令数据列表。
    """
    data = [addr, 0x03, 0x00, 0x03, 0x00, 0x01]
    return data


def checkCRC(returnData:list):
    """
    检查数据的CRC校验是否正确。
    参数：
        returnData (list): 包含CRC的数据列表。
    返回：
        bool: 校验是否通过。
    """
    calCRC = getCRC(returnData[0:-2])
    if calCRC == ((returnData[-1]<<8)+returnData[-2]):
        return True
    else:
        return False
    
def getInstantFlowRateParse(data):
    """
    解析即时流量数据。
    参数：
        data (list): 原始数据列表。
    返回：
        float: 解析后的流量值（0~1）。
    """
    flowRateList = data[3:5]
    flowRate = (flowRateList[0]<<8) + flowRateList[1]
    
    return float(flowRate - 0x4000)/float(0xC000 - 0x4000)
    
def parseReadMessage(message):
    """
    解析读取返回的消息，提取地址和数据。
    参数：
        message (bytes): 读取到的原始消息。
    返回：
        tuple: (设备地址, 数据值)
    """
    dataList = parseBytes2Data(message)
    assert checkCRC(dataList)
    addr = dataList[0]
    dataLength = dataList[2]
    data = dataList[3:3+dataLength]
    if dataLength == 2:
        return addr,(data[0] << 8)+data[1]
    elif dataLength <= 0:
        return addr,0
    else:
        res = data[0]
        for i in range(dataLength - 1):
            res = res << 8
            res += data[i+1]
        return addr,res
        
    
def getInstantFlowRate(ser, addr):
    """
    发送读取即时流量命令并解析返回值。
    参数：
        ser: 串口对象。
        addr (int): 设备地址。
    返回：
        tuple: (设备地址, 流量值)
        流量值取值范围为0-1，表示流量的百分比，实际值需乘量程。
    """
    sendData = [addr, 0x03, 0x00, 0x03, 0x00, 0x01]
    sendData = addCRC(sendData)
    sendMessage = mergeData2Bytes(sendData)
    ser.write(sendMessage)
    
    readMessage = ser.read(7)
    addr, _ = parseReadMessage(readMessage)
    
    data = parseBytes2Data(readMessage)
    flowRate = getInstantFlowRateParse(data)
    return addr,flowRate


def setValveFlowRate(ser, addr, flowRate):
    """
    发送设置阀门流量命令并返回回复数据。
    参数：
        ser: 串口对象。
        addr (int): 设备地址。
        flowRate (float): 目标流量（0~1）。
    返回：
        list: 回复数据列表。
    """
    sendData = setValveFlowRateSend(addr, flowRate)
    sendData = addCRC(sendData)
    sendMessage = mergeData2Bytes(sendData)
    ser.write(sendMessage)
    #写命令的回复长度与发送一致
    writeMessage = ser.read(len(sendData))
    #todo:比较发送和回写是否一致，注意addr和crc可能不同
    
    writeData = parseBytes2Data(writeMessage)
    return writeData




















