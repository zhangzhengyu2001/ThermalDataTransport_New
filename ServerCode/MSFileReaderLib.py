from win32com.client import VARIANT as variant
import pythoncom
from win32com.client import Dispatch
import os

class SmoothingTypeEnum:
    NoSmoothing = 0
    Boxcar = 1
    Gaussian = 2

class CutoffTypeEnum:
    NoCutoff = 0
    Absolute = 1
    Relative = 2

class ControllerType:
    NoDevice = -1
    MS = 0
    Analog = 1
    ADcard = 2
    PDA = 3
    UV = 4

class MsFileReader:
    def __init__(self, filePath:str, ControllerType = ControllerType.MS, ControllerNum = 1):
        assert filePath.endswith(".raw") or filePath.endswith(".RAW")
        self.Reader = Dispatch('MSFileReader.XRawFile')
        self.Reader.open(filePath)
        self.Reader.SetCurrentController(ControllerType, ControllerNum)

    #获得整体的质谱图，横坐标质荷比，纵坐标信号绝对强度
    def GetAverageMassList(self, specStart = 1, specEnd = 0, BkgStart1 = 0, BkgEnd1 = 0, BkgStart2 = 0, BkgEnd2 = 0, filter = "", CutoffType = CutoffTypeEnum.NoCutoff, CutoffValue = 0, MaxNumPeaks = 0):
        assert specStart > 0
        if specEnd == 0:
            specEnd = self.GetNumSpectra() - 1
        assert specStart < specEnd
        #额外加2个0
        VARIANT_VT_EMPTY = variant(pythoncom.VT_EMPTY, [])
        VARIANT_VT_UI8 = variant(pythoncom.VT_UI8, 0)
        res = self.Reader.GetAverageMassList(specStart, specEnd, BkgStart1, BkgEnd1, BkgStart2, BkgEnd2, filter, CutoffType, CutoffValue, MaxNumPeaks, 0, 0, VARIANT_VT_EMPTY, VARIANT_VT_EMPTY, VARIANT_VT_UI8.value)
        mz = res[7][0]
        sig = res[7][1]
        return mz, sig, res

    #给定一个谱图序号，获得该序号对应的质谱图。横坐标质荷比，纵坐标信号绝对强度。
    #如果想要查找某一时间对应的谱图，可以用ScanNumFromRT()获得谱图序号，再调用该函数。
    def GetMassListFromScanNum(self, specNum, filter = "", CutoffType = CutoffTypeEnum.NoCutoff, CutoffValue = 0, MaxNumPeaks = 0, Centroid = 0):
        assert specNum > 0
        VARIANT_VT_EMPTY = variant(pythoncom.VT_EMPTY, [])
        VARIANT_VT_UI8 = variant(pythoncom.VT_UI8, 0)
        #额外加一个0
        res = self.Reader.GetMassListFromScanNum(specNum, filter, CutoffType, CutoffValue, MaxNumPeaks, Centroid, 0, VARIANT_VT_EMPTY, VARIANT_VT_EMPTY, VARIANT_VT_UI8.value)
        mz = res[2][0]
        sig = res[2][1]
        return mz, sig, res

    #获得EIC谱图，横坐标时间，纵坐标信号强度。需要输入一个质荷比范围以确定待展示的信号
    #ChroType1=0   Mass Range
    #ChroType1=1   TIC
    #ChroType1=2   basePeak
    def GetChroData(self, ChroType1 = 0, ChroOperator = 0, ChroType2 = 0, filter = "", MassRange1 = "", MassRange2 = "", Delay = 0.0, startTime = 0, EndTime = 0, SmoothingType = SmoothingTypeEnum.NoSmoothing, SmoothingValue = 0):
        assert startTime >= 0 and EndTime >= 0
        VARIANT_VT_EMPTY = variant(pythoncom.VT_EMPTY, [])
        VARIANT_VT_UI8 = variant(pythoncom.VT_UI8, 0)
        if startTime == 0:
            startTime = self.Reader.GetStartTime()
        if EndTime == 0:
            EndTime = self.Reader.GetEndTime()
        assert EndTime >= startTime

        #不用额外加0
        res = self.Reader.GetChroData(ChroType1, ChroOperator, ChroType2, filter, MassRange1, MassRange2, Delay, startTime, EndTime, SmoothingType, SmoothingValue, VARIANT_VT_EMPTY, VARIANT_VT_EMPTY, VARIANT_VT_UI8.value)
        _time = res[2][0]
        sig = res[2][1]
        return _time, sig, res

    #给定时间,获得对应的谱图序号
    def ScanNumFromRT(self, RT):
        assert RT >= self.Reader.GetStartTime() and RT <= self.Reader.GetEndTime()
        return self.Reader.ScanNumFromRT(RT, 0)

    def GetNumSpectra(self):
        return self.Reader.GetNumSpectra()
    
    #判断质谱序（1级质谱/2级质谱等）
     #Neutral gain
     #Neutral loss
     #Parent scan–3–2–1
     #Any scan order 0
     #MS 1
     #MS2 2
     #MS3 3
     #MS4 4
     #MS5 5
     #MS6 6
     #MS7 7
     #MS8 8
     #MS9 9
     #MS10 10
    def GetMSOrderForScanNum(self, ScanNum):
        return self.Reader.GetMSOrderForScanNum(ScanNum)
    #得到前级质谱的质量数
    def GetPrecursorMassForScanNum(self,ScanNum):
        MsOrder = self.GetMSOrderForScanNum(ScanNum)
        assert MsOrder > 0
        return self.Reader.GetPrecursorMassForScanNum(ScanNum, MsOrder)
    
    def GetNumStatusLog(self):
        return self.Reader.GetNumStatusLog(0)
    
    def GetStatusLogForScanNum(self, scanNum):
        VARIANT_VT_EMPTY = variant(pythoncom.VT_EMPTY, [])
        rt,label,data,length = self.Reader.GetStatusLogForScanNum(scanNum,0,VARIANT_VT_EMPTY,VARIANT_VT_EMPTY,0)
        return rt,label,data,length
    
    def GetStatusLogForRT(self, rt):
        VARIANT_VT_EMPTY = variant(pythoncom.VT_EMPTY, [])
        rt,label,data,length = self.Reader.GetStatusLogForRT(rt,VARIANT_VT_EMPTY, VARIANT_VT_EMPTY,0)
        return rt,label,data,length
        
        
    def GetStatusLogAtIndex(self):
        VARIANT_VT_EMPTY = variant(pythoncom.VT_EMPTY, [])
        return self.Reader.GetStatusLogAtIndex(VARIANT_VT_EMPTY, VARIANT_VT_EMPTY, VARIANT_VT_EMPTY)
    
    #0:Overall Status
    #1:Status
    #2:Performance
    #3:Ion Source
    #4:Spray Voltage (V)
    #5:Spray Current (µA)
    #6:Spray Current std. dev. (µA)
    #...
    #use GetStatusLogForRT or GetStatusLogForScanNum to see the log label
    def GetStatusLogForPos(self, pos):
        VARIANT_VT_EMPTY = variant(pythoncom.VT_EMPTY, [])
        rt,data,length = self.Reader.GetStatusLogForPos(pos,VARIANT_VT_EMPTY, VARIANT_VT_EMPTY, 0)
        return rt, data, length
        
    def GetStartTime(self):
        return self.Reader.GetStartTime()
        
    def GetEndTime(self):
        return self.Reader.GetEndTime()
    
    def GetFirstSpectrumNumber(self):
        return self.Reader.GetFirstSpectrumNumber()
    
    def GetLastSpectrumNumber(self):
        return self.Reader.GetLastSpectrumNumber()
    
    def Close(self):
        self.Reader.Close()

    def checkMassRangeValidate(massRange:str):
        splitMassRange = massRange.split("-")
        if not len(splitMassRange) == 2:
            return False,"must contain only one \"-\""
        try:
            mass1 = float(splitMassRange[0])
            mass2 = float(splitMassRange[1])
        except:
            return False,"m/z must be a number"
        if mass1 > mass2:
            return False,"last m/z must be greater than front m/z"
        
        return True,""
        
