#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import sys
import math
from array import array 
from PyQt4 import QtCore, QtGui, QtOpenGL
from guiqwt.plot import PlotManager, CurvePlot
from guiqwt.builder import make
from serial_trans import *
import thread  



PLOT_DEFINE = [[u"Roll",u"Pitch",u"Yaw"],[u"PID_Roll"],[u"PID_Pitch"],[u"PID_Yaw"]]
COLORS = ["blue","red","black"] 
DT = 0.1
RPY_Array = [1,1,1]
Roll = 0
Pitch = 0
Yaw = 0
Angle1 = 0
Angle2 = 0
Angle3 = 0
xSlider = 0
ySlider = 0
zSlider = 0
try:
    from OpenGL.GL import *
except ImportError:
    app = QtGui.QApplication(sys.argv)
    QtGui.QMessageBox.critical(None, "OpenGL grabber",
            "PyOpenGL must be installed to run this example.")
    sys.exit(1)

from sip import setdestroyonexit
setdestroyonexit(False)

class GLWidget(QtOpenGL.QGLWidget):
    xRotationChanged = QtCore.pyqtSignal(int)
    yRotationChanged = QtCore.pyqtSignal(int)
    zRotationChanged = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)

        self.gear1 = 0
        self.gear2 = 0
        self.gear3 = 0
        self.gear4 = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0
        self.gear1Rot = 0

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.advanceGears)
        timer.start(20)

    def setXRotation(self, angle):
        self.normalizeAngle(angle)

        if angle != self.xRot:
            self.xRot = angle
            self.xRotationChanged.emit(angle)
            self.updateGL()

    def setYRotation(self, angle):
        self.normalizeAngle(angle)

        if angle != self.yRot:
            self.yRot = angle
            self.yRotationChanged.emit(angle)
            self.updateGL()

    def setZRotation(self, angle):
        self.normalizeAngle(angle)

        if angle != self.zRot:
            self.zRot = angle
            self.zRotationChanged.emit(angle)
            self.updateGL()

    def initializeGL(self):
        lightPos = (5.0, 5.0, 10.0, 1.0)
        #reflectance1 = (0.8, 0.1, 0.0, 1.0)#颜色
        #reflectance2 = (0.0, 0.8, 0.2, 1.0)
        reflectance3 = (0.2, 0.2, 1.0, 1.0)

        glLightfv(GL_LIGHT0, GL_POSITION, lightPos)#设置光源在场景中的位置
        glEnable(GL_LIGHTING)#启用光照
        glEnable(GL_LIGHT0)#启用光
        glEnable(GL_DEPTH_TEST)#剔除隐藏面
                                   #内径外径，厚度，齿深，齿数
        #self.gear1 = self.makeGear(reflectance1, 1.0, 4.0, 1.0, 0.7, 20)
        #self.gear2 = self.makeGear(reflectance2, 0.5, 2.0, 2.0, 0.7, 10)
        #self.gear3 = self.makeGear(reflectance3, 1.3, 2.0, 0.5, 0.7, 10)
        self.gear4 = self.makeGear(reflectance3, 0.01, 2.5, 0.4,0.03, 20)
        glEnable(GL_NORMALIZE)
        glClearColor(0.0, 0.0, 0.0, 1.0)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)#清理颜色/深度缓存

        glPushMatrix()#保存变换后位置角度
        glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)#鼠标拖动的变换
        glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)
        glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)
                    #self, gear, dx, dy, dz, angle
        #self.drawGear(self.gear1, -3.0, -2.0, 0.0, self.gear1Rot / 16.0)
        #self.drawGear(self.gear2, +3.1, -2.0, 0.0,
        #        -2.0 * (self.gear1Rot / 16.0) - 9.0)

        glRotated(+90.0, 1.0, 0.0, 0.0)
        self.drawGear(self.gear4, -3.1, 3.1, -2.0,
                +2.0 * (self.gear1Rot / 16.0) - 2.0)
        self.drawGear(self.gear4, -3.1, -3.1, -2.0,
                +2.0 * (self.gear1Rot / 16.0) - 2.0)
        self.drawGear(self.gear4, 3.1, -3.1, -2.0,
                +2.0 * (self.gear1Rot / 16.0) - 2.0)
        self.drawGear(self.gear4, 3.1, 3.1, -2.0,
                +2.0 * (self.gear1Rot / 16.0) - 2.0)
            #向下z为正 向左x负 向进y正
        #############
        glBegin(GL_QUADS)    #顶面
        #glColor3f(1.0,1.0,1.0)
        glNormal3f(0.0,1.0,0.0)
        glVertex3f(1.2,1.2,1.2)
        glVertex3f(1.2,1.2,-1.2)
        glVertex3f(-1.2,1.2,-1.2)
        glVertex3f(-1.2,1.2,1.2)
        glEnd()
        glBegin(GL_QUADS)    #底面
        glNormal3f(0.0,-1.0,0.0)
        glVertex3f(1.2,-1.2,1.2)
        glVertex3f(-1.2,-1.2,1.2)
        glVertex3f(-1.2,-1.2,-1.2)
        glVertex3f(1.2,-1.2,-1.2)
        glEnd()
        glBegin(GL_QUADS)    #前面
        glNormal3f(0.0,0.0,1.0)
        glVertex3f(1.2,1.2,1.2)
        glVertex3f(-1.2,1.2,1.2)
        glVertex3f(-1.2,-1.2,1.2)
        glVertex3f(1.2,-1.2,1.2)
        glEnd()
        glBegin(GL_QUADS)    #背面
        glNormal3f(0.0,0.0,-1.0)
        glVertex3f(1.2,1.2,-1.2)
        glVertex3f(1.2,-1.2,-1.2)
        glVertex3f(-1.2,-1.2,-1.2)
        glVertex3f(-1.2,1.2,-1.2)
        glEnd()
        glBegin(GL_QUADS)    #左面
        glNormal3f(-1.0,0.0,0.0)
        glVertex3f(-1.2,1.2,1.2)
        glVertex3f(-1.2,1.2,-1.2)
        glVertex3f(-1.2,-1.2,-1.2)
        glVertex3f(-1.2,-1.2,1.2)
        glEnd()
        glBegin(GL_QUADS)    #右面
        glNormal3f(1.0,0.0,0.0)
        glVertex3f(1.2,1.2,1.2)
        glVertex3f(1.2,-1.2,1.2)
        glVertex3f(1.2,-1.2,-1.2)
        glVertex3f(1.2,1.2,-1.2)
        glEnd()
        
        glPopMatrix()#恢复位置角度 

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        glViewport((width - side) // 2, (height - side) // 2, side, side)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-1.0, +1.0, -1.0, 1.0, 5.0, 60.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()#复位旋转角度计数器
        glTranslated(0.0, 0.0, -40.0)

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if event.buttons() & QtCore.Qt.LeftButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setYRotation(self.yRot + 8 * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setZRotation(self.zRot + 8 * dx)

        self.lastPos = event.pos()

    def advanceGears(self):
        self.gear1Rot += 2 * 16 #齿轮旋转速度
        self.updateGL()    

    def xRotation(self):
        return self.xRot

    def yRotation(self):
        return self.yRot

    def zRotation(self):
        return self.zRot

    def makeGear(self, reflectance, innerRadius, outerRadius, thickness, toothSize, toothCount):
        list = glGenLists(1)
        glNewList(list, GL_COMPILE)
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance)

        r0 = innerRadius
        r1 = outerRadius - toothSize / 2.0
        r2 = outerRadius + toothSize / 2.0
        delta = (2.0 * math.pi / toothCount) / 4.0
        z = thickness / 2.0

        glShadeModel(GL_FLAT)

        for i in range(2):
            if i == 0:
                sign = +1.0
            else:
                sign = -1.0

            glNormal3d(0.0, 0.0, sign)

            glBegin(GL_QUAD_STRIP)

            for j in range(toothCount+1):
                angle = 2.0 * math.pi * j / toothCount
                glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), sign * z)
                glVertex3d(r1 * math.cos(angle), r1 * math.sin(angle), sign * z)
                glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), sign * z)
                glVertex3d(r1 * math.cos(angle + 3 * delta), r1 * math.sin(angle + 3 * delta), sign * z)

            glEnd()

            glBegin(GL_QUADS)

            for j in range(toothCount):
                angle = 2.0 * math.pi * j / toothCount                
                glVertex3d(r1 * math.cos(angle), r1 * math.sin(angle), sign * z)
                glVertex3d(r2 * math.cos(angle + delta), r2 * math.sin(angle + delta), sign * z)
                glVertex3d(r2 * math.cos(angle + 2 * delta), r2 * math.sin(angle + 2 * delta), sign * z)
                glVertex3d(r1 * math.cos(angle + 3 * delta), r1 * math.sin(angle + 3 * delta), sign * z)

            glEnd()

        glBegin(GL_QUAD_STRIP)

        for i in range(toothCount):
            for j in range(2):
                angle = 2.0 * math.pi * (i + (j / 2.0)) / toothCount
                s1 = r1
                s2 = r2

                if j == 1:
                    s1, s2 = s2, s1

                glNormal3d(math.cos(angle), math.sin(angle), 0.0)
                glVertex3d(s1 * math.cos(angle), s1 * math.sin(angle), +z)
                glVertex3d(s1 * math.cos(angle), s1 * math.sin(angle), -z)

                glNormal3d(s2 * math.sin(angle + delta) - s1 * math.sin(angle), s1 * math.cos(angle) - s2 * math.cos(angle + delta), 0.0)
                glVertex3d(s2 * math.cos(angle + delta), s2 * math.sin(angle + delta), +z)
                glVertex3d(s2 * math.cos(angle + delta), s2 * math.sin(angle + delta), -z)

        glVertex3d(r1, 0.0, +z)
        glVertex3d(r1, 0.0, -z)
        glEnd()

        glShadeModel(GL_SMOOTH)

        glBegin(GL_QUAD_STRIP)

        for i in range(toothCount+1):
            angle = i * 2.0 * math.pi / toothCount
            glNormal3d(-math.cos(angle), -math.sin(angle), 0.0)
            glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), +z)
            glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), -z)

        glEnd()

        glEndList()

        return list    

    def drawGear(self, gear, dx, dy, dz, angle):
        glPushMatrix()
        glTranslated(dx, dy, dz)
        glRotated(angle, 0.0, 0.0, 1.0)
        glCallList(gear)
        glPopMatrix()

    def normalizeAngle(self, angle):
        while (angle < 0):
            angle += 360 * 16

        while (angle > 360 * 16):
            angle -= 360 * 16
            
#########################################

def get_data():
    global RPY_Array,Roll,Pitch,Yaw,Angle1,Angle2,Angle3
    global serial_com3
    try:
        serial_com3 = serial.Serial('com8',115200)
        print ("connected")
    except:
        print("serial connect failed.")
    Re_buf = []
    ucStr = []
    counter = 0
    Roll = 0
    Pitch = 0
    Yaw = 0
    Angle1 = 0
    Angle2 = 0
    Angle3 = 0
    for i in range(28):
        Re_buf.append(0)
        ucStr.append(0)
    
    while(1):
        
        Re_buf = serial_com3.readline()
        
        Angle3 = ''.join(Re_buf[1:61])
        #print Re_buf,
        #Re_buf[counter] = str(n)
        
        Roll = float(''.join(Re_buf[1:11]))
        
        Pitch = float(''.join(Re_buf[11:21]))
        
        Yaw = float(''.join(Re_buf[21:31]))
        
        Angle1 = float(''.join(Re_buf[31:41]))
        
        Angle2 = float(''.join(Re_buf[41:51]))
        
        Angle3 = float(''.join(Re_buf[51:61]))
        print "\n",
        print Roll,Pitch,Yaw,Angle1,Angle2,Angle3
        
        #print "\n",
        #print Roll,Pitch,Yaw,Angle1,Angle2,Angle3
                

class WorkThread(QtCore.QThread):   
    def __int__(self):  
        super(WorkThread,self).__init__()  
  
    def run(self):  
        
        get_data()

class SyncXAxis(QtGui.QWidget):
    
    def __init__(self):
        super(SyncXAxis, self).__init__()
        
        self.data = {u"t":array("d")} 
        for name in sum(PLOT_DEFINE, []): 
            self.data[name] = array("d") 
        
        self.i = 0
        self.x = []
        self.curves = {} 
        self.t = 0
        self.sint = []
        self.get_Roll = []
        self.get_Pitch = []
        self.get_Yaw = []
        self.get_Angle1 = []
        self.get_Angle2 =[]
        self.get_Angle3 = []
        vbox = QtGui.QGridLayout()
        #工具栏
        vbox.addLayout(self.setup_toolbar(),0,0) 
        self.manager = PlotManager(self)
        self.plots = []
        #生成竖直排列图形窗口
        for i, define in enumerate(PLOT_DEFINE): 
            plot = CurvePlot() 
            plot.axisScaleDraw(CurvePlot.Y_LEFT).setMinimumExtent(60) 
            self.manager.add_plot(plot) 
            self.plots.append(plot) 
            plot.plot_id = id(plot) 
            for j, curve_name in enumerate(define): 
                curve = self.curves[curve_name] = make.curve([0], [0], color=COLORS[j], title=curve_name) 
                plot.add_item(curve) 
            plot.add_item(make.legend("BL")) 
            #vbox.addWidget(plot)
        vbox.addWidget(self.plots[0],1,0)
        vbox.addWidget(self.plots[1],1,1)
        vbox.addWidget(self.plots[2],2,0)
        vbox.addWidget(self.plots[3],2,1)
        
        self.manager.register_standard_tools()
        self.manager.get_default_tool().activate()
        self.manager.synchronize_axis(CurvePlot.X_BOTTOM, self.manager.plots.keys()) 
        self.setLayout(vbox)
        
        self.startTimer(20) 
        
    def setup_toolbar(self): 
        toolbar = QtGui.QGridLayout() 
        self.auto_xrange_checkbox = QtGui.QCheckBox(u"X轴自动调节") 
        self.xrange_box = QtGui.QSpinBox() 
        self.xrange_box.setMinimum(5) 
        self.xrange_box.setMaximum(100) 
        self.xrange_box.setValue(50) 
        self.auto_xrange_checkbox.setChecked(True) 
        
        self.Roll_label = QtGui.QLabel("PID KP:")
        self.lineEdit1 = QtGui.QLineEdit()
        self.lineEdit1.setText("1")
        self.lineEdit1.returnPressed.connect(self.PID_Roll)
        
        self.Roll_label.setBuddy(self.lineEdit1)
        self.Pitch_label = QtGui.QLabel("PID KI:")
        self.lineEdit2 = QtGui.QLineEdit()
        self.Pitch_label.setBuddy(self.lineEdit2)
        self.lineEdit2.setText("1")
        self.lineEdit2.returnPressed.connect(self.PID_Pitch)
        
        self.Yaw_label = QtGui.QLabel("PID KD:")
        self.lineEdit3 = QtGui.QLineEdit()
        self.Yaw_label.setBuddy(self.lineEdit3)
        self.lineEdit3.setText("1")
        self.lineEdit3.returnPressed.connect(self.PID_Yaw)
        
        toolbar.addWidget(self.auto_xrange_checkbox,0,0) 
        toolbar.addWidget(self.xrange_box,0,1) 
        toolbar.addWidget(self.Roll_label,1,0) 
        toolbar.addWidget(self.lineEdit1,1,1) 
        toolbar.addWidget(self.Pitch_label,2,0) 
        toolbar.addWidget(self.lineEdit2,2,1) 
        toolbar.addWidget(self.Yaw_label,3,0) 
        toolbar.addWidget(self.lineEdit3,3,1) 
        return toolbar 
        
    
    def PID_Roll(self):
        global RPY_Array
        try:
            #print str(int(self.lineEdit1.text()))
            RPY_Array[0] = int(self.lineEdit1.text())
        except:
            pass
            
    def PID_Pitch(self):
        global RPY_Array
        try:
            #print str(self.lineEdit1.text())
            RPY_Array[1] = int(self.lineEdit2.text())
        except:
            pass
            
    def PID_Yaw(self):
        global RPY_Array
        try:
            #print str(self.lineEdit1.text())
            RPY_Array[2] = int(self.lineEdit3.text())
        except:
            pass
        
    def timerEvent(self, event): 
        global RPY_Array,Roll,Pitch,Yaw,Angle1,Angle2,Angle3
        global xSlider,ySlider,zSlider
        self.x.append(self.t)
        self.t += DT 
        self.sint.append(np.sin(self.t))
        self.get_Roll.append(Roll)
        self.get_Pitch.append(Pitch)
        self.get_Yaw.append(Yaw)
        self.get_Angle1.append(Angle1)
        self.get_Angle2.append(Angle2)
        self.get_Angle3.append(Angle3)
        xSlider.setValue(Angle1 * 16)#初始值
        ySlider.setValue(Angle3 * 16)
        zSlider.setValue(Angle2 * 16)
        #x轴的移动
        if self.auto_xrange_checkbox.isChecked(): 
            xmax = self.x[-1] 
            xmin = self.xrange_box.value()
            if len(self.x) > xmin:
                
                self.x = self.x[-xmin:-1]
                self.get_Roll = self.get_Roll[-xmin:-1]
                self.get_Pitch = self.get_Pitch[-xmin:-1]
                self.get_Yaw = self.get_Yaw[-xmin:-1]
                self.get_Angle1 = self.get_Angle1[-xmin:-1]
                self.get_Angle2 = self.get_Angle2[-xmin:-1]
                self.get_Angle3 = self.get_Angle3[-xmin:-1]
        else :
            pass
        self.y_array = [[],[],[],[],[],[]]
        #3 最后一个
        #0 三
        #2 二
        self.y_array[2] = self.get_Roll
        self.y_array[0] = self.get_Pitch
        self.y_array[3] = self.get_Yaw
        self.y_array[1] = self.get_Angle1
        self.y_array[4] = self.get_Angle2
        self.y_array[5] = self.get_Angle3
        #print "\n",
        #print Roll,Pitch,Yaw,Angle1,Angle2,Angle3
        #发送PID参数
        global RPY_Array
        #serial_send(str("%04d" %RPY_Array[0]) + str("%04d" %RPY_Array[1]) + str("%04d" %RPY_Array[2]) + "111111")
        
        #更新数据 刷新图像
        self.i = 0
        for key, curve in self.curves.iteritems(): 
            #self.sint = self.sint + 1
            #self.sint = [x + 0.1 for x in self.sint]
            #print type(self.x),type(self.t)
            try:
                curve.set_data(self.x,self.y_array[self.i])
            except:
                pass
            #print self.i
            #print key
            self.i += 1
            
            
        for plot in self.plots: 
            if self.auto_xrange_checkbox.isChecked(): 
                plot.do_autoscale() 
            elif self.auto_xrange_checkbox.isChecked(): 
                plot.set_axis_limits("bottom", xmin, xmax) 
                plot.replot() 
            else: 
                plot.replot() 
        #print self.xrange_box.value(),np.sin(self.t)
        
class MainWindow(QtGui.QMainWindow):
    def __init__(self):        
        super(MainWindow, self).__init__()
        global xSlider,ySlider,zSlider
        centralWidget = QtGui.QWidget()
        self.setCentralWidget(centralWidget)
        
        self.glWidget = GLWidget()
        self.glWidgetArea = QtGui.QScrollArea()
        self.glWidgetArea.setWidget(self.glWidget)
        self.glWidgetArea.setWidgetResizable(True)
        self.glWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored,
                QtGui.QSizePolicy.Ignored)
        self.glWidgetArea.setMinimumSize(300, 300)
        #self.glWidgetArea.setSizePolicy(400, 500)
        self.glWidgetArea.setFixedSize( 660, 660 )
        xSlider = self.createSlider(self.glWidget.xRotationChanged,
                self.glWidget.setXRotation)
        ySlider = self.createSlider(self.glWidget.yRotationChanged,
                self.glWidget.setYRotation)
        zSlider = self.createSlider(self.glWidget.zRotationChanged,
                self.glWidget.setZRotation)
        
        #self.createActions() #快捷键
        centralLayout = QtGui.QGridLayout()
        centralLayout.addWidget(self.glWidgetArea, 0, 1,1,1)#代表位置
        self.SyncXAxis = SyncXAxis()
        
        centralLayout.addWidget(self.SyncXAxis, 0, 0)
        centralLayout.addWidget(xSlider, 1, 1, 1, 1)#添加组件
        centralLayout.addWidget(ySlider, 2, 1, 1, 1)#控件名，行，列，占用行数，占用列数，对齐方式
        centralLayout.addWidget(zSlider, 3, 1, 1, 1)
        
        centralWidget.setLayout(centralLayout)
        xSlider.setValue(15 * 16)#初始值
        ySlider.setValue(345 * 16)
        zSlider.setValue(0 * 16)
        
        self.setWindowTitle("Quadcopter PID")
        self.resize(1520, 700)
        
        
        centralWidget.setLayout(centralLayout)
        
    def createSlider(self, changedSignal, setterSlot):
        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, 360 * 16)
        slider.setSingleStep(16)
        slider.setPageStep(15 * 16)
        slider.setTickInterval(15 * 16)
        slider.setTickPosition(QtGui.QSlider.TicksRight)

        slider.valueChanged.connect(setterSlot)
        changedSignal.connect(slider.setValue)

        return slider

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    mainWin = MainWindow()
    
    
    workThread=WorkThread()  
    workThread.start() 
    mainWin.show()
    sys.exit(app.exec_())    