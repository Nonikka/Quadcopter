#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math

from PyQt4 import QtCore, QtGui, QtOpenGL


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

            
class MainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__()
        
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
        self.glWidgetArea.setMinimumSize(100, 100)
        
        xSlider = self.createSlider(self.glWidget.xRotationChanged,
                self.glWidget.setXRotation)
        ySlider = self.createSlider(self.glWidget.yRotationChanged,
                self.glWidget.setYRotation)
        zSlider = self.createSlider(self.glWidget.zRotationChanged,
                self.glWidget.setZRotation)
        
        #self.createActions() #快捷键
        centralLayout = QtGui.QGridLayout()
        centralLayout.addWidget(self.glWidgetArea, 0, 1)#代表位置
        
        centralLayout.addWidget(xSlider, 1, 0, 1, 2)#添加组件
        centralLayout.addWidget(ySlider, 2, 0, 1, 2)#控件名，行，列，占用行数，占用列数，对齐方式
        centralLayout.addWidget(zSlider, 3, 0, 1, 2)
        
        centralWidget.setLayout(centralLayout)
        xSlider.setValue(15 * 16)#初始值
        ySlider.setValue(345 * 16)
        zSlider.setValue(0 * 16)
        
        self.setWindowTitle("Grabber")
        self.resize(400, 300)
        # connect signals
        #self.ui.some_button.connect(self.on_button)
    
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
    mainWin.show()
    sys.exit(app.exec_())    