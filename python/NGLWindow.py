#!/usr/bin/env python
from PyQt5.QtGui import QOpenGLWindow,QSurfaceFormat
from PyQt5.QtWidgets import *
from  PyQt5.QtCore import *
import sys
from pyngl import *
# THE FOLLOWING TWO LINES SHOULD PROBABLY BE DISABLED WHEN DOING INDEPTH DEBUGGING
import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *
import flock
import time

updateframes = 5



class NGLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super(NGLWidget, self).__init__(parent)
        self.cam = Camera()
        self.mouseGlobalTX = Mat4()
        self.width = 1024
        self.height = 720
        self.setWindowTitle('Genetic Boids - Generation: ' + str(0) + 'MiniGen: ' + str(0) + '| ' + str(0) + '/' + str(0))
        self.spinXFace = 0
        self.spinYFace = 0
        self.rotate = False
        self.translate = False
        self.origX = 0
        self.origY = 0
        self.origXPos = 0
        self.origYPos = 0
        self.INCREMENT = 0.01
        self.ZOOM = 0.1
        self.modelPos = Vec3()
        self.flock = flock.Flock()

        self.flock.AddFood(7)
        self.startTimer(updateframes)
        self.start = False

    def initializeGL(self):
        self.makeCurrent()
        NGLInit.instance()
        glClearColor(0.4, 0.4, 0.4, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_MULTISAMPLE)
        shader = ShaderLib.instance()
        shader.use('nglDiffuseShader')

        self.cam.set(Vec3(1, 1, -100), Vec3.zero(), Vec3.up())
        self.cam.setShape(45.0, 720.0 / 576.0, 0.05, 350.0)
        shader.setUniform("viewerPos", self.cam.getEye().toVec3())
        iv = self.cam.getViewMatrix()
        iv.transpose()
        light = Light(Vec3(-2.0, 5.0, 2.0), Colour(1.0, 1.0, 1.0, 1.0), Colour(1.0, 1.0, 1.0, 1.0),
                      LightModes.POINTLIGHT)
        light.setTransform(iv)
        light.loadToShader('light')
        prim = VAOPrimitives.instance()
        prim.createDisk("disc",0.5,8)



    def update(self):
        super(QOpenGLWidget, self).update()



    def loadMatricesToShader(self):
        shader = ShaderLib.instance()

        normalMatrix = Mat3()
        M = self.mouseGlobalTX
        MV = self.cam.getViewMatrix() * M
        MVP = self.cam.getVPMatrix() * M
        #normalMatrix = Mat3(MV)
        #normalMatrix.inverse().transpose()
        shader.setUniform("MV", MV)
        shader.setUniform("MVP", MVP)
        #shader.setUniform("normalMatrix", normalMatrix)
        shader.setUniform("M", M)

    def paintGL(self):
        glViewport(0, 0, self.width, self.height)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        shader = ShaderLib.instance()

        shader.use('nglColourShader')
        shader.setUniform('Colour',1.0,0.0,0.0,1.0)
        if self.start == True:
            self.flock.Draw(self.cam, shader)



    def resizeGL(self, w, h):
        self.width = int(w * self.devicePixelRatio())
        self.height = int(h * self.devicePixelRatio())
        self.cam.setShape(45.0, float(w) / h, 0.05, 350.0)

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Escape:
            exit()
        elif key == Qt.Key_W:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        elif key == Qt.Key_S:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        elif key == Qt.Key_Space:
            self.spinXFace = 0
            self.spinYFace = 0
            self.modelPos.set(Vec3.zero())

        self.update()

    def mouseMoveEvent(self, event):
        if self.rotate and event.buttons() == Qt.LeftButton:
            diffx = event.x() - self.origX
            diffy = event.y() - self.origY
            self.spinXFace += int(0.5 * diffy)
            self.spinYFace += int(0.5 * diffx)
            self.origX = event.x()
            self.origY = event.y()
            self.update()

        elif self.translate and event.buttons() == Qt.RightButton:

            diffX = int(event.x() - self.origXPos)
            diffY = int(event.y() - self.origYPos)
            self.origXPos = event.x()
            self.origYPos = event.y()
            self.modelPos.m_x += self.INCREMENT * diffX
            self.modelPos.m_y -= self.INCREMENT * diffY
            self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.origX = event.x()
            self.origY = event.y()
            self.rotate = True

        elif event.button() == Qt.RightButton:
            self.origXPos = event.x()
            self.origYPos = event.y()
            self.translate = True

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.rotate = False

        elif event.button() == Qt.RightButton:
            self.translate = False

    def wheelEvent(self, event):
        numPixels = event.pixelDelta()

        if numPixels.x() > 0:
            self.modelPos.m_z += self.ZOOM

        elif numPixels.x() < 0:
            self.modelPos.m_z -= self.ZOOM
        self.update()

    def timerEvent(self, QTimerEvent):
        if self.start == True:
            start = time.time()

            self.flock.Flock()
            self.flock.Update()

            end = time.time() - start

            #print 'Time to update: ' + str(end) + 's'
            if self.flock.bestBoid == None:
                self.parentWidget().setWindowTitle('Genetic Boids - Generation: ' + str(self.flock.m_genCount) + ' MiniGen: '
                             + str(self.flock.m_miniGenCount) + ' | ' + str(self.flock.ticks)
                             + '/' + str(self.flock.maxTicks))
            else:
                self.parentWidget().setWindowTitle('Genetic Boids - Generation: ' + str(self.flock.m_genCount) + ' MiniGen: '
                          + str(self.flock.m_miniGenCount) + ' | ' + str(self.flock.ticks)
                          + '/' + str(self.flock.maxTicks) + ' Fittest Boid: ' + self.flock.bestBoid.m_name)

        self.update()

class NGLWindow(QWidget):

    def __init__(self):
        super(NGLWindow, self).__init__()

        self.glWidget = NGLWidget(parent=self)

        self.sideTabs = QTabWidget()
        self.tabOne = QGroupBox()
        self.tabTwo = QGroupBox()
        self.tabThree = QGroupBox()

        self.startButtonA = QPushButton("Start Simulation")
        self.startButtonA.clicked.connect(self.StartSim)
        self.startButtonB = QPushButton("Start Simulation")
        self.startButtonB.clicked.connect(self.StartSim)
        self.startButtonC = QPushButton("Start Simulation")
        self.startButtonC.clicked.connect(self.StartSim)

        # Tab 1 - Boids

        self.BC = QLabel("Cohesion Weight")
        self.boidCoh = QDoubleSpinBox()
        self.boidCoh.setValue(1.0)
        self.boidCoh.setMinimum(0)
        self.boidCoh.setMaximum(10)

        self.BS = QLabel("Separation Weight")
        self.boidSep = QDoubleSpinBox()
        self.boidSep.setValue(1.75)
        self.boidSep.setMinimum(0)
        self.boidSep.setMaximum(10)

        self.BA = QLabel("Alignment Weight")
        self.boidAli = QDoubleSpinBox()
        self.boidAli.setValue(1.5)
        self.boidAli.setMinimum(0)
        self.boidAli.setMaximum(10)

        self.BN = QLabel("Number of Boids")
        self.boidNum = QSpinBox()
        self.boidNum.setValue(50)
        self.boidNum.setMinimum(5)
        self.boidNum.setMaximum(75)



        self.vbox = QVBoxLayout()
        self.vbox.addWidget(self.BN)
        self.vbox.addWidget(self.boidNum)
        self.vbox.addWidget(self.BC)
        self.vbox.addWidget(self.boidCoh)
        self.vbox.addWidget(self.BS)
        self.vbox.addWidget(self.boidSep)
        self.vbox.addWidget(self.BA)
        self.vbox.addWidget(self.boidAli)
        self.vbox.addStretch(1)

        self.vbox.addWidget(self.startButtonA)
        self.tabOne.setLayout(self.vbox)

        self.sideTabs.addTab(self.tabOne, "Boids")

        # Tab 2 - Predators


        self.PN = QLabel("Number of Predators")
        self.predNum = QSpinBox()
        self.predNum.setValue(5)
        self.predNum.setMinimum(1)
        self.predNum.setMaximum(10)

        self.PAt = QLabel("Attack Weight")
        self.predAt = QDoubleSpinBox()
        self.predAt.setValue(2.0)
        self.predAt.setMinimum(0)
        self. predAt.setMaximum(10)



        self.PSi = QLabel("Sight Radius")
        self.predSig = QSpinBox()
        self.predSig.setValue(25)
        self.predSig.setMinimum(1)
        self.predSig.setMaximum(50)

        self.PSp = QLabel("Slowness")
        self.predSp = QDoubleSpinBox()
        self.predSp.setValue(0.35)
        self.predSp.setMinimum(0.01)
        self.predSp.setMaximum(1)

        # Begin Layout Design for Tab 1
        self.pbox = QVBoxLayout()
        self.pbox.addWidget(self.PN)
        self.pbox.addWidget(self.predNum)
        self.pbox.addWidget(self.PAt)
        self.pbox.addWidget(self.predAt)

        self.pbox.addWidget(self.PSi)
        self.pbox.addWidget(self.predSig)
        self.pbox.addWidget(self.PSp)
        self.pbox.addWidget(self.predSp)
        self.pbox.addStretch(1)
        self.pbox.addWidget(self.startButtonB)
        self.tabTwo.setLayout(self.pbox)

        # Add Tab to Stack

        self.sideTabs.addTab(self.tabTwo, "Predators")


        # Tab 3 - Genetic Settings

        self.GA = QLabel("Maximum Initial Awareness")
        self.genAware = QDoubleSpinBox()
        self.genAware.setValue(5.0)
        self.genAware.setMinimum(1)
        self.genAware.setMaximum(100)

        self.GS = QLabel("Maximum Initial Strength")
        self.genStr = QDoubleSpinBox()
        self.genStr.setValue(0.2)
        self.genStr.setMinimum(0.05)
        self.genStr.setMaximum(5)

        self.GTR = QLabel("Maximum Initial Tiredness Rate")
        self.genTired = QDoubleSpinBox()
        self.genTired.setValue(0.05)
        self.genTired.setMinimum(0.005)
        self.genTired.setMaximum(1)

        self.GRR = QLabel("Maximum Initial Recovery Rate")
        self.genRecov = QDoubleSpinBox()
        self.genRecov.setValue(0.01)
        self.genRecov.setMinimum(0)
        self.genRecov.setMaximum(1)

        self.gbox = QVBoxLayout()
        self.gbox.addWidget(self.GA)
        self.gbox.addWidget(self.genAware)
        self.gbox.addWidget(self.GS)
        self.gbox.addWidget(self.genStr)
        self.gbox.addWidget(self.GTR)
        self.gbox.addWidget(self.genTired)
        self.gbox.addWidget(self.GRR)
        self.gbox.addWidget(self.genRecov)
        self.gbox.addStretch(1)
        self.gbox.addWidget(self.startButtonC)
        self.tabThree.setLayout(self.gbox)

        self.sideTabs.addTab(self.tabThree, "Genetics")

        self.mainLayout = QHBoxLayout()
        self.mainLayout.addWidget(self.glWidget)

        self.mainLayout.addWidget(self.sideTabs)
        self.mainLayout.setStretch(0, 3)
        self.mainLayout.setStretch(1, 1)
        self.setLayout(self.mainLayout)



        self.setWindowTitle(
            'Genetic Boids - Generation: ' + str(0) + 'MiniGen: ' + str(0) + '| ' + str(0) + '/' + str(0))

    def StartSim(self):
        self.glWidget.flock.startTired = self.genTired.value().__float__()
        self.glWidget.flock.startRecov = self.genRecov.value().__float__()
        self.glWidget.flock.startStr = self.genStr.value().__float__()
        self.glWidget.flock.startAware = self.genAware.value().__float__()
        self.glWidget.flock.boidCohesion = self.boidCoh.value().__float__()
        self.glWidget.flock.boidSeparation = self.boidSep.value().__float__()
        self.glWidget.flock.boidAlignment = self.boidAli.value().__float__()
        self.glWidget.flock.predSig = self.predSig.value().__float__()
        self.glWidget.flock.predSpeed = self.predSp.value().__float__()
        self.glWidget.flock.predAtt = self.predAt.value().__float__()

        self.glWidget.flock.AddBoid(self.boidNum.value().__int__())
        self.glWidget.flock.AddPredator(self.predNum.value().__int__())

        self.startButtonA.deleteLater()
        self.startButtonB.deleteLater()
        self.startButtonC.deleteLater()

        self.glWidget.start=True

if __name__ == '__main__':
  app = QApplication(sys.argv)
  format=QSurfaceFormat()
  format.setSamples(4)
  format.setMajorVersion(4)
  format.setMinorVersion(1)
  format.setProfile(QSurfaceFormat.CoreProfile)
  # now set the depth buffer to 24 bits
  format.setDepthBufferSize(24)
  # set that as the default format for all windows
  QSurfaceFormat.setDefaultFormat(format)



  window = NGLWindow()
  #window.setFormat(format)
  window.resize(1024,720)
  window.show()


sys.exit(app.exec_())