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

updateframes = 10



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
        self.flock.AddBoid(50)
        self.flock.AddPredator(5)
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
        tabOne = QGroupBox()
        tabTwo = QGroupBox()
        tabThree = QGroupBox()

        startButtonA = QPushButton("Start Simulation")
        startButtonA.clicked.connect(self.StartSim)
        startButtonB = QPushButton("Start Simulation")
        startButtonB.clicked.connect(self.StartSim)

        # Tab 1 - Boids

        BC = QLabel("Cohesion Weight")
        boidCoh = QDoubleSpinBox()
        boidCoh.setValue(1.0)
        boidCoh.setMinimum(0)
        boidCoh.setMaximum(10)

        BS = QLabel("Separation Weight")
        boidSep = QDoubleSpinBox()
        boidSep.setValue(1.75)
        boidSep.setMinimum(0)
        boidSep.setMaximum(10)

        BA = QLabel("Alignment Weight")
        boidAli = QDoubleSpinBox()
        boidAli.setValue(1.5)
        boidAli.setMinimum(0)
        boidAli.setMaximum(10)


        vbox = QVBoxLayout()
        vbox.addWidget(BC)
        vbox.addWidget(boidCoh)
        vbox.addWidget(BS)
        vbox.addWidget(boidSep)
        vbox.addWidget(BA)
        vbox.addWidget(boidAli)
        vbox.addStretch(1)


        vbox.addWidget(startButtonA)
        tabOne.setLayout(vbox)

        self.sideTabs.addTab(tabOne, "Boids")

        # Tab 2 - Predators

        PC = QLabel("Cohesion Weight (To Boids)")
        predCoh = QDoubleSpinBox()
        predCoh.setValue(2.0)
        predCoh.setMinimum(0)
        predCoh.setMaximum(10)

        PS = QLabel("Separation Weight")
        predSep = QDoubleSpinBox()
        predSep.setValue(2.0)
        predSep.setMinimum(0)
        predSep.setMaximum(10)

        PA = QLabel("Alignment Weight")
        predAli = QDoubleSpinBox()
        predAli.setValue(0.3)
        predAli.setMinimum(0)
        predAli.setMaximum(10)

        PAt = QLabel("Attack Weight")
        predAt = QDoubleSpinBox()
        predAt.setValue(2.0)
        predAt.setMinimum(0)
        predAt.setMaximum(10)

        PR = QLabel("Random Movement Weight")
        predR = QDoubleSpinBox()
        predR.setValue(0.2)
        predR.setMinimum(0)
        predR.setMaximum(10)

        PSi = QLabel("Sight Radius")
        predSig = QSpinBox()
        predSig.setValue(25)
        predSig.setMinimum(1)
        predSig.setMaximum(50)


        PSp = QLabel("Maximum Speed")
        predSp = QDoubleSpinBox()
        predSp.setValue(0.35)
        predSp.setMinimum(0.01)
        predSp.setMaximum(1)

        # Begin Layout Design for Tab 1
        pbox = QVBoxLayout()
        pbox.addWidget(PC)
        pbox.addWidget(predCoh)
        pbox.addWidget(PS)
        pbox.addWidget(predSep)
        pbox.addWidget(PA)
        pbox.addWidget(predAli)
        pbox.addWidget(PAt)
        pbox.addWidget(predAt)
        pbox.addWidget(PR)
        pbox.addWidget(predR)
        pbox.addWidget(PSi)
        pbox.addWidget(predSig)
        pbox.addWidget(PSp)
        pbox.addWidget(predSp)
        pbox.addStretch(1)
        pbox.addWidget(startButtonB)
        tabTwo.setLayout(pbox)

        # Add Tab to Stack

        self.sideTabs.addTab(tabTwo, "Predators")


        # Tab 3 - Genetic Settings

        self.sideTabs.addTab(tabThree, "Genetics")


        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.glWidget)

        mainLayout.addWidget(self.sideTabs)
        mainLayout.setStretch(0, 3)
        mainLayout.setStretch(1, 1)
        self.setLayout(mainLayout)



        self.setWindowTitle(
            'Genetic Boids - Generation: ' + str(0) + 'MiniGen: ' + str(0) + '| ' + str(0) + '/' + str(0))

    def StartSim(self):
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