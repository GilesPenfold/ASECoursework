import numpy as np
import random
from pyngl import *

class Food():

    def __init__(self, *args, **kwargs):
        self.m_pos = kwargs.get('_pos', np.random.uniform(-25, 25, 2))
        self.m_stock = kwargs.get('_stock', random.randint(25,50))

    def Reset(self):
        self.m_pos = np.random.uniform(-25, 25, 2)
        self.m_stock = random.randint(25,50)

    def Draw(self, _camera):
        shader = ShaderLib.instance()

        t = Transformation()
        t.setScale(self.m_stock*0.05,self.m_stock*0.05,self.m_stock*0.05)
        t.setPosition(self.m_pos[0], self.m_pos[1], 0)

        M = t.getMatrix()
        MV = _camera.getViewMatrix()*M
        MVP = _camera.getVPMatrix()*M
        #normalMatrix=MV
        #normalMatrix.inverse().transpose()

        shader.setUniform("MVP", MVP)
        #shader.setUniform("normalMatrix", normalMatrix)

        prim = VAOPrimitives.instance()
        prim.draw("disc")
