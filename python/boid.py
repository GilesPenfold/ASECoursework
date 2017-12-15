import random
import math
from pyngl import *



class Boid() :

    def __init__(self, *args, **kwargs):
        self.m_pos = kwargs.get('_pos', Vec3())
        self.m_vel = kwargs.get('_vel', Vec3())
        self.m_acc = kwargs.get('_acc',Vec3(random.uniform(-1.0,1.0),random.uniform(-1.0,1.0),0))

        self.m_colour = kwargs.get('_colour', Vec3(1.0,1.0,1.0))

        self.m_r = kwargs.get('_r', 58.0)
        self.m_maxForce = kwargs.get('_mf', 0.05)
        self.m_maxSpeed = kwargs.get('_ms', 0.3)



    def Run(self, _boids):
        self.Flock(_boids)
        self.Update()
        self.Borders()

    def Draw(self, _camera):
        shader = ShaderLib.instance()

        t = Transformation()
        t.setPosition(self.m_pos.m_x, self.m_pos.m_y, self.m_pos.m_z)

        M = t.getMatrix()
        MV = _camera.getViewMatrix()*M
        MVP = _camera.getVPMatrix()*M
        #normalMatrix=MV
        #normalMatrix.inverse().transpose()

        shader.setUniform("MVP", MVP)
        #shader.setUniform("normalMatrix", normalMatrix)

        prim = VAOPrimitives.instance()
        prim.draw("sphere")

    def Update(self):
        self.m_vel = Vec3(self.m_vel.m_x + self.m_acc.m_x,self.m_vel.m_y + self.m_acc.m_y,self.m_vel.m_z + self.m_acc.m_z)
        self.m_vel = self.Limit(self.m_vel, self.m_maxSpeed)

        if self.Magnitude(self.m_vel) == Vec3(0.0,0.0,0.0):
            self.m_vel = Vec3(self.m_maxSpeed, self.m_maxSpeed, 0.0)

        self.m_pos = Vec3(self.m_pos.m_x + self.m_vel.m_x ,self.m_pos.m_y + self.m_vel.m_y ,self.m_pos.m_z + self.m_vel.m_z )

        self.m_acc.null()

    def Flock(self, _boids):
        S = self.Separate(_boids)
        A = self.Align(_boids)
        C = self.Cohesion(_boids)

        S = Vec3(S.m_x * 2.0, S.m_y * 2.0, S.m_z * 2.0)
        A = Vec3(A.m_x * 1.5, A.m_y * 1.5, A.m_z * 1.5)
        C = Vec3(C.m_x * 1.0, C.m_y * 1.0, C.m_z * 1.0)

        self.m_acc = Vec3(self.m_acc.m_x + S.m_x, self.m_acc.m_y + S.m_y, self.m_acc.m_z + S.m_z)
        self.m_acc = Vec3(self.m_acc.m_x + A.m_x, self.m_acc.m_y + A.m_y, self.m_acc.m_z + A.m_z)
        self.m_acc = Vec3(self.m_acc.m_x + C.m_x, self.m_acc.m_y + C.m_y, self.m_acc.m_z + C.m_z)

    def Seek(self, _target):
        self.m_acc += self.Steer(_target, False)

    def Arrive(self, _target):
        self.m_acc += self.Steer(_target, True)

    def Steer(self, _target, _slow):

        loc = Vec3(_target.m_x - self.m_pos.m_x, _target.m_y - self.m_pos.m_y, _target.m_z - self.m_pos.m_z)

        dist = self.Magnitude(loc)

        if dist > 0:
            loc = Vec3(loc.m_x/dist,loc.m_y/dist,loc.m_z/dist)
            t = self.m_maxSpeed * (dist/100.0)
            if _slow and dist < 100.0:
                loc = Vec3(loc.m_x*t,loc.m_y*t,loc.m_z*t)
            else:
                loc = Vec3(loc.m_x*self.m_maxSpeed,loc.m_y*self.m_maxSpeed,loc.m_z*self.m_maxSpeed)

            SVec = Vec3(loc.m_x - self.m_vel.m_x,loc.m_y - self.m_vel.m_y,loc.m_z - self.m_vel.m_z)
            SVec = self.Limit(SVec, self.m_maxForce)
        else:
            SVec = Vec3(0.0,0.0,0.0)

        return SVec

    def Borders(self):
        adj = 1.5
        if self.m_pos.m_x < -self.m_r-2:
            self.m_pos.m_x = self.m_r
        if self.m_pos.m_y > self.m_r/adj+1:
            self.m_pos.m_y = -self.m_r/adj
        if self.m_pos.m_x > self.m_r+2:
            self.m_pos.m_x = -self.m_r
        if self.m_pos.m_y < -self.m_r/adj-1:
            self.m_pos.m_y = self.m_r/adj

    # Rules of Flocking

    def Separate(self, _boids):

        SeparationLimit = 10.0
        _return = Vec3(0.0,0.0,0.0)
        count=0


        for b in _boids:
            dist = self.Magnitude(self.m_pos - b.m_pos)
            if dist > 0 and dist < SeparationLimit:
                diff = self.m_pos - b.m_pos
                diff = self.Normalize(diff)
                diff = Vec3(diff.m_x/dist,diff.m_y/dist,diff.m_z/dist)
                _return = Vec3(_return.m_x + diff.m_x, _return.m_y + diff.m_y,_return.m_z + diff.m_z)
                count+=1

        if count > 0:
            _return = Vec3(_return.m_x/count,_return.m_y/count,_return.m_z/count)

        return _return

    def Align(self, _boids):
        NeighbourDistance = 10.0
        _return = Vec3(0.0,0.0,0.0)
        count = 0

        for b in _boids:
            dist = self.Magnitude(self.m_pos - b.m_pos)
            if dist > 0 and dist < NeighbourDistance:
                _return += b.m_vel
                count+=1

        if count > 0:
            _return = Vec3(_return.m_x/count,_return.m_y/count,_return.m_z/count)
            _return = self.Limit(_return, self.m_maxForce)

        return _return


    def Cohesion(self, _boids):
        NeighbourDistance = 10.0
        _return = Vec3(0.0, 0.0, 0.0)
        count = 0

        for b in _boids:
            dist = self.Magnitude(self.m_pos - b.m_pos)
            if dist > 0 and dist < NeighbourDistance:
                _return = Vec3(_return.m_x + b.m_pos.m_x,_return.m_y + b.m_pos.m_y,_return.m_z + b.m_pos.m_z)
                count+=1
        if count > 0:
            _return = Vec3(_return.m_x/count,_return.m_y/count,_return.m_z/count)
            return self.Steer(_return, False)

        return _return


    #Useful Functions

    def Magnitude(self, _input):
        return math.sqrt(_input.m_x*_input.m_x + _input.m_y*_input.m_y + _input.m_z*_input.m_z)

    def Normalize(self, _input):
        _return = _input
        mag = self.Magnitude(_input)

        if mag > 0 :
            _return  = Vec3(_return.m_x/mag, _return.m_y/mag, _return.m_z/mag)

        return _return

    def Limit(self, _input, _val):
        _return = Vec3()
        mag = self.Magnitude(_input)

        if mag > _val:
            _return = self.Normalize(_input)
            _return = Vec3(_return.m_x * self.m_maxSpeed, _return.m_y * self.m_maxSpeed, _return.m_z * self.m_maxSpeed)

        return _return











