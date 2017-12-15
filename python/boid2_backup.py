import numpy as np
from pyngl import *

# Based from YABI implementation of flocking

# Boid Params
WeightCohesion = 0.05
WeightSeparation = 1.0
WeightAlign = 0.125
WeightRandom = 0.0
WeightCenter = 0.0
WeightAvoidPredator = 0.05
MaxSpeed = 0.2
MaxForce = 0.5
Center = 50 * np.ones(2)
MinSeparation = 3

# Predator Params
PredatorRadius = 5
PredatorKillRadius = 1
PredatorSight = 10
PredatorWeightCohesion = 1.0
PredatorWeightSeparation = 1.5
PredatorWeightAttack = 0.5
PredatorWeightRandom = 0.2
WeightKnot = 2
PredatorMaxSpeed = 0.25
PredatorMinSeparation = 3

# World Params
Borders = np.array([58, 58])


class Boid2():
    def __init__(self, *args, **kwargs):
        self.m_id = kwargs.get('_id', 0)
        self.m_pos = kwargs.get('_pos', np.random.uniform(-50, 50, 2))
        self.m_vel = kwargs.get('_vel', np.zeros(2))
        self.m_acc = kwargs.get('_acc', np.zeros(2))

        self.m_dead = False

        self.m_colour = kwargs.get('_colour', np.ones(3))

    def CapVelocity(self, _mv):
        if (np.sqrt(self.m_vel.dot(self.m_vel)) > _mv):
            self.m_vel = (self.m_vel / np.sqrt(self.m_vel.dot(self.m_vel))) * _mv

    def CenterOfMass(self, _boids):
        com = np.zeros(2)
        count = 0
        for dim in range(2):
            for b in _boids:
                if b.m_dead == False:
                    com[dim] = com[dim] + b.m_pos[dim]
                    count = count + 1
            com[dim] = com[dim] / count
        return com

    def CenterOfVelocity(self, _boids):
        cov = np.zeros(2)
        count = 0
        for dim in range(2):
            for b in _boids:
                if b.m_dead == False:
                    cov[dim] = cov[dim] + b.m_vel[dim]
                    count = count + 1
            cov[dim] = cov[dim] / count
        return cov

    def Normalize(self, _input):
        _return = _input
        mag = np.sqrt(_input.dot(_input))
        if mag > 0:
            _return = _return / mag
        return _return

    def Bordering(self):
        adj = 1.5
        if self.m_pos[0] < -Borders[0] - 2:
            self.m_pos[0] = Borders[0]
        if self.m_pos[1] > Borders[1] / adj + 1:
            self.m_pos[1] = -Borders[1] / adj
        if self.m_pos[0] > Borders[0] + 2:
            self.m_pos[0] = -Borders[0]
        if self.m_pos[1] < -Borders[1] / adj - 1:
            self.m_pos[1] = Borders[1] / adj

    def Seek(self, _target):
        self.m_acc += self.Steer(_target, False)

    def Arrive(self, _target):
        self.m_acc += self.Steer(_target, True)

    def Limit(self, _input, _val):
        _return = np.zeros(2)
        mag = np.linalg.norm(_input)

        if mag > _val:
            _return = self.Normalize(_input)
            _return = _return * MaxSpeed
        return _return

    def Steer(self, _target, _slow):
        loc = _target - self.m_pos
        dist = np.linalg.norm(loc)

        if dist > 0:
            loc = loc / dist
            t = MaxSpeed * (dist / 100.0)
            if _slow and dist < 100.0:
                loc = loc * t
            else:
                loc = loc * MaxSpeed
            SVec = loc - self.m_vel
            SVec = self.Limit(SVec, MaxForce)
        else:
            SVec = np.zeros(2)
        return SVec

    def Move(self, _boids, _predators):
        for b in _boids:
            if b.m_dead == False:
                b.m_vel = b.m_vel + b.m_acc
                b.m_pos = b.m_pos + b.m_vel
                b.Bordering()
                b.m_acc = np.zeros(2)
        if len(_predators) > 0:
            for p in _predators:
                p.m_pos = p.m_pos + p.m_vel

                p.Bordering()

    def Flock(self, _boids, _predators):

        COMBOIDS = self.CenterOfMass(_boids)
        COVBOIDS = self.CenterOfVelocity(_boids)
        if len(_predators) > 0:
            for p in _predators:

                # Rule 1 - Cohesion
                pCohesion = (COMBOIDS - p.m_pos) * PredatorWeightCohesion

                # Rule 2 - Separation
                pSeparation = np.zeros(2)
                for p2 in _predators:
                    diff = p2.m_pos - p.m_pos
                    dist = np.sqrt(diff.dot(diff))
                    if dist < PredatorMinSeparation and p2 != p and dist > 0:
                        pSeparation = (pSeparation - diff) * PredatorWeightSeparation
                pSeparation = pSeparation

                # Rule 3 - Attack
                pAttack = np.zeros(2)
                for b in _boids:
                    if b.m_dead == False:
                        diff = p.m_pos - b.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < PredatorSight:
                            pAttack = pAttack - np.sqrt(diff.dot(diff))
                pAttack = pAttack * PredatorWeightAttack

                # Extra Rule 4 - Randomness
                randomness = np.random.uniform(-1, 1, 2) * PredatorWeightRandom

                p.m_vel = pCohesion + pAttack + pSeparation + randomness

                p.CapVelocity(PredatorMaxSpeed)

        for b in _boids:
            if b.m_dead == False:
                # Rule 1 - Cohesion
                # cohesion = self.Steer((self.CenterOfMass(_boids)-b.m_pos), False)*WeightCOM
                cohesion = (COMBOIDS - b.m_pos) * WeightCohesion

                # Rule 2 - Separation
                separation = np.zeros(2)
                for b2 in _boids:
                    if b.m_dead == False:
                        diff = b2.m_pos - b.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < MinSeparation and b2 != b and dist > 0:
                            separation = separation - (np.sqrt(diff.dot(diff))) / dist
                            # separation = separation + (self.Normalize(diff)/dist)
                separation = separation * WeightSeparation

                # Rule 3 - Alignment
                # alignment = self.Limit((self.CenterOfVelocity(_boids) - b.m_vel), MaxForce)*WeightAlign
                alignment = (COVBOIDS - b.m_vel) * WeightAlign

                # Extra Rule 4 - Randomness
                randomness = np.random.uniform(-1, 1, 2) * WeightRandom

                # Extra Rule 5 - Move towards center
                centralmove = (Center - b.m_pos) * WeightCenter

                # Extra Rule 6 - Flee from predators
                if len(_predators) > 0:
                    flee = np.zeros(2)
                    for p in _predators:
                        diff = p.m_pos - b.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < PredatorRadius:
                            if dist < PredatorKillRadius:
                                b.m_dead = True
                            else:
                                flee = (flee - diff) * WeightAvoidPredator
                else:
                    flee = 0

                if b.m_dead == False:
                    b.m_acc = randomness
                    b.m_vel = centralmove + flee + cohesion + alignment + separation
                    b.CapVelocity(MaxSpeed)
                else:
                    b.m_colour = np.array([0.0, 0.0, 0.0])

                    # if np.linalg.norm(b.m_vel) == 0:
                    #    b.m_vel = np.array(MaxSpeed,MaxSpeed)

    def Draw(self, _camera):
        shader = ShaderLib.instance()

        t = Transformation()
        t.setPosition(self.m_pos[0], self.m_pos[1], 0)

        M = t.getMatrix()
        MV = _camera.getViewMatrix() * M
        MVP = _camera.getVPMatrix() * M
        # normalMatrix=MV
        # normalMatrix.inverse().transpose()

        shader.setUniform("MVP", MVP)
        # shader.setUniform("normalMatrix", normalMatrix)

        prim = VAOPrimitives.instance()
        prim.draw("cone")
