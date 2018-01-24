import boid2
import food
from random import choice
from random import getrandbits
from random import uniform
import numpy as np
import random

sys_random = random.SystemRandom()

# TODO
# Add overall fitness to the boids in relation to food and time spent alive
# Set the target for the network as a distance away from the nearest predator
# And closest to the nearest food

class Flock():

    def __init__(self, *args, **kwargs):
        self.m_boids = kwargs.get('_flock', [])
        self.m_predators = kwargs.get('_predators', [])
        self.m_food = []
        self.m_run = []
        self.ticks = 0
        self.maxTicks = 2000
        self.m_maxBoids = 50
        self.m_maxPredators = 5
        self.m_genCount = 0
        self.m_miniGenCount = 0
        self.m_survivorCount = []
        self.bestBoid = None

        self.startAware = kwargs.get('_aware',5)
        self.startStr = kwargs.get('_str',0.2)
        self.startTired = kwargs.get('_tired',0.05)
        self.startRecov = kwargs.get('_recov',0.005)

        self.boidCohesion = 1.75
        self.boidAlignment = 1.5
        self.boidSeparation = 2.0

        self.predAtt = 2.0
        self.predSig = 25
        self.predSpeed = 0.35


    def Flock(self):
        self.m_boids[0].Flock(self.m_boids, self.m_predators, self.m_food, self.bestBoid)


    def Update(self):
        if self.ticks < self.maxTicks:
            self.m_boids[0].Move(self.m_boids, self.m_predators)
            self.ticks = self.ticks + 1
        elif self.ticks == self.maxTicks:
            self.MiniGeneration()

    def MiniGeneration(self):

        # Reward Survivors, Punish Dead
        survivors = []
        dead = []
        self.bestBoid = self.m_boids[0]
        for x in self.m_boids:
            if x.m_dead == False:
                survivors.append(x)
                x.m_fitness += 1.0
                x.m_fitness += x.m_ticksAlive/1000
                x.m_ticksAlive = 0
                x.m_pos = np.random.uniform(-50,50,2)
            else:
                dead.append(x)
                x.m_fitness -= 1.5
                x.m_fitness += x.m_ticksAlive / 1000
                x.m_ticksAlive = 0
                x.m_dead = False
                x.m_pos = np.random.uniform(-50,50,2)
                x.m_colour = np.ones(3)
            if x.m_fitness > self.bestBoid.m_fitness:
                self.bestBoid = x
        self.bestBoid.m_colour = np.array([1.0,0.8,0])

        print 'MiniGeneration: ' + str(self.m_miniGenCount)
        print '# of Survivors: ' + str(len(survivors))

        # Increment mini generation count
        self.m_miniGenCount += 1

        # Reset ticks
        self.ticks = 0

        if self.m_miniGenCount % 10 == 0:
            self.m_miniGenCount = 0
            self.NextGeneration()

    def NextGeneration(self):

        topTwenty = sorted(self.m_boids, key=lambda x: x.m_fitness, reverse=True)
        topTwenty = topTwenty[:20]

        print 'Top Performers of Generation #' + str(self.m_genCount) + ':'
        for x in topTwenty:
            print str(x.m_id) + ':' + str(x.m_fitness) + ': ' + x.m_name


        # Update our generation counter
        self.m_genCount += 1

        # Reset ticks
        self.ticks = 0

        # If we're done, print out results
        if self.m_genCount % 10 == 0:
            print self.m_survivorCount
        else:
            # Clear population
            self.m_boids = []

            # Randomly breed 1/2 of the surviving population
            for x in range(0,10):
                mother = choice(topTwenty)
                father = choice(topTwenty)
                self.BreedBoids(mother, father)

            # Randomly mutate 1/4 of the surviving population
            for x in range(0,5):
                self.MutateBoids(choice(topTwenty))

            # Add survivors back into the population
            for x in range (0,20):
                tBoid = topTwenty[x]
                tBoid.m_colour = np.array([0.0,0.0,1.0])
                tBoid.m_id = len(self.m_boids)
                tBoid.m_fitness = 0
                tBoid.m_ticksAlive = 0
                self.m_boids.append(tBoid)

            # If we're not at max population count, randomly generate more boids
            if len(self.m_boids) < self.m_maxBoids:
                diff = self.m_maxBoids - len(self.m_boids)
                for i in range(self.m_maxBoids - diff, self.m_maxBoids):
                    self.m_boids.append((boid2.Boid2(_id=i)))

            # Reset Predators
            self.m_predators = []

            # Re-Add Predators
            self.AddPredator(self.m_maxPredators)

    def AddFood(self, _i):
        for x in range (0,_i):
            self.m_food.append(food.Food())

    def AddBoid(self, _i):
        for x in range(0,_i):
            self.m_boids.append(boid2.Boid2(_id=x, _aware=random.uniform(1,self.startAware), _str=random.uniform(0.05,self.startStr), _rec=random.uniform(0,self.startRecov),
                                            _tired=random.uniform(0.005,self.startTired), _globAlign=self.boidAlignment, _globCoh=self.boidCohesion, _globSep=self.boidSeparation))
            random.seed(self.m_boids[x].GetRandomSeed())
            self.m_run.append(False)

    def BreedBoids(self, _mother, _father):
        if _mother != _father:
            strength = choice([_mother.m_strength, _father.m_strength])
            boostA = choice([_mother.m_boost[0], _father.m_boost[0]])
            boostB = choice([_mother.m_boost[1], _father.m_boost[1]])
            tirednessRate = choice([_mother.m_tirednessRate, _father.m_tirednessRate])
            recoveryRate = choice([_mother.m_recoveryRate, _father.m_recoveryRate])
            awareness = choice([_mother.m_awareness, _father.m_awareness])
            align = choice([_mother.m_alignment, _father.m_alignment])
            sep = choice([_mother.m_separation, _father.m_separation])
            cohes = choice([_mother.m_cohesion, _father.m_cohesion])
            avoid = choice([_mother.m_avoidance, _father.m_avoidance])
            rand = choice([_mother.m_random, _father.m_random])
            food = choice([_mother.m_food, _father.m_food])
            self.m_boids.append(boid2.Boid2(_id=len(self.m_boids), _str=strength, _boost=np.array([boostA, boostB]),
                                            _tired=tirednessRate, _rec=recoveryRate, _aware=awareness,
                                            _align=align, _cohes=cohes, _sep=sep, _avoid=avoid, _random=rand,
                                            _food=food, _colour=np.array([1.0,0.4,6])))

    def MutateBoids(self, _boid):
        if bool(getrandbits(1)):
            strength = _boid.m_strength + uniform(-0.2,0.2)
        else:
            strength = _boid.m_strength
        if bool(getrandbits(1)):
            boost = _boid.m_boost + np.random.uniform(-0.2,0.2,2)
        else:
            boost = _boid.m_boost
        if bool(getrandbits(1)):
            tired = _boid.m_tirednessRate + uniform(-0.02,0.02)
        else:
            tired = _boid.m_tirednessRate
        if bool(getrandbits(1)):
            recover = _boid.m_recoveryRate + uniform(-0.02,0.02)
        else:
            recover = _boid.m_recoveryRate
        if bool(getrandbits(1)):
            align = _boid.m_alignment + uniform(-0.2, 0.2)
        else:
            align = _boid.m_alignment
        if bool(getrandbits(1)):
            sep = _boid.m_separation + uniform(-0.2, 0.2)
        else:
            sep = _boid.m_separation
        if bool(getrandbits(1)):
            cohes = _boid.m_cohesion + uniform(-0.2, 0.2)
        else:
            cohes = _boid.m_cohesion
        if bool(getrandbits(1)):
            avoid = _boid.m_avoidance + uniform(-0.2, 0.2)
        else:
            avoid = _boid.m_avoidance
        if bool(getrandbits(1)):
            rand = _boid.m_random + uniform(-0.2, 0.2)
        else:
            rand = _boid.m_random
        if bool(getrandbits(1)):
            food = _boid.m_food + uniform(-0.2, 0.2)
        else:
            food = _boid.m_food
        if bool(getrandbits(1)):
            aware = _boid.m_awareness + uniform(-0.02, 0.02)
        else:
            aware = _boid.m_awareness

        self.m_boids.append(boid2.Boid2(_id=len(self.m_boids), _str=strength, _boost=boost,
                                        _tired=tired, _rec=recover, _aware=aware,
                                        _align=align, _cohes=cohes, _sep=sep, _avoid=avoid, _random=rand,
                                        _food=food,_colour=np.array([0,1,0])))

    def AddPredator(self, _i):
        for x in range(0,_i):
            self.m_predators.append(boid2.Boid2(_id=x, _predator=True, _predAtt=self.predAtt, _predSig=self.predSig, _predSpeed=self.predSpeed))

    def Draw(self, _camera, _shader):

        for f in self.m_food:
            _shader.setUniform("Colour", 0.2, 0.2, 0.0, 1.0)
            f.Draw(_camera)
        for b in self.m_boids:
            _shader.setUniform("Colour", b.m_colour[0], b.m_colour[1], b.m_colour[2],1.0)
            b.Draw(_camera)

        if len(self.m_predators) > 0:
            for p in self.m_predators:
                _shader.setUniform("Colour", 1.0, 0.0, 0.0, 1.0)
                p.Draw(_camera)













