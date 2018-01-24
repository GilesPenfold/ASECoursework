import numpy as np
from pyngl import *
import random


# Based from YABI implementation of flocking and processing.org

sys_random = random.SystemRandom()

# Boid Global Params
WeightRandom = 0.05
WeightSeek = 0.0
WeightAvoidPredator = 0.2
WeightSeekFood = 0.02
FoodSight = 20
FeedDistance = 1
MaxSpeed = 0.3
MaxForce = 0.05
MinSeparation = 5
CohesionDistance = 15.0

# Predator Global Params
PredatorRadius = 5
PredatorKillRadius = 1
PredatorWeightCohesion = 2.0
PredatorWeightSelfCohesion = 0.0
PredatorWeightSeparation = 2.0
PredatorWeightAlign = 0.3
PredatorWeightRandom = 0.2
PredatorMinSeparation = 3

# World Params - Do not change
Borders = np.array([56,56])


class Boid2():

    def __init__(self, *args, **kwargs):
        self.m_id = kwargs.get('_id', 0)
        self.m_pos = kwargs.get('_pos', np.random.uniform(-50,50,2))
        self.m_vel = kwargs.get('_vel', np.zeros(2))
        self.m_acc = kwargs.get('_acc', np.zeros(2))
        random.seed(self.GetRandomSeed())

        self.m_name = self.RandomNameGenerator()

        # GUI Variables

        self.WeightCohesion = kwargs.get('_globCoh', 1.5)
        self.WeightSeparation  = kwargs.get('_globSep', 2.0)
        self.WeightAlign = kwargs.get('_globAlign', 1.75)

        self.PredatorSight = kwargs.get('_predSig', 25)
        self.PredatorWeightAttack = kwargs.get('_predAtt',2.0)
        self.PredatorMaxSpeed = kwargs.get('_predSpeed',0.35)


        # Genetic algorithm params
        self.m_ticksAlive = 0
        self.m_dead = False
        self.m_tiredness = 0
        self.m_awareness = kwargs.get('_aware', random.uniform(1,5))
        self.m_strength = kwargs.get('_str', random.uniform(0.05,0.2))
        self.m_tirednessRate = kwargs.get('_tired', random.uniform(0.005,0.05))
        self.m_recoveryRate = kwargs.get('_rec', random.uniform(0,0.005))
        self.m_boost = kwargs.get('_boost', np.random.uniform(1,2,2))
        self.m_run = False

        # Genetic Adjustments
        self.m_alignment = kwargs.get('_align', random.uniform(0.2,1))
        self.m_cohesion = kwargs.get('_cohes', random.uniform(0.2, 1))
        self.m_separation = kwargs.get('_sep', random.uniform(0.2, 1))
        self.m_avoidance = kwargs.get('_avoid', random.uniform(0.2, 1))
        self.m_random = kwargs.get('_random', random.uniform(0.2, 1))
        self.m_food = kwargs.get('_food', random.uniform(0.2,1))

        # Fitness Scores
        self.m_fitness = 0

        self.m_isPredator = kwargs.get('_predator', False)

        self.m_colour = kwargs.get('_colour', np.ones(3))



    def GetRandomSeed(self):
        token = ''
        letters = "abcdefghiklmnopqrstuvwwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"
        for i in range(1, 36):
            token = token + sys_random.choice(letters)
        return token

    def RandomNameGenerator(self):
        name = ''
        description = ['Amazing', 'Wonderous', 'Outstanding', 'Powerful', 'Beautiful', 'Brave',
                       'Grey', 'Wise', 'Infamous', 'Famous', 'Quirky', 'Adorable', 'Excitable',
                       'Harmonious', 'Tasteful', 'Dirty', 'Rugged', 'Shaggy', 'Quarrelsome',
                       'Splendid', 'Angry', 'Broken', 'Wry', 'Envious', 'Jolly', 'Fearless',
                       'Yellow-bellied', 'Hulking', 'Tidy', 'Hateful', 'Young', 'Old', 'Majestic',
                       'Elderly', 'Youthful', 'Swanky', 'Productive', 'Strong', 'Awesome', 'Cyncial',
                       'Enchanting', 'Meek', 'Humble', 'Overpowering', 'Ambitious', 'Ethereal', 'Combative']
        first = ['Dave', 'Steve', 'George', 'Chris', 'Sarah', 'Lydia', 'Rachel', 'Alex',
                 'Donovan', 'William', 'Alice', 'Tom', 'Harry', 'Evan', 'Milo', 'Corentin',
                 'Giles', 'Josh', 'Maria', 'Graham', 'Joe', 'Jonathon', 'Lawrence', 'Jill',
                 'Frankie', 'Roy', 'Taliesin', 'Matthew', 'Laura', 'Travis', 'Sam', 'Philip',
                 'Ellen', 'Deborah', 'Isabel', 'Harold', 'Mike', 'Daisy', 'Harvey', 'Jesse',
                 'Marisha', 'Radvile', 'Caesar', 'Simon', 'Fiona', 'Ellie', 'Ryan', 'Yogi']
        second = ['Clayton', 'Penfold', 'Myers', 'Murray', 'Diaz', 'Swanson', 'Jaffe', 'Mercer',
                  'Willingham', 'Waterman', 'Smith', 'Sutton', 'Gomez', 'Pierce', 'Bond', 'Fox',
                  'Wolf', 'Cox', 'Harmon', 'Yates', 'Johnson', 'Floyd', 'Pratt', 'Griffin', 'Holmes',
                  'Walsh', 'Bush', 'Bailey', 'Ray', 'Bone', 'Wallace', 'Perkins', 'Harper', 'Blake',
                  'Macey', 'Blair', 'Caesar', 'Erickson', 'Underwood', 'Alfaro', 'McCrea', 'Swann',
                  'Sherwood', 'Shepard', 'Delves', 'Cooze', 'Lancaster', 'Read', 'Wilks', 'Wyer',
                  'Osman', 'Bear', 'Turney', 'Louise']
        title = ['Duke', 'King', 'Queen', 'Dutchess', 'Prince', 'Princess', 'Caesar', 'Emperor',
                 'Count', 'Viscount', 'Countess', 'Baron', 'Baroness', 'Empress', 'Captain',
                 'Lieutenant', 'General', 'Private', 'Admiral', 'Sir', 'Miss', 'Mister', 'Madame',
                 'Knight', 'Sultan', 'Bishop', 'Arch-Bishop', 'Dame', 'Viceroy', 'Pharaoh', 'Doge']
        place = ['Bournemouth', 'Norwich', 'Manchester', 'London', 'England', 'Ireland', 'Wales', 'Scotland',
                 'Munich', 'Berlin', 'Paris', 'Bristol', 'Edinburgh', 'Nottingham', 'Bangkok', 'Nairobi',
                 'Beijing', 'Yokohama', 'Tehran', 'Casablanca', 'Ankara', 'Surat', 'Busan', 'Cairo',
                 'Cape Town', 'Madrid', 'Tokyo', 'Suzhou', 'Dongguan', 'Mumbai', 'Singapore', 'Yangon',
                 'Jakarta', 'Bangalore', 'New York', 'York', 'Johannesburg', 'Santiago', 'Mexico',
                 'Washington', 'Austin', 'Madrid', 'Lagos', 'Hong Kong', 'Bogota', 'Baghdad', 'Alexandria',
                 'Kolkata', 'Delhi', 'Jaipur', 'Moscow', 'Shanghai', 'Los Angeles']
        name = 'The ' + sys_random.choice(description) + ' ' + sys_random.choice(title) + ' ' + sys_random.choice(first) + ' ' + sys_random.choice(second) + ' of ' + sys_random.choice(place)
        return name

    def CapVelocity(self, _mv):
        if(np.sqrt(self.m_vel.dot(self.m_vel))>_mv):
            self.m_vel = (self.m_vel/np.sqrt(self.m_vel.dot(self.m_vel)))*_mv

    def CenterOfMass(self,_boids):
        com = np.zeros(2)
        count = 0
        for dim in range(2):
            for b in _boids:
                if b.m_dead == False:
                    com[dim] = com[dim] + b.m_pos[dim]
                    count = count + 1
            com[dim] = com[dim]/count
        return com

    def CenterOfVelocity(self,_boids):
        cov = np.zeros(2)
        count = 0
        for dim in range(2):
            for b in _boids:
                if b.m_dead == False:
                    cov[dim] = cov[dim] + b.m_vel[dim]
                    count = count + 1
            cov[dim] = cov[dim] / count
        return cov

    def Bordering(self):
        adj = 1.5
        if self.m_pos[0] < -Borders[0]-2:
            self.m_pos[0] = Borders[0]
        if self.m_pos[1] > Borders[1]/adj+1:
            self.m_pos[1] = -Borders[1]/adj
        if self.m_pos[0] > Borders[0]+2:
            self.m_pos[0] = -Borders[0]
        if self.m_pos[1] < -Borders[1]/adj-1:
            self.m_pos[1] = Borders[1]/adj
        # if self.m_pos[0] < -Borders[0]:
        #     self.m_vel[0] += 5
        # if self.m_pos[0] > Borders[0]:
        #     self.m_vel[0] += -5
        # if self.m_pos[1] < -Borders[1]:
        #     self.m_vel[1] += 5
        # if self.m_pos[1] > Borders[1]:
        #     self.m_vel[1] += -5

    def Seek(self, _target):
        self.m_acc += self.Steer(_target, False)

    def Arrive(self, _target):
        self.m_acc += self.Steer(_target, True)

    def Limit(self, _input, _val):
        _return = np.zeros(2)
        # Magnitude
        mag = np.sqrt(_input.dot(_input))

        if mag > _val:
            _return = self.Normalize(_input)
            _return = _return * MaxSpeed
        return _return

    def Normalize(self, _input):
        _return = _input
        mag = np.sqrt(_input.dot(_input))
        if mag > 0:
            _return = _return / mag
        return _return

    def Steer(self, _target, _slow, _force):
        loc = _target - self.m_pos
        dist = np.sqrt(loc.dot(loc))

        if dist > 0:
            loc = loc/dist
            t = MaxSpeed * (dist/100.0)
            if _slow and dist < 100.0:
                loc = loc*t
            else:
                loc = loc*MaxSpeed
            SVec = loc-self.m_vel
            SVec = self.Limit(SVec, _force)
        else:
            SVec = np.zeros(2)
        return SVec

    def Move(self, _boids, _predators):
        for b in _boids:
            if b.m_dead == False:

                b.m_vel = b.m_vel + b.m_acc
                b.m_vel = b.Limit(b.m_vel, MaxSpeed)

                test = np.zeros(2)
                if b.m_vel[0] < 0.001 and b.m_vel[1] < 0.001:
                    b.m_vel[0] = MaxSpeed
                    b.m_vel[1] = MaxSpeed

                b.m_ticksAlive = b.m_ticksAlive + 1

                if b.m_tiredness < b.m_strength and b.m_run:
                    b.m_vel*=b.m_boost
                    b.m_tiredness+=b.m_tirednessRate
                    b.m_run = False
                if b.m_tiredness > 0:
                    b.m_tiredness -= b.m_recoveryRate

                b.Bordering()

                b.m_pos += b.m_vel



        if len(_predators) > 0:
            for p in _predators:
                p.m_acc += p.Steer(np.array([0, 0]), False, 0.008)
                p.m_vel += p.m_acc

                p.m_vel = p.Limit(p.m_vel, p.PredatorMaxSpeed)
                p.m_pos = p.m_pos + p.m_vel


                p.Bordering()

    def Flock(self, _boids, _predators, _food, _best):


        if len(_predators) > 0:
            for p in _predators:

                # Reset Acceleration
                p.m_acc = np.zeros(2)

                # Rule 1 - Cohesion
                #pCohesion = (COMBOIDS - p.m_pos)*PredatorWeightCohesion
                count = 0
                pCohesion = np.zeros(2)
                for b2 in _boids:
                    if b2.m_dead == False:
                        diff = b2.m_pos - p.m_pos
                        dist = np.sqrt(diff.dot((diff)))
                        if dist > 0 and dist < CohesionDistance:
                            pCohesion = pCohesion + b2.m_pos
                            count += 1
                if count > 0:
                    pCohesion = pCohesion / count
                    pCohesion = p.Steer(pCohesion, False, MaxForce)
                pCohesion = pCohesion * PredatorWeightCohesion

                count = 0
                pPCohesion = np.zeros(2)
                for b2 in _predators:
                    if b2.m_dead == False:
                        diff = b2.m_pos - p.m_pos
                        dist = np.sqrt(diff.dot((diff)))
                        if dist > 0 and dist < CohesionDistance:
                            pPCohesion = pPCohesion + b2.m_pos
                            count += 1
                if count > 0:
                    pPCohesion = pPCohesion / count
                    pPCohesion = p.Steer(pPCohesion, False, MaxForce)
                pPCohesion = pPCohesion * PredatorWeightSelfCohesion

                #Rule 2 - Separation
                pSeparation = np.zeros(2)
                count = 0
                for p2 in _predators:
                    if p.m_dead == False:
                        diff = p.m_pos - p2.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < MinSeparation and p2 != p and dist > 0:
                            temp = p2.Normalize(diff)
                            temp = temp / dist
                            pSeparation += temp
                            count += 1
                if count > 0:
                    pSeparation = pSeparation / count
                pSeparation = pSeparation * PredatorWeightSeparation

                # Rule 3 - Alignment
                count = 0
                pAlignment = np.zeros(2)
                for p2 in _predators:
                    if p.m_dead == False:
                        diff = p2.m_pos - p.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < CohesionDistance and p2 != p and dist > 0:
                            pAlignment = pAlignment + p2.m_vel
                            count += 1
                if count > 0:
                    pAlignment = pAlignment / count
                    pAlignment = p.Limit(pAlignment, MaxForce)
                pAlignment = pAlignment * PredatorWeightAlign

                # Rule 4 - Attack
                pAttack = np.zeros(2)
                for b in _boids:
                    if b.m_dead == False:
                        diff = p.m_pos - b.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < p.PredatorSight:
                            pAttack = pAttack - np.sqrt(diff.dot(diff))
                pAttack = pAttack*p.PredatorWeightAttack


                # Extra Rule 5 - Randomness
                randomness = np.random.uniform(-1, 1, 2) * PredatorWeightRandom


                p.m_acc += pCohesion +  pSeparation + pAlignment+ randomness + pPCohesion

        for b in _boids:
            if b.m_dead == False:

                # Reset acceleration
                b.m_acc = np.zeros(2)


                # Create counting variables
                countC = 0
                countA = 0
                countS = 0

                # Create placeholder arrays
                cohesion = np.zeros(2)
                separation = np.zeros(2)
                alignment = np.zeros(2)

                # Loop through our boids and obtain our 3 ruleset results
                for b2 in _boids:
                    if b2 == _best:
                        mult = 2
                    else:
                        mult = 1
                    if b2.m_dead == False:
                        diff = b2.m_pos - b.m_pos
                        diffS = b.m_pos - b2.m_pos
                        dist = np.sqrt(diff.dot((diff)))
                        distS = np.sqrt(diffS.dot(diffS))
                        if dist > 0 and dist < CohesionDistance:
                            cohesion = cohesion + b2.m_pos * mult
                            alignment = alignment + b2.m_vel * mult
                            countA += 1
                            countC +=1
                        if distS < MinSeparation and b2 != b and distS > 0:
                            temp = b2.Normalize(diffS)
                            temp = temp / distS
                            separation += temp
                            countS += 1

                # Apply our results and skew for the number of boids

                # Rule 1 - Cohesion

                if countC > 0:
                    cohesion = cohesion/countC
                    cohesion = b.Steer(cohesion, False, MaxForce)
                cohesion = cohesion * self.WeightCohesion * b.m_cohesion

                # Rule 2 - Separation

                if countS > 0:
                    separation = separation/countS
                separation = separation * self.WeightSeparation * b.m_separation

                # Rule 3 - Alignment

                if countA > 0:
                    alignment = alignment / countA
                    alignment = b.Limit(alignment, MaxForce)
                alignment = alignment * self.WeightAlign * b.m_alignment


                # Extra Rule 4 - Randomness
                randomness = np.random.uniform(-1,1,2)*WeightRandom * b.m_random

                # Extra Rule 5 - Flee from predators
                if len(_predators) > 0:
                    flee = np.zeros(2)
                    for p in _predators:
                        diff = p.m_pos - b.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < PredatorRadius+b.m_awareness:
                            b.m_run = True
                            if dist < PredatorKillRadius:
                                b.m_dead = True
                            else:
                                flee = (flee-diff)*WeightAvoidPredator * b.m_avoidance
                else:
                    flee = 0

                # Rule 6 - Food
                FoodCutoff = 15
                bFood = np.zeros(2)
                for f in _food:
                    if b.m_dead == False:
                        diff = b.m_pos - f.m_pos
                        dist = np.sqrt(diff.dot(diff))
                        if dist < FoodSight:
                            bFood = (bFood-diff)* WeightSeekFood * b.m_food
                        if dist < FeedDistance and f.m_stock > FoodCutoff:
                            b.m_fitness += 0.01
                            f.m_stock -= 1
                        if f.m_stock <= FoodCutoff:
                            f.Reset()


                if b.m_dead == False:
                    b.m_acc += separation + alignment + cohesion + randomness +  flee + bFood
                else:
                    b.m_colour = np.array([0.5,0.5,0.5])

                #if np.linalg.norm(b.m_vel) == 0:
                #    b.m_vel = np.array(MaxSpeed,MaxSpeed)

    def Draw(self, _camera):
        shader = ShaderLib.instance()

        t = Transformation()
        #t.setRotation(0,self.m_acc[0]*100,(self.m_acc[1]*100))
        if self.m_isPredator:
            t.setScale(3,3,3)
        t.setPosition(self.m_pos[0], self.m_pos[1], 0)

        M = t.getMatrix()
        MV = _camera.getViewMatrix()*M
        MVP = _camera.getVPMatrix()*M
        #normalMatrix=MV

        shader.setUniform("MVP", MVP)
        #shader.setUniform("normalMatrix", normalMatrix)

        prim = VAOPrimitives.instance()
        if self.m_isPredator:
            prim.draw("troll")
        else:
            prim.draw("disc")
