import numpy as np
import random

class Perceptron():

    def __init__(self, *args, **kwargs):
        self.numWeights = kwargs.get('_weights', 0)
        self.weights = np.random.uniform(-1,1,self.numWeights)

        self.learningRate = kwargs.get('_lr', 0.001)

    def FeedForward(self, _forces):

        sum = np.zeros(len(self.weights))
        sum += _forces[0] * self.weights
        return sum

    def Activate(self, _sum):
        if _sum > 0:
            return 1
        else:
            return -1


    def Train(self, _forces, _error):
        for i in range(0,len(self.weights)):
            self.weights[i] += self.learningRate * _error[0] * _forces[0][0]
            self.weights[i] += self.learningRate * _error[1] * _forces[0][1]



