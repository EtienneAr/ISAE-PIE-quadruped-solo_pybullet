# class to represent a parameter (scalar, vector) in
# in an abstract way that allows it to be optimized 
# by a genetic algorithm

import random as rand

# Super class with methods that should be implemented
# in order to pass the parameter to the GA
class genAlgParam(object):
    # value could be a scalar, a list, a list of points...
    def __init__(self):
        self.value = None

    # method to pick a random value for this instance 
    # of the parameter
    def initRandom(self):
        self.value = randNewVal
    
    # method to create 2 new parameters from existing ones
    # (reprod. step in the GA)
    def mergeWith(self, other):
        return childVal1, childVal2

    # mutation method : returns a mutated value from 
    # the current instance value
    def mutate(self):
        return mutVal

# scalar parameter
class GA_scalar(genAlgParam):
    def __init__(self, minMaxVals, value = 0):
        self.value = value
        self.minVal = minMaxVals[0]
        self.maxVal = minMaxVals[1]

    def initRandom(self):
        self.value = self.minVal + (self.maxVal - self.minVal)*rand.random()
    
    def mergeWith(self, other):
        coeff = rand.random()
        childVal1 = coeff*self.value + (1-coeff)*other.value
        childVal2 = (1-coeff)*self.value + coeff*other.value
        return GA_scalar([self.minVal, self.maxVal], value=childVal1),GA_scalar([self.minVal, self.maxVal], value=childVal2)
    
    def mutate(self):
        newVal = (0.9 + 0.2*rand.random())*self.value
        newVal = min(self.maxVal, newVal)
        newVal = max(self.minVal, newVal)
        return GA_scalar([self.minVal, self.maxVal], value=newVal)

# 2d point parameter
class GA_2dPoint(genAlgParam):
    def __init__(self, xyRanges, value = [0,0]):
        self.xRange = xyRanges[0]
        self.yRange = xyRanges[1]
        self.value = value

    def initRandom(self):
        randX = self.xRange[0] + rand.random()*(self.xRange[1] - self.xRange[0])
        randY = self.yRange[0] + rand.random()*(self.yRange[1] - self.yRange[0])
        self.value = [randX, randY]

    def mergeWith(self, other):
        coeff = rand.random()
        diff = [other.value[0] - self.value[0], other.value[1] - self.value[1]]
        childVals1 = [self.value[0] + coeff*diff[0], self.value[1] + coeff*diff[1]]
        childVals2 = [self.value[0] + (1-coeff)*diff[0], self.value[1] + (1-coeff)*diff[1]]
        return GA_2dPoint([self.xRange, self.yRange], value=childVals1), GA_2dPoint([self.xRange, self.yRange], value=childVals2)

    def mutate(self):
        randIncX = 0.1*(-0.5 + rand.random())*self.xRange[1]
        randIncY = 0.1*(-0.5 + rand.random())*self.yRange[1]
        newVal = [randIncX, randIncY]
        return GA_2dPoint([self.xRange, self.yRange], value=newVal)
