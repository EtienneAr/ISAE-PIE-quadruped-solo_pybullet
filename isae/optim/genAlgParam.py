# class to represent a parameter (scalar, vector) in
# in an abstract way that allows it to be optimized 
# by a genetic algorithm

import random as rand
import numpy as np

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
    
    def toArray(self):
        return self.value

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
    
    def toArray(self):
        return self.value

class GA_pointFootTraj(genAlgParam):
    def __init__(self, paramRanges, value=None):
        # paramRanges : [xRange, yRange, nRange]
        self.xRange = paramRanges[0]
        self.yRange = paramRanges[1]
        self.nRange = paramRanges[2]
        self.value = value

    def pointsToPolygon(self, points):    
        sort_x = points[points[:,0].argsort()]
        sorted_points = np.array([sort_x[0]])

        superior_y = sort_x[sort_x[:,1] > sort_x[0,1]]
        superior_y = superior_y[superior_y[:,0].argsort()]

        inferior_y = sort_x[sort_x[:,1] < sort_x[0,1]]
        inferior_y = inferior_y[inferior_y[:,0].argsort()[::-1]]

        sorted_points = np.vstack((sorted_points, superior_y))
        sorted_points = np.vstack((sorted_points, inferior_y))
        sorted_points = np.vstack((sorted_points, sorted_points[0]))

        return sorted_points
    
    def initRandom(self):
        self.value = []
        n = rand.randint(int(self.nRange[0]), int(self.nRange[1]))
        for k in range(n):
            randX = self.xRange[0] + rand.random()*(self.xRange[1] - self.xRange[0])
            randY = self.yRange[0] + rand.random()*(self.yRange[1] - self.yRange[0])
            self.value.append([randX, randY])
        self.value = np.array(self.value)
        self.value = self.pointsToPolygon(self.value)

    def mergeWith(self, other):
        min_len = min(len(self.value), len(other.value))
        cut_index = rand.randint(1,min_len - 2)

        childVal1 = np.vstack((self.value[:cut_index], other.value[cut_index:]))
        childVal2 = np.vstack((other.value[:cut_index], self.value[cut_index:]))

        childVal1[-1] = childVal1[0]
        childVal2[-1] = childVal2[0]

        return GA_pointFootTraj([self.xRange, self.yRange, self.nRange], value=childVal1), GA_pointFootTraj([self.xRange, self.yRange, self.nRange], value=childVal2)

    def mutate_movePoints(self):
        for k in range(len(self.value) - 1):
            randIncX = 0.1*(-0.5 + rand.random())*self.xRange[1]
            randIncY = 0.1*(-0.5 + rand.random())*self.yRange[1]
            newVal = np.array([randIncX, randIncY])
            self.value[k] += newVal

        return

    def mutate_addPoint(self):
        # appends point between new_index and new_index + 1 
        new_index = rand.randint(0,len(self.value) - 2)
        newX = 0.5*(self.value[new_index][0] + self.value[new_index + 1][0])
        newY = 0.5*(self.value[new_index][1] + self.value[new_index + 1][1])

        self.value = np.insert(self.value, new_index + 1, [newX, newY], axis = 0)
        return 

    def mutate_removePoint(self):
        rmv_index = rand.randint(0,len(self.value) - 2)

        self.value = np.delete(self.value, rmv_index + 1, axis = 0)
        return 

    def mutate(self):
        self.mutate_movePoints()
        if rand.random() < 0.2:
            self.mutate_addPoint()
        if rand.random() < 0.2:
            self.mutate_removePoint()

        self.value[-1] = self.value[0]
        return self

    def toArray(self):
        return self.value

class GA_legsOffsets(genAlgParam):
    def __init__(self, maxOffset=1, value=[0.,0.,0.,0.]):
        self.maxOffset = maxOffset
        self.value = np.array(value)

    def initRandom(self):
        randOffsets = [self.maxOffset*rand.random() for i in range(4)]
        self.value = np.array(randOffsets)
    
    def mergeWith(self, other):
        randInd = rand.randint(0,4)
        childVal1 = np.concatenate((self.value[randInd:],other.value[:randInd]))
        childVal2 = np.concatenate((other.value[randInd:],self.value[:randInd]))
        return GA_legsOffsets(self.maxOffset, value=childVal1), GA_legsOffsets(self.maxOffset, value=childVal2)
    
    def mutate(self):
        randInd = rand.randint(0,3)
        self.value[randInd] += 0.1*(0.5 - rand.random())
        self.value[randInd] = max(0,min(1, self.value[randInd]))
        return GA_legsOffsets(self.maxOffset, self.value)
    
    def toArray(self):
        return np.array(self.value)
