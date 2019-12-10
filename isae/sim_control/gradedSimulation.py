# Import walkSimulation mother class
from isae.sim_control.walkSimulation import *

class gradedSimulation(walkSimulation):
    def __init__(self):
        walkSimulation.__init__(self)
        self.grade = 0

    # functions needed for evaluation
    # def updateGrade(self):
    # def getFinalGrade(self):

    def updateGrade(self):
        # use grading function from grading package
        return 
    
    def getGrade(self):
        return self.grade