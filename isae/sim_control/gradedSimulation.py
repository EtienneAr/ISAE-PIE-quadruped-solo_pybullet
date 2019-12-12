# Import walkSimulation mother class
from isae.sim_control.walkSimulation import *
#from isae.optim.tn_grading_functions import *

class gradedSimulation(walkSimulation):
    def __init__(self):
        walkSimulation.__init__(self)
        self.grades = [0,0]
        self.gradesSeries = []

        #self.total_grade = 0
        #self.gradingClass = None # set from outside

    # functions needed for evaluation
    # def updateGrade(self):
    # def getGrade(self):

    #def setGrading(self, gradingClass):
    #    self.gradingClass = gradingClass

    def updateGrade_RMStoQdotRef(self,qdot_ref, factors, dt):
        return -np.sum(factors * (self.qdotBase[0][:6] - qdot_ref) ** 2) * dt

    def updateGrade_MaxFinalDist(self, maxDist, offset = 0):
        #print(len(self.qBase[0]))
        x0, y0 = self.qBase[0][0], self.qBase[0][1]
        xf, yf = self.qBase[-1][0], self.qBase[-1][1]
        currDist = np.sqrt((xf-x0)**2 + (yf-y0)**2)[0]

        if(currDist > maxDist):
            diff = currDist - maxDist
            maxDist = currDist
            return diff
        return 0

    # update all specified grades, 2 for now
    # has to be the same number as len(self.grades)
    def updateGrades(self):
        # Dist
        self.grades[0] += self.updateGrade_MaxFinalDist(self.grades[0])
        # RMS       
        self.grades[1] += self.updateGrade_RMStoQdotRef(np.vstack([75, 10, 10, 1, 1, 1]), np.vstack([.2, 0, 0, 0, 0, 0]), self.dt)
    
    def stepSim(self):
        super(gradedSimulation, self).stepSim()
        self.updateGrades()
        #print(self.grades[1])
        self.gradesSeries.append(list(self.grades))
    
    def plotGrades(self):
        gr_array = np.array(self.gradesSeries)
        plt.plot(gr_array[:,0], label='Dist')