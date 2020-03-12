# Import walkSimulation mother class
from isae.sim_control.walkSimulation import *
#from isae.optim.tn_grading_functions import *

class gradedSimulation(walkSimulation):
    def __init__(self):
        walkSimulation.__init__(self)
        self.grades = [0]*4

        #self.total_grade = 0
        #self.gradingClass = None # set from outside

    # functions needed for evaluation
    # def updateGrade(self):
    # def getGrade(self):

    #def setGrading(self, gradingClass):
    #    self.gradingClass = gradingClass

    def updateGrade_RMStoQdotRef(self,qdot_ref, factors, dt):
        return -np.sum(factors * (self.qdotBase[-1][:6] - qdot_ref) ** 2) * dt

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

    def updateGrade_penalizeContacts(self):
        contacts = p.getContactPoints(bodyA = 0) # checks collisions with bodyA=0 (ground plane)
        return -len(contacts)/2

    def updateGrade_constantZ(self, bhs):
        z_error = np.sqrt((0.25*np.sum(bhs) - self.qBase[-1][2])**2)
        return -z_error

    # target orientation specified as a quaternion
    def updateGrade_constantOrientation(self, targetOrientation):
        selfOrientation = np.squeeze(np.asarray(self.qBase[-1][3:]))
        targetOrientation = np.squeeze(np.asarray(targetOrientation))
        orientationError = 1 - np.inner(selfOrientation, targetOrientation)**2
        return -1.*orientationError


    # update all specified grades, 2 for now
    # has to be the same number as len(self.grades)
    def updateGrades(self):
        # Dist
        d = self.updateGrade_MaxFinalDist(self.grades[0])
        self.grades[0] += d
        #print(self.grades[0])
        # RMS       
        self.grades[1] += self.updateGrade_RMStoQdotRef(np.vstack([.2, 0, 0, 0, 0, 0]), np.vstack([150, 20, 20, 1, 1, 1]), self.dt) + 0.1*self.updateGrade_constantOrientation(np.array([0,0,0,1]))
        # Contacts penalization
        #self.grades[2] += self.updateGrade_penalizeContacts()
        # Body height stability
        #self.grades[2] += self.updateGrade_constantZ(self.robotController.bHs)/(self.duration/self.dt)
        self.grades[2] += self.updateGrade_constantOrientation(np.array([0,0,0,1]))
        #print(self.updateGrade_constantOrientation(np.array([0,0,0,1])))
        # Dist + contacts
        #self.grades[3] += d + 0.1*self.updateGrade_penalizeContacts()/(self.duration/self.dt)
        self.grades[3] += 0.4*d + 0.1*self.updateGrade_constantOrientation(np.array([0,0,0,1])) + 0.1*self.updateGrade_penalizeContacts()/(self.duration/self.dt)
    
    
    def stepSim(self):
        super(gradedSimulation, self).stepSim()
        self.updateGrades()
        #print(self.grades[2])
        #self.gradesSeries.append(list(self.grades))
    
    #def plotGrades(self):
    #    gr_array = np.array(self.grades[0])
    #    plt.plot(gr_array, label='Dist')