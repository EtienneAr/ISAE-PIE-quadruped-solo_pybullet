import numpy as np 
import matplotlib.pyplot as plt 

genLog = np.load("optim_logs/optim_gen30_pop60_bh_traj_offs.npy", allow_pickle=True)
#print(genLog[-1,:,0])

plt.figure()
for k in range(int(len(genLog))):
    scores = genLog[k,0,:]
    plt.plot(scores, 'black',linestyle='-', marker='', alpha=0.1+0.9*k/len(genLog), lw=0.5)
    #plt.hist(scores, bins=10,linestyle='-', alpha=0.1+0.9*k/len(genLog), lw=0.5)

plt.ylabel("Score")
plt.xlabel("Sorted population")
plt.show()

k = 0
#traj = genLog[-1,1,k]
#print(traj[2])
#
#plt.figure()
#
#for k in range(int(len(genLog[-1,0]))):
#    traj = genLog[-1,1,k]
#    plt.plot([traj[2][i][0] for i in range(len(traj[2]))], [traj[2][i][1] for i in range(len(traj[2]))], 'b', alpha=0.1+0.9*k/len(genLog[-1,0]), lw=0.5)
#plt.show()