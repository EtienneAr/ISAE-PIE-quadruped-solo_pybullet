import subprocess
from isae.tools.cameraTool import *

file1 = open("MyFile.txt","r") 
#file1.write('\n')
#file1.write('\n')
#file1.write('ok')
st = file1.readlines()
#st.split('\n')
counter = 1
k = 0
for elt in st:
    k = k + 1
    if k == 1 or k%5 == 0 :      
          
        elt = elt[:-1]
        elt = elt.split(' ')
        elt = elt[1:]
        subprocess.check_output(["python3 -m solo_pybullet False False " + ' '.join(map(str, elt))], shell=True, universal_newlines=True)
        print('ok')
        saveVideo(counter)
        counter = counter + 1 



