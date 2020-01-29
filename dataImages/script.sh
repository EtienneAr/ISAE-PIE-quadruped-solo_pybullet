#!/bin/bash


#for f in ./videos/*.avi; do echo "file '$f'" >> myList.txt; done 
ffmpeg -f concat -safe 0 -i myList.txt -c copy output2.avi

