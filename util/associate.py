# -*- coding: utf-8 -*-
"""
 * This is a script file to associate images and IMU data.

 * Copyright 2018 Nanjing University of Science and Technology
 * Author: Zhixing Hou <zxhou@njust.edu.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.

"""

fileImage = "D:/Dataset/myKinect/dataIMU1/filenames.txt"
fileIMU = "D:/Dataset/myKinect/dataIMU1/imuData.txt"
fileIMU2 = "D:/Dataset/myKinect/dataIMU1/imuDataRemErr.txt"
fileAss = "D:/Dataset/myKinect/dataIMU1/association.txt"
timeStampImage = []
timeStampIMU = []
timeImage = []
timeIMU = []

fl = open(fileImage,"r")
lines = fl.readlines()



for line3 in lines:
#    for li in line3:
    timeStampImage.append(line3.split()[0])
    timeImage.append(float(line3.split()[0][6:]))
fl.close()
    
fl = open(fileIMU,"r")
lineIMU = fl.readlines()
fl.close()

firstIMU = True
anglePrev = []
angleCurr = []
fl = open(fileIMU2,"w")
for line3 in lineIMU:
    if firstIMU:
        timeStampIMU.append(line3.split()[0])
        timeIMU.append(float(line3.split()[0][6:]))
        anglePrev = line3.split()[7:10]
#        print([anglePrev,"\n"])
        fl.writelines([" ".join(line3.split()), "\n"])
        firstIMU = False
    else:
        angleCurr = line3.split()[7:10]
        if abs(float(anglePrev[2]))> 178.5 or abs(float(angleCurr[2]))> 178.5:
            if abs(float(angleCurr[0]) - float(anglePrev[0])) < 3.0 and abs(float(angleCurr[1]) - float(anglePrev[1])) < 3.0 and abs(abs(float(angleCurr[2])) - abs(float(anglePrev[2]))) < 3.0 :
                timeStampIMU.append(line3.split()[0])
                timeIMU.append(float(line3.split()[0][6:]))
                fl.writelines([" ".join(line3.split()), "\n"])
                anglePrev = angleCurr
#                print([line3.split()[0], " ", angleCurr,"\n"])
#            else:
#                print([line3.split()[0], " ", angleCurr," ", anglePrev, "\n"])
        else:            
            if abs(float(angleCurr[0]) - float(anglePrev[0])) < 3.0 and abs(float(angleCurr[1]) - float(anglePrev[1])) < 3.0 and abs(float(angleCurr[2]) - float(anglePrev[2])) < 3.0 :
                timeStampIMU.append(line3.split()[0])
                timeIMU.append(float(line3.split()[0][6:]))
                fl.writelines([" ".join(line3.split()), "\n"])
                anglePrev = angleCurr
#                print([line3.split()[0], " ", angleCurr,"\n"])
#            else:
#                print([line3.split()[0], " ", angleCurr," ", anglePrev, "\n"])
fl.close()

fl = open(fileAss,'w')

indImage = 0
indIMU = 0
minDiffTime = 10

for tImage in timeImage:
    for tIMU in timeIMU:
        diffTime = abs(tImage - tIMU)
        if diffTime < minDiffTime:
            minDiffTime = diffTime
            indIMU = timeIMU.index(tIMU)
    fl.writelines([timeStampImage[indImage]," ",timeStampIMU[indIMU],"\n"])
    print([timeStampImage[indImage]," ",timeStampIMU[indIMU],"\n"])
    indImage = indImage + 1
    minDiffTime = 10
 
fl.close()
