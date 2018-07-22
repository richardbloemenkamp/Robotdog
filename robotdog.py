# -*- coding: utf-8 -*-
"""
Created on Sun Jun 10 18:28:23 2018

@author: Richard Bloemenkamp
"""
import pybullet as p
import time
import numpy as np
    

p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)


#Dog robot part shapes
sh_body = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.45, 0.08, 0.02])
sh_extraweight = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.45, 0.08, 0.025])
sh_roll = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
sh_hip = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
sh_knee = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
sh_foot = p.createCollisionShape(p.GEOM_SPHERE,radius=0.04)

#The Dog robot is the body with other shapes linked to it
body_Mass = 1
visualShapeId = -1
link_Masses=[.1, .1, .1, .1,
             .1, .1, .1, .1,
             .1, .1, .1, .1,
             .1, .1, .1, .1,
             20]
linkCollisionShapeIndices=[sh_roll, sh_hip, sh_knee, sh_foot,
                           sh_roll, sh_hip, sh_knee, sh_foot,
                           sh_roll, sh_hip, sh_knee, sh_foot,
                           sh_roll, sh_hip, sh_knee, sh_foot,
                           sh_extraweight]
nlnk=len(link_Masses)
linkVisualShapeIndices=[-1]*nlnk    #=[-1,-1,-1, ... , -1]
#link positions wrt the link they are attached to
xhipf=0.4
xhipb=-0.4
yhipl=0.1

xoffh=0.05
yoffh=0.05
hu=0.3
hl=0.3
linkPositions=[[xhipf, yhipl, 0], [xoffh, yoffh, 0], [0, 0, -hu], [0, 0, -hl],
               [xhipf, -yhipl, 0], [xoffh, -yoffh, 0], [0, 0, -hu], [0, 0, -hl],
               [xhipb, yhipl, 0], [xoffh, yoffh, 0], [0, 0, -hu], [0, 0, -hl],
               [xhipb, -yhipl, 0], [xoffh, -yoffh, 0], [0, 0, -hu], [0, 0, -hl],
               [0,0,+0.029]]
linkOrientations=[[0,0,0,1]]*nlnk
linkInertialFramePositions=[[0,0,0]]*nlnk
#Note the orientations are given in quaternions (4 params). There are function to convert of Euler angles and back
linkInertialFrameOrientations=[[0,0,0,1]]*nlnk
#indices determine for each link which other link it is attached to
# for example 3rd index = 2 means that the front left knee jjoint is attached to the front left hip
indices=[0, 1, 2, 3,
         0, 5, 6, 7,
         0, 9,10,11,
         0,13,14,15,
         0]
#Most joint are revolving. The prismatic joints are kept fixed for now
jointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
            p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
            p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
            p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
            p.JOINT_PRISMATIC]
#revolution axis for each revolving joint
axis=[[1,0,0], [0,1,0], [0,1,0], [0,0,1],
      [1,0,0], [0,1,0], [0,1,0], [0,0,1],
      [1,0,0], [0,1,0], [0,1,0], [0,0,1],
      [1,0,0], [0,1,0], [0,1,0], [0,0,1],
      [0,0,1]]

#Drop the body in the scene at the following body coordinates
basePosition = [0,0,1]
baseOrientation = [0,0,0,1]
#Main function that creates the dog
dog = p.createMultiBody(body_Mass,sh_body,visualShapeId,basePosition,baseOrientation,
                        linkMasses=link_Masses,
                        linkCollisionShapeIndices=linkCollisionShapeIndices,
                        linkVisualShapeIndices=linkVisualShapeIndices,
                        linkPositions=linkPositions,
                        linkOrientations=linkOrientations,
                        linkInertialFramePositions=linkInertialFramePositions,
                        linkInertialFrameOrientations=linkInertialFrameOrientations,
                        linkParentIndices=indices,
                        linkJointTypes=jointTypes,
                        linkJointAxis=axis)			
#Due to the weight the prismatic extraweight block needs to be motored up
joint=16
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.01,force=1000,maxVelocity=3)
#Same for the prismatic feet spheres
joint=3
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)
joint=7
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)
joint=11
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)
joint=15
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=3)

#Add earth like gravity
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(1)
#Point the camera at the robot at the desired angle and distance
p.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.25])



if (0):
    t0=time.time()
    t=time.time()
    while ((t-t0)<10):
        t=time.time()
    p.disconnect()
    brake    

#Scenery e.g. an inclined box
boxHalfLength = 2.5
boxHalfWidth = 2.5
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
mass = 1
block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [-2,0,-0.1],baseOrientation=[0.0,0.1,0.0,1])


#Add extra lateral friction to the feet spheres
p.changeDynamics(dog,3,lateralFriction=2)
p.changeDynamics(dog,7,lateralFriction=2)
p.changeDynamics(dog,11,lateralFriction=2)
p.changeDynamics(dog,15,lateralFriction=2)


#Function to calculate roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
def xyztoang(x,y,z,yoffh,hu,hl):
    dyz=np.sqrt(y**2+z**2)
    lyz=np.sqrt(dyz**2-yoffh**2)
    gamma_yz=-np.arctan(y/z)
    gamma_h_offset=-np.arctan(-yoffh/lyz)
    gamma=gamma_yz-gamma_h_offset
    
    lxzp=np.sqrt(lyz**2+x**2)
    n=(lxzp**2-hl**2-hu**2)/(2*hu)
    beta=-np.arccos(n/hl)
    
    alfa_xzp=-np.arctan(x/lyz)
    alfa_off=np.arccos((hu+n)/lxzp)
    alfa=alfa_xzp+alfa_off
    if any( np.isnan([gamma,alfa,beta])):
        print(x,y,z,yoffh,hu,hl)
    return [gamma,alfa,beta]


def setlegsxyz(xvec,yvec,zvec,vvec):
    #[a1,a2]=xztoang(xvec[0],zvec[0],1,1)
    a=xyztoang(xvec[0]-xhipf,yvec[0]-yhipl,zvec[0],yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=1
    #any(np.isnan(a))
    joint=0
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=1000,maxVelocity=spd)
    joint=1
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=1000,maxVelocity=vvec[0])
    joint=2
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=1000,maxVelocity=vvec[0])

    a=xyztoang(xvec[1]-xhipf,yvec[1]+yhipl,zvec[1],-yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=1.0
    joint=4
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=1000,maxVelocity=spd)
    joint=5
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=1000,maxVelocity=vvec[1])
    joint=6
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=1000,maxVelocity=vvec[1])

    a=xyztoang(xvec[2]-xhipb,yvec[2]-yhipl,zvec[2],yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=1.0
    joint=8
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=1000,maxVelocity=spd)
    joint=9
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=1000,maxVelocity=vvec[2])
    joint=10
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=1000,maxVelocity=vvec[2])

    a=xyztoang(xvec[3]-xhipb,yvec[3]+yhipl,zvec[3],-yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=1.0
    joint=12
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=1000,maxVelocity=spd)
    joint=13
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=1000,maxVelocity=vvec[3])
    joint=14
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=1000,maxVelocity=vvec[3])

#Pre-init robot position
setlegsxyz([xhipf,xhipf,xhipb,xhipb],[yhipl+0.1,-yhipl-0.1,yhipl+0.1,-yhipl-0.1],[-0.5,-0.5,-0.5,-0.5],[1,1,1,1])
t0=time.time()
t=time.time()
while ((t-t0)<4):
    t=time.time()


#Rotation matrix for yaw only between robot-frame and world-frame
def RotYawr(yawr):
    Rhor=np.array([[np.cos(yawr),-np.sin(yawr),0], [np.sin(yawr),np.cos(yawr),0], [0,0,1]])
    return Rhor

#Init robot position, orientation and pose params
# O means in world-centered coordinates
# R means in robot-centered coordinates
# r is for "of the robot"
# i is initial
yawri=1.3
xrOi=np.array([1,1,0.5])
legsRi=np.array([[xhipf,xhipf,xhipb,xhipb],
               [yhipl+0.1,-yhipl-0.1,yhipl+0.1,-yhipl-0.1],
               [-0.5,-0.5,-0.5,-0.5]])
#Set body to the robot pos
xbOi=xrOi
#Init body position and orientation
quat=p.getQuaternionFromEuler([0,0,yawri])
p.resetBasePositionAndOrientation(dog,xbOi,quat)
#Init leg abs pos
Ryawri=RotYawr(yawri)
legsO=(np.dot(Ryawri,legsRi).T + xbOi).T   #Apply rotation plus translation

#Set the non-initial variables and matrix
yawr=yawri
xrO=xrOi
xbO=xrO
Ryawr=RotYawr(yawri)

#Recalc leg rel pos in robot frame and set the legs
dlegsO=(legsO.T-xbO).T
dlegsR=np.dot(Ryawr.T,dlegsO)
setlegsxyz(dlegsR[0],dlegsR[1],dlegsR[2],[1,1,1,1])

#Calculate a new robot center position from the average of the feet positions
#Calculate a new robot yaw ditrection also from the feet positions
xfO=(legsO[:,0]+legsO[:,1])/2.0
xbO=(legsO[:,2]+legsO[:,3])/2.0
xrOn=(xfO+xbO)/2.0 + np.array([0,0,0.5])
xfmbO=xfO-xbO
yawrn=np.arctan2(xfmbO[1],xfmbO[0])

#Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
cyaw=10
cpitch=-15
cdist=1.5

#Walking speed (changes the walking loop time)
walkLoopSpd=400

#Change general motor speed
vvec=[12]*4

#Current leg to change position
l=0
#Init the center for the robot rotation to the current robot pos
xrcO=xrO
#Set the body position to the robot position
xoff=0
yoff=0
#Init to walking fwd
dr=0
drp=0
#Leg sequence (for rotating the robot, I chose to chg legs in the order front-left, fr, br, bl)
lseq=[0,1,3,2]
lseqp=[0,1,3,2]
#lseq=[2,0,3,1]
#lseqp=[2,0,3,1]

while (1):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(dog)
    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)

    keys = p.getKeyboardEvents()
    #Keys to change camera
    if keys.get(100):  #D
        cyaw+=1
    if keys.get(97):   #A
        cyaw-=1
    if keys.get(99):   #C
        cpitch+=1
    if keys.get(102):  #F
        cpitch-=1
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01
    #Keys to change the robot walk (fwd, bkw, rot right, rot left)
    if keys.get(65297): #Up
        drp=0
    if keys.get(65298): #Down
        drp=2
    if keys.get(65296): #Right
        drp=1
        xrcO=xrO        #Set the center for the robot rotation to the current robot pos
        lseqp=[1,0,2,3] #Change the leg sequence to open up the front arms rather than close them
    if keys.get(65295): #Left
        drp=3
        xrcO=xrO
        lseqp=[0,1,3,2] #Change the leg sequence to open up the front arms rather than close them

    #Time cycle
    tv=int(((time.time()-t0)*walkLoopSpd)  % 800)
    #One leg movement in 200 units. one 4-leg walk cycle in 800 units
    #Using <, >, % (modulo) and divide we can easily do something in a specific part of the cycle
    
    #Apply new walking cycle type (e.g. chg from fwd to bkw) only at the start of next cycle
    if tv<20 and (not dr==drp):
        dr=drp
        lseq=lseqp
    
    #Index of the leg to move
    l=int(tv/200)
    #Actual leg to move
    k=lseq[l]
       
    #In the beginning of the leg cycle the body is centered at the robot center
    #then it gradually moves in the opposite direction of the leg to be moved 
    #to ensure the center of gravity remains on the other 3 legs
    #when the moving leg goes down again the body center returns to the robot center
    #The vars xoff and yoff move the body w.r.t. the robot center in the robot frame
    if int(tv%200)<10:
        xoff=0
        yoff=0
    elif int(tv%200)<80:
        xoff+=0.002*(-1+2*int(k/2))  #Work it out on paper to see it moves opposite to the stepping leg
        yoff+=0.002*(-1+2*(k%2))     

    elif int(tv%200)>160:
        xoff-=0.004*(-1+2*int(k/2))
        yoff-=0.004*(-1+2*(k%2))     

    #Recalc leg rel pos in desired robot frame
    dlegsO=(legsO.T-xrO).T  #Translate
    dlegsR=np.dot(Ryawr.T,dlegsO)  #Rotate (Note the inverse rotation is the transposed matrix)
    #Then apply the body movement and set the legs
    setlegsxyz(dlegsR[0]-xoff-0.03,dlegsR[1]-yoff,dlegsR[2],vvec)  #0.03 is for tweaking the center of grav.
    

    if int(tv%200)>80:
        dlegsO=(legsO.T-xrcO).T
        yawlO=np.arctan2(dlegsO[1,k],dlegsO[0,k])
        rlO=np.sqrt(dlegsO[0,k]**2+dlegsO[1,k]**2)
        
        if dr==0:
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]+0.01*np.cos(yawr)
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]+0.01*np.sin(yawr)
        elif dr==1:
            yawlO-=0.015 
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]
        elif dr==2:
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]-0.01*np.cos(yawr)
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]-0.01*np.sin(yawr)
        elif dr==3:
            yawlO+=0.015 
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]
        
        if int(tv%200)<150:
            #Move leg k upwards 
            legsO[2,k]+=.006
        else:
            #Move leg k wards 
            legsO[2,k]-=.006
    else:
        #Move/keep all legs down to the ground
        legsO[2,0]=0.0
        legsO[2,1]=0.0
        legsO[2,2]=0.0
        legsO[2,3]=0.0
        
    
    #Calculate vectors and matrix for the next loop
    xfrO=(legsO[:,0]+legsO[:,1])/2.0
    xbkO=(legsO[:,2]+legsO[:,3])/2.0
    xrO=(xfrO+xbkO)/2.0 
    xrO[2]=0.5
    xfmbO=xfrO-xbkO
    yawr=np.arctan2(xfmbO[1],xfmbO[0])
    Ryawr=RotYawr(yawr)

    time.sleep(0.01)
	
p.disconnect()
    






















