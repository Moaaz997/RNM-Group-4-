import numpy as np
import cv2 as cv

# Creating counters for moving over the lines of the poses.txt and trk.tx files
counter_g2b=0
counter_t2c=0

# Creating lists of the 40 Transforms for both bast to gripper and camera to target
H_t2k=[]
H_g2b=[]

# Creating arrays of the rotation matrices and translation vectors for the gripper to base and the target 2 camera 
r_g2b_l=[] 
t_g2b_l=[]
r_t2c_l=[]
t_t2c_l=[]

# A function that would return a rotation matric and a translation vector from the txt files (the function uses the above mentioned counter to read the following lines properly and to not repeat itself).
def file2Arrays(file_name:str):
    if file_name=="pose.txt":
        flag=False
        global counter_g2b
        counter=counter_g2b
        counter_g2b+=1
    elif file_name=="trk.txt":
        flag=True
        global counter_t2c
        counter=counter_t2c
        counter_t2c+=1
    f=open(file_name,"r")
    l_list=f.readlines()
    l_str=l_list[counter].split()
    l=list(map(lambda x:float(x),l_str))
    mat=np.array(l)
    mat=mat.reshape((4,4))
    if flag:
        mat=np.linalg.inv(mat)
        H_t2k.append(mat)
    else:
        H_g2b.append(mat)
    r=mat[0:3,0:3]
    t=mat[0:3,3]
    t=t.reshape(-1,1)
    # print("counter: ", counter+1)
    counter+=1
    return r, t

def main():
    for i in range(0,40):
        r_g2b, t_g2b=file2Arrays("pose.txt")
        r_g2b_l.append(r_g2b)
        t_g2b_l.append(t_g2b)
        # print("Rotation Matrix: \n",r_g2b)
        # print("Translation Vector: \n",t_g2b)
        r_t2c, t_t2c=file2Arrays("trk.txt")
        r_t2c_l.append(r_t2c)
        t_t2c_l.append(t_t2c)
        # print("Rotation Matrix: \n",r_t2c)
        # print("Translation Vector: \n",t_t2c)
    row=np.array([0,0,0,1])
    r_result, t_result=cv.calibrateHandEye(r_g2b_l,t_g2b_l, r_t2c_l,t_t2c_l)
    merged_result=np.c_[r_result,t_result]
    Transform=np.vstack([merged_result,row])
    print("End effector to camera transformation is: \n", Transform)
    # Transform_t2b=np.matmul(H_g2b[0], Transform, H_t2k[0])
    # print("Target to base transformation is: \n", Transform_t2b)


if __name__=="__main__":
    main()