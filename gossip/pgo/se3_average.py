import numpy as np
from sophus import SO3,SE3
import g2o

def linearize_at(poses,linearized_point):
    #linearize se3 at linearizePoint
        #pose:se3 vector6
        #linearized_point: vector6

    linearized_poses = []
    r_ref = linearized_point[3:6]

    R_ref = SO3().exp(r_ref)
    R_ref_inv = R_ref.inverse().matrix()
    # print(R_ref_inv)
    # print(type(R_ref_inv))
    # print(dir(R_ref_inv))
    # print(poses.shape[0])
    for i in range(poses.shape[0]):
        pose_se3 = SE3().exp(poses[i,:])
        rot_se3 = pose_se3.rotationMatrix()
        rot_in_ref = np.dot(R_ref_inv,rot_se3)
        linearized_rot = SO3(rot_in_ref).log()
        linearized_pose = [0,0,0,0,0,0]
        linearized_pose[0:3] = poses[i,0:3]
        for j in range(3):
            linearized_pose[j+3] = linearized_rot[j][0]
        linearized_poses.append(linearized_pose)

    linearized_poses = np.array(linearized_poses)
    return linearized_poses

def update_pose(poses,delta_poses):
    new_poses = np.zeros(delta_poses.shape)
    for i in range(delta_poses.shape[0]):
        delta_trans = delta_poses[i,0:3]
        delta_rot = delta_poses[i,3:6]

        delta_Rot = SO3().exp(delta_rot).matrix()
        # print(poses[i,:])
        pose_se3 = SE3().exp(poses[i,:])
        new_Rot = np.dot(pose_se3.rotationMatrix(),delta_Rot)
        new_rot = SO3(new_Rot).log()

        new_trans = poses[i,0:3]+delta_trans
        new_rot = np.array([rot[0] for rot in new_rot])
        new_pose = np.concatenate((new_trans,new_rot),axis=0)
        new_poses[i,:] = new_pose
        # print(new_pose)
    
    return new_poses

def se3_average_at(new_poses_list,col_weight_list,local_poses,local_weight):
    delta_poses = np.zeros(local_poses.shape)
    print('local pose shape')
    print(delta_poses.shape)
    # print(delta_pose)
    for i in range(new_poses_list.shape[0]):
        new_pose = new_poses_list[i,:]
        col_weight = col_weight_list[i]
        # print(new_pose.shape)
        # print(local_pose.shape)
        linearized_poses = linearize_at(new_pose,local_poses)
        # print("--linear--")
        # print(linearized_poses.shape)
        # print(col_weight)
        # c = linearized_poses*col_weight
        # print(c.shape)
        # print(c)
        # print(linearized_poses*col_weight)
        delta_poses += linearized_poses*col_weight
    print('delta pose')
    print(delta_poses.shape)
    avg_pose = update_pose(local_poses,delta_poses)
    print('average pose')
    print(avg_pose.shape)
    return avg_pose