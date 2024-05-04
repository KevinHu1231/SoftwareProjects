import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
from tqdm import tqdm
import time
import scipy as sp

def load_point_cloud(path):
    # Load the point cloud data (do NOT change this function!)
    data = pd.read_csv(path, header=None)
    point_cloud = data.to_numpy()
    return point_cloud


def nearest_search(pcd_source,transf_pcd_source, pcd_target):
    # TODO: Implement the nearest neighbour search
    # TODO: Compute the mean nearest euclidean distance between the source and target point cloud

    corr_target = []
    corr_source = []
    ec_dist_mean = 0

    for i in range(pcd_source.shape[0]):
        org_source_pt = pcd_source[i,:]
        source_pt = transf_pcd_source[i,:]

        best_dist = np.inf
        best_pt = None

        for target_pt in pcd_target:
            dist = np.linalg.norm(source_pt-target_pt)
            if dist<best_dist:
                best_dist = dist
                best_pt = target_pt
        ec_dist_mean += best_dist
        corr_target.append(best_pt)
        corr_source.append(org_source_pt)
    
    ec_dist_mean = ec_dist_mean/pcd_source.shape[0]

    return corr_source, corr_target, ec_dist_mean

def skew(v):
    return np.array([[0,-v[2,0],v[1,0]],[v[2,0],0,-v[0,0]],[-v[1,0],v[0,0],0]])

def estimate_pose(corr_source, corr_target, Cop):
    # TODO: Compute the 6D pose (4x4 transform matrix)
    # TODO: Get the 3D translation (3x1 vector)

    # Iterative approach from State Estimation by Tim Barfoot
    corr_source_arr = np.stack(corr_source,axis=0).T
    corr_target_arr = np.stack(corr_target,axis=0).T
    w = corr_source_arr.shape[1]

    yj = corr_target_arr - corr_source_arr
    y = np.expand_dims(np.sum(yj, axis = 1)/w, axis = 1)
    pj = corr_target_arr
    p = np.expand_dims(np.sum(pj, axis = 1)/w, axis = 1)
    
    W = ((yj - y)@((pj - p).T))/w

    I = np.zeros((3,3))
    for i in range(w):
        Ii = skew(np.expand_dims(pj[:,i], axis=1) - p)@skew(np.expand_dims(pj[:,i], axis=1) - p)
        I = I + Ii
    I = -I/w

    b1 = np.trace(skew(np.array([[1],[0],[0]]))@Cop@(W.T))
    b2 = np.trace(skew(np.array([[0],[1],[0]]))@Cop@(W.T))
    b3 = np.trace(skew(np.array([[0],[0],[1]]))@Cop@(W.T))
    b = np.array([[b1],[b2],[b3]])
    phi_star = Cop@np.linalg.inv(I)@(Cop.T)@b
    Cnew = sp.linalg.expm(skew(phi_star))@Cop
    rnew = p - ((Cnew.T)@y)

    pose = np.identity(4)
    pose[:3,:3] = Cnew
    pose[:3,3] = rnew.reshape(3,)

    translation_x = rnew[0,0]
    translation_y = rnew[1,0]
    translation_z = rnew[2,0]

    return pose, translation_x, translation_y, translation_z

def icp(pcd_source, pcd_target):
    # TODO: Put all together, implement the ICP algorithm
    # TODO: Use your implemented functions "nearest_search" and "estimate_pose"
    # TODO: Run 30 iterations
    # TODO: Show the plot of mean euclidean distance (from function "nearest_search") for each iteration
    # TODO: Show the plot of pose translation (from function "estimate_pose") for each iteration
    
    pose = np.identity(4)
    
    n = pcd_source.shape[0]

    ec_dist_means = []
    xs = []
    ys = []
    zs = []
    iters = []

    for i in range(30):
        pcd_source_1 = np.vstack([pcd_source.T,np.ones((1,n))])
        transf_pcd_source_1 = pose @ pcd_source_1
        transf_pcd_source = transf_pcd_source_1[:3,:].T
        corr_source, corr_target, ec_dist_mean = nearest_search(pcd_source,transf_pcd_source,pcd_target)
        pose, tx, ty, tz = estimate_pose(corr_source, corr_target, pose[:3,:3])

        ec_dist_means.append(ec_dist_mean)
        xs.append(tx)
        ys.append(ty)
        zs.append(tz)
        iters.append(i)

        # # Transform the point cloud
        # TODO: Replace the ground truth pose with your computed pose and transform the source point cloud
        pts = np.vstack([np.transpose(pcd_source), np.ones(len(pcd_source))])
        cloud_registered = np.matmul(pose, pts)
        cloud_registered = np.transpose(cloud_registered[0:3, :])

        # # Visualize the point clouds after the registration
        ax = plt.axes(projection='3d')
        ax.scatter3D(cloud_registered[:,0], cloud_registered[:,1], cloud_registered[:,2], cmap='Greens')
        ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
        plt.legend(["Transformed Source Point Cloud", "Target Point Cloud"])
        ax.set_title('Point Clouds After Registration')
        plt.show()

    
    fig, ax = plt.subplots()
    ax.plot(iters,ec_dist_means)
    ax.set_title('Iteration and Corresponding Mean Euclidean Distance')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Mean Euclidean Distance (m)')
    plt.show()
    plt.savefig('euclidean_dist.png')

    fig, ax = plt.subplots()
    ax.plot(iters, xs, label='Translation X')
    ax.plot(iters, ys, label='Translation Y')
    ax.plot(iters, zs, label='Translation Z')
    ax.set_title('Iteration and Corresponding Translation')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Translation (m)')
    ax.legend()
    plt.show()
    plt.savefig('translation.png')

    return pose

def euler_from_rot(M):
    euler = np.zeros((3, 1))
    euler[0] = np.arctan2(M[2, 1], M[2, 2])
    s = -M[2, 0]
    c = np.sqrt(M[0, 0]*M[0, 0] + M[1, 0]*M[1, 0])
    if np.abs(c) > 1e-10:
      euler[1] = np.arctan2(s,c)
    else:
      euler[1] = np.pi/2
      if s < 0:
        euler[1] = -euler[1]
    euler[2] = np.arctan2(M[1, 0], M[0, 0])
    return euler

def main():
    start_time = time.time()
    
    # Dataset and ground truth poses
    #########################################################################################
    # Training and test data (3 pairs in total)
    train_file = ['bunny', 'dragon']
    test_file = ['armadillo']

    # Ground truth pose (from training data only, used for validating your implementation)
    GT_poses = []
    gt_pose = [0.8738,-0.1128,-0.4731,24.7571,
            0.1099,0.9934,-0.0339,4.5644,
            0.4738,-0.0224,0.8804,10.8654,
            0.0,0.0,0.0,1.0]
    gt_pose = np.array(gt_pose).reshape([4,4])
    GT_poses.append(gt_pose)
    gt_pose = [0.7095,-0.3180,0.6289,46.3636,
            0.3194,0.9406,0.1153,3.3165,
            -0.6282,0.1191,0.7689,-6.4642,
            0.0,0.0,0.0,1.0]
    gt_pose = np.array(gt_pose).reshape([4,4])
    GT_poses.append(gt_pose)
    #########################################################################################

    # Training (validate your algorithm)
    ##########################################################################################################
    for i in range(2):
        # Load data
        path_source = './training/' + train_file[i] + '_source.csv'
        path_target = './training/' + train_file[i] + '_target.csv'
        pcd_source = load_point_cloud(path_source)
        pcd_target = load_point_cloud(path_target)
        gt_pose_i = GT_poses[i]

        # Visualize the point clouds before the registration
        ax = plt.axes(projection='3d')
        ax.scatter3D(pcd_source[:,0], pcd_source[:,1], pcd_source[:,2], cmap='Greens')
        ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
        plt.legend(["Source Point Cloud" , "Target Point Cloud"])
        ax.set_title('Point Clouds Before Registration')
        plt.show()

        # TODO: Use your implemented ICP algorithm to get the estimated 6D pose (from source to target point cloud)
        
        pose = icp(pcd_source, pcd_target)

        # Transform the point cloud
        # TODO: Replace the ground truth pose with your computed pose and transform the source point cloud
        pts = np.vstack([np.transpose(pcd_source), np.ones(len(pcd_source))])
        cloud_registered = np.matmul(pose, pts)
        cloud_registered = np.transpose(cloud_registered[0:3, :])

        # TODO: Evaluate the rotation and translation error of your estimated 6D pose with the ground truth pose
        euler_gt = euler_from_rot(gt_pose_i[:3,:3])
        euler_pose = euler_from_rot(pose[:3,:3])
        pose_diff = np.linalg.norm(euler_gt - euler_pose)
        translation_diff = np.linalg.norm(gt_pose_i[:3,3] - pose[:3,3])

        #print("Euler Angle Rotation Error Norm: ", pose_diff)
        #print("Translation Error Norm: ", translation_diff)

        # Visualize the point clouds after the registration
        ax = plt.axes(projection='3d')
        ax.scatter3D(cloud_registered[:,0], cloud_registered[:,1], cloud_registered[:,2], cmap='Greens')
        ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
        plt.legend(["Transformed Source Point Cloud", "Target Point Cloud"])
        ax.set_title('Point Clouds After Registration')
        plt.show()

        ##########################################################################################################

    # # Test
    # ####################################################################################
    # for i in range(1):
    #     # Load data
    #     path_source = './test/' + test_file[i] + '_source.csv'
    #     path_target = './test/' + test_file[i] + '_target.csv'
    #     pcd_source = load_point_cloud(path_source)
    #     pcd_target = load_point_cloud(path_target)

    #     # Visualize the point clouds before the registration
    #     ax = plt.axes(projection='3d')
    #     ax.scatter3D(pcd_source[:,0], pcd_source[:,1], pcd_source[:,2], cmap='Greens')
    #     ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
    #     plt.legend(["Source Point Cloud" , "Target Point Cloud"])
    #     ax.set_title('Point Clouds Before Registration')
    #     plt.show()

    #     # TODO: Use your implemented ICP algorithm to get the estimated 6D pose (from source to target point cloud)
    #     pose = icp(pcd_source, pcd_target)

    #     # TODO: Show your outputs in the report
    #     # TODO: 1. Show your estimated 6D pose (4x4 transformation matrix)
    #     print("Estimated Pose: ", pose)
    #     # TODO: 2. Visualize the registered point cloud and the target point cloud

    #     pts = np.vstack([np.transpose(pcd_source), np.ones(len(pcd_source))])
    #     cloud_registered = np.matmul(pose, pts)
    #     cloud_registered = np.transpose(cloud_registered[0:3, :])

    #     ax = plt.axes(projection='3d')
    #     ax.scatter3D(cloud_registered[:,0], cloud_registered[:,1], cloud_registered[:,2], cmap='Greens')
    #     ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
    #     plt.legend(["Transformed Source Point Cloud", "Target Point Cloud"])
    #     ax.set_title('Point Clouds After Registration')
    #     plt.show()
    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Elapsed time: {:.6f} seconds".format(elapsed_time))


if __name__ == '__main__':
    main()