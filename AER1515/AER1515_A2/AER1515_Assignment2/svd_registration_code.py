import numpy as np
from matplotlib import pyplot as plt
import pandas as pd

def load_point_cloud(path):
    # Load the point cloud data (do NOT change this function!)
    data = pd.read_csv(path, header=None)
    point_cloud = data.to_numpy()
    return point_cloud


def nearest_search(pcd_source, pcd_target):
    # TODO: Implement the nearest neighbour search
    # TODO: Compute the mean nearest euclidean distance between the source and target point cloud

    # Initialize target and source list and set mean Euclidean distance to zero 
    corr_target = []
    corr_source = []
    ec_dist_mean = 0

    # Loop over source points
    for source_pt in pcd_source:
        # Initialize best distance and best point
        best_dist = np.inf
        best_pt = None
        # Loop over target points
        for target_pt in pcd_target:
            # Calculate Euclidean distance
            dist = np.linalg.norm(source_pt-target_pt)
            # If Euclidean distance is lower than the current best, update current best distance and target point
            if dist<best_dist:
                best_dist = dist
                best_pt = target_pt
        
        # Add best distance to total distance, add best point to target list, add source point to source list
        ec_dist_mean += best_dist
        corr_target.append(best_pt)
        corr_source.append(source_pt)

    # Divide total distance by number of source points to get Euclidean mean distance
    ec_dist_mean = ec_dist_mean/pcd_source.shape[0]

    return corr_source, corr_target, ec_dist_mean


def estimate_pose(corr_source, corr_target):
    # TODO: Compute the 6D pose (4x4 transform matrix)
    # TODO: Get the 3D translation (3x1 vector)

    # Point Cloud Alignment Algorithm

    # Get source and target arrays
    corr_source_arr = np.stack(corr_source,axis=0).T
    corr_target_arr = np.stack(corr_target,axis=0).T

    # Get cross covariance matrix H
    H = np.dot(corr_source_arr, corr_target_arr.T)
    
    # Perform SVD
    U, _, Vt = np.linalg.svd(H)

    # Get optimal rotation matrix
    R = np.dot(Vt.T, U.T)

    # Make sure rotation matrix has valid determinant of +1 instead of -1
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Get source and target centroids
    source_centroid = np.mean(corr_source_arr, axis=1)
    target_centroid = np.mean(corr_target_arr, axis=1)

    # Get optimal pose and translation
    t = target_centroid - np.dot(R, source_centroid)

    pose = np.identity(4)
    pose[:3,:3] = R
    pose[:3,3] = t

    translation_x = t[0]
    translation_y = t[1]
    translation_z = t[2]

    return pose, translation_x, translation_y, translation_z

# Get Euler angles from rotation matrix
def euler_from_rot(M):
    # Initialize Euler angle matrix
    euler = np.zeros((3, 1))
    # Get roll angle
    euler[0] = np.arctan2(M[2, 1], M[2, 2])
    s = -M[2, 0]
    c = np.sqrt(M[0, 0]*M[0, 0] + M[1, 0]*M[1, 0])
    # Check for Gimbal lock and get pitch angle
    if np.abs(c) > 1e-10:
      euler[1] = np.arctan2(s,c)
    else:
      euler[1] = np.pi/2
      if s < 0:
        euler[1] = -euler[1]
    #Get yaw angle
    euler[2] = np.arctan2(M[1, 0], M[0, 0])
    return euler

def icp(pcd_source, pcd_target):
    # TODO: Put all together, implement the ICP algorithm
    # TODO: Use your implemented functions "nearest_search" and "estimate_pose"
    # TODO: Run 30 iterations
    # TODO: Show the plot of mean euclidean distance (from function "nearest_search") for each iteration
    # TODO: Show the plot of pose translation (from function "estimate_pose") for each iteration
    
    # Initialize final pose and pose for current iteration
    pose = np.identity(4)
    pose_i = np.identity(4)
    
    # Get number of source points and convert to homogeneous coordinates
    n = pcd_source.shape[0]
    pcd_source_1 = np.vstack([pcd_source.T,np.ones((1,n))])

    # Initialize lists for plotting
    ec_dist_means = []
    xs = []
    ys = []
    zs = []

    euler1s = [] 
    euler2s = []
    euler3s = []
    iters = []

    # Add data for initial state
    print("Iteration: ", 0)
    euler = euler_from_rot(pose[:3,:3])
    xs.append(pose[0][3])
    ys.append(pose[1][3])
    zs.append(pose[2][3])
    euler1s.append(euler[0])
    euler2s.append(euler[1])
    euler3s.append(euler[2])
    iters.append(0)

    # Loop over 30 iterations
    for i in range(30):
        # Transform source points
        transf_pcd_source_1 = pose @ pcd_source_1
        # Set new source points to transformed source points
        new_pcd_source = transf_pcd_source_1[:3,:].T
        # Do nearest search for point correspondences
        corr_source, corr_target, ec_dist_mean = nearest_search(new_pcd_source,pcd_target)
        # Estimate pose transformation
        pose_i, tx, ty, tz = estimate_pose(corr_source, corr_target)
        # Calculate total pose transformation
        pose = pose_i @ pose

        #Add statistics to list
        euler = euler_from_rot(pose[:3,:3])
        ec_dist_means.append(ec_dist_mean)
        xs.append(pose[0][3])
        ys.append(pose[1][3])
        zs.append(pose[2][3])
        euler1s.append(euler[0])
        euler2s.append(euler[1])
        euler3s.append(euler[2])
        iters.append(i+1)
        print("Iteration: ", i+1)

    # Get point correspondence result and mean Euclidean distance for final iteration
    pcd_source_1 = np.vstack([pcd_source.T,np.ones((1,n))])
    transf_pcd_source_1 = pose @ pcd_source_1
    new_pcd_source = transf_pcd_source_1[:3,:].T
    corr_source, corr_target, ec_dist_mean = nearest_search(new_pcd_source,pcd_target)
    ec_dist_means.append(ec_dist_mean)

    # Plot mean Euclidean distance for each iteration
    fig, ax = plt.subplots()
    ax.plot(iters,ec_dist_means)
    ax.set_title('Iteration and Corresponding Mean Euclidean Distance')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Mean Euclidean Distance (m)')
    plt.show()

    # Plot translation for each iteration

    fig, ax = plt.subplots()
    ax.plot(iters, xs, label='Translation X')
    ax.plot(iters, ys, label='Translation Y')
    ax.plot(iters, zs, label='Translation Z')
    ax.set_title('Iteration and Corresponding Translation')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Translation (m)')
    ax.legend()
    plt.show()

    # Plot Euler angles for each iteration

    fig, ax = plt.subplots()
    ax.plot(iters, euler1s, label='Roll')
    ax.plot(iters, euler2s, label='Pitch')
    ax.plot(iters, euler3s, label='Yaw')
    ax.set_title('Iteration and Corresponding Orientation in Euler Angles')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Angle (rad)')
    ax.legend()
    plt.show()

    return pose

def main():
    
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

        # Get ground truth and computed pose Euler angles
        euler_gt = euler_from_rot(gt_pose_i[:3,:3])
        euler_pose = euler_from_rot(pose[:3,:3])

        # Calculate pose difference using the norm and difference
        pose_diff_norm = np.linalg.norm(euler_gt - euler_pose)
        pose_diff = euler_gt - euler_pose

        # Calculate translation difference using norm and difference
        translation_diff_norm = np.linalg.norm(gt_pose_i[:3,3] - pose[:3,3])
        translation_diff = gt_pose_i[:3,3] - pose[:3,3]

        print("Euler Angle Rotation Error Norm: ", pose_diff_norm)
        print("Euler Angle Rotation Error: ", pose_diff)
        print("Translation Error Norm: ", translation_diff_norm)
        print("Translation Error: ", translation_diff)


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
    for i in range(1):
        # Load data
        path_source = './test/' + test_file[i] + '_source.csv'
        path_target = './test/' + test_file[i] + '_target.csv'
        pcd_source = load_point_cloud(path_source)
        pcd_target = load_point_cloud(path_target)

        # Visualize the point clouds before the registration
        ax = plt.axes(projection='3d')
        ax.scatter3D(pcd_source[:,0], pcd_source[:,1], pcd_source[:,2], cmap='Greens')
        ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
        plt.legend(["Source Point Cloud" , "Target Point Cloud"])
        ax.set_title('Point Clouds Before Registration')
        plt.show()

        # TODO: Use your implemented ICP algorithm to get the estimated 6D pose (from source to target point cloud)
        pose = icp(pcd_source, pcd_target)

        # TODO: Show your outputs in the report
        # TODO: 1. Show your estimated 6D pose (4x4 transformation matrix)
        print("Estimated Pose: ", pose)
        # TODO: 2. Visualize the registered point cloud and the target point cloud

        pts = np.vstack([np.transpose(pcd_source), np.ones(len(pcd_source))])
        cloud_registered = np.matmul(pose, pts)
        cloud_registered = np.transpose(cloud_registered[0:3, :])

        ax = plt.axes(projection='3d')
        ax.scatter3D(cloud_registered[:,0], cloud_registered[:,1], cloud_registered[:,2], cmap='Greens')
        ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
        plt.legend(["Transformed Source Point Cloud", "Target Point Cloud"])
        ax.set_title('Point Clouds After Registration')
        plt.show()


if __name__ == '__main__':
    main()