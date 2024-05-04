import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
from tqdm import tqdm
import time

def load_point_cloud(path):
    # Load the point cloud data (do NOT change this function!)
    data = pd.read_csv(path, header=None)
    point_cloud = data.to_numpy()
    return point_cloud


def nearest_search(pcd_source, pcd_target):
    # TODO: Implement the nearest neighbour search
    # TODO: Compute the mean nearest euclidean distance between the source and target point cloud

    corr_target = []
    corr_source = []
    ec_dist_mean = 0

    for source_pt in pcd_source:
        best_dist = np.inf
        best_pt = None
        for target_pt in pcd_target:
            dist = np.linalg.norm(source_pt-target_pt)
            if dist<best_dist:
                best_dist = dist
                best_pt = target_pt
        ec_dist_mean += best_dist
        corr_target.append(best_pt)
        corr_source.append(source_pt)
    
    ec_dist_mean = ec_dist_mean/pcd_source.shape[0]

    return corr_source, corr_target, ec_dist_mean


def estimate_pose(corr_source, corr_target):
    # TODO: Compute the 6D pose (4x4 transform matrix)
    # TODO: Get the 3D translation (3x1 vector)

    # Scalar-Weighted Point Cloud Alignment
    corr_source_arr = np.stack(corr_source,axis=0).T
    corr_target_arr = np.stack(corr_target,axis=0).T
    w = corr_source_arr.shape[1]
    source_centroid = np.expand_dims(np.sum(corr_source_arr,axis=1)/w, axis=0).T
    target_centroid = np.expand_dims(np.sum(corr_target_arr,axis=1)/w, axis=0).T

    source_diffs = np.subtract(corr_source_arr,source_centroid)
    target_diffs = np.subtract(corr_target_arr,target_centroid)

    W_DM = np.dot(target_diffs,source_diffs.T)/w
    V, _, UT = np.linalg.svd(W_DM)

    det = np.linalg.det(UT.T)*np.linalg.det(V)
    M = np.array([[1,0,0],[0,1,0],[0,0,det]])
    C_DM = V @ M @ UT
    t_DM_M = (-C_DM.T@target_centroid) + source_centroid

    t = -C_DM@t_DM_M

    pose = np.identity(4)
    pose[:3,:3] = C_DM
    pose[:3,3] = t.reshape(3,)

    #print("Pose: ", pose)

    translation_x = t[0,0]
    translation_y = t[1,0]
    translation_z = t[2,0]

    return pose, translation_x, translation_y, translation_z

def random_transform():
    # Generate a random 3x3 rotation matrix
    rotation_matrix = np.random.rand(3, 3)
    u, s, vh = np.linalg.svd(rotation_matrix)
    rotation_matrix = np.dot(u, vh)

    # Generate a random translation vector
    translation_vector = -70*np.ones((3, 1)) + 140*np.random.rand(3, 1)

    # Create the 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation_vector.flatten()

    return transformation_matrix

def icp(pcd_source, pcd_target, init_pose):
    # TODO: Put all together, implement the ICP algorithm
    # TODO: Use your implemented functions "nearest_search" and "estimate_pose"
    # TODO: Run 30 iterations
    # TODO: Show the plot of mean euclidean distance (from function "nearest_search") for each iteration
    # TODO: Show the plot of pose translation (from function "estimate_pose") for each iteration
    
    pose = init_pose
    
    n = pcd_source.shape[0]
    new_pcd_source = pcd_source

    ec_dist_means = []
    xs = []
    ys = []
    zs = []
    iters = []

    for i in range(30):
        new_pcd_source_1 = np.vstack([new_pcd_source.T,np.ones((1,n))])
        transf_pcd_source_1 = pose @ new_pcd_source_1
        new_pcd_source = transf_pcd_source_1[:3,:].T
        corr_source, corr_target, ec_dist_mean = nearest_search(new_pcd_source,pcd_target)
        pose, tx, ty, tz = estimate_pose(corr_source, corr_target)

        ec_dist_means.append(ec_dist_mean)
        xs.append(tx)
        ys.append(ty)
        zs.append(tz)
        iters.append(i)
    
    # fig, ax = plt.subplots()
    # ax.plot(iters,ec_dist_means)
    # ax.set_title('Iteration and Corresponding Mean Euclidean Distance')
    # ax.set_xlabel('Iteration')
    # ax.set_ylabel('Mean Euclidean Distance (m)')
    # plt.show()
    # plt.savefig('euclidean_dist.png')

    # fig, ax = plt.subplots()
    # ax.plot(iters, xs, label='Translation X')
    # ax.plot(iters, ys, label='Translation Y')
    # ax.plot(iters, zs, label='Translation Z')
    # ax.set_title('Iteration and Corresponding Translation')
    # ax.set_xlabel('Iteration')
    # ax.set_ylabel('Translation (m)')
    # ax.legend()
    # plt.show()
    # plt.savefig('translation.png')

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
    eval_funcs_20 = []
    eval_funcs_15 = []
    eval_funcs_10 = []
    eval_funcs_5 = []
    eval_funcs_1 = []
    pose_diffs = []
    translation_diffs = []
    rand_inits = []

    
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


    for iter in tqdm(range(500)):
        eval_func_20_i = 0
        eval_func_15_i = 0
        eval_func_10_i = 0
        eval_func_5_i = 0
        eval_func_1_i = 0
        pose_diff_i = 0
        translation_diff_i = 0
        rand_init = random_transform()
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
            # ax = plt.axes(projection='3d')
            # ax.scatter3D(pcd_source[:,0], pcd_source[:,1], pcd_source[:,2], cmap='Greens')
            # ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
            # plt.legend(["Source Point Cloud" , "Target Point Cloud"])
            # ax.set_title('Point Clouds Before Registration')
            #plt.show()

            # TODO: Use your implemented ICP algorithm to get the estimated 6D pose (from source to target point cloud)
            
            pose = icp(pcd_source, pcd_target, rand_init)

            # Transform the point cloud
            # TODO: Replace the ground truth pose with your computed pose and transform the source point cloud
            # pts = np.vstack([np.transpose(pcd_source), np.ones(len(pcd_source))])
            # cloud_registered = np.matmul(pose, pts)
            # cloud_registered = np.transpose(cloud_registered[0:3, :])

            # TODO: Evaluate the rotation and translation error of your estimated 6D pose with the ground truth pose
            euler_gt = euler_from_rot(gt_pose_i[:3,:3])
            euler_pose = euler_from_rot(pose[:3,:3])
            pose_diff = np.linalg.norm(euler_gt - euler_pose)
            translation_diff = np.linalg.norm(gt_pose_i[:3,3] - pose[:3,3])
            
            eval_func_20_i += 20*pose_diff + translation_diff
            eval_func_15_i += 15*pose_diff + translation_diff
            eval_func_10_i += 10*pose_diff + translation_diff
            eval_func_5_i += 5*pose_diff + translation_diff
            eval_func_1_i += pose_diff + translation_diff
            pose_diff_i += pose_diff
            translation_diff_i += translation_diff

            #print("Euler Angle Rotation Error Norm: ", pose_diff)
            #print("Translation Error Norm: ", translation_diff)

            # Visualize the point clouds after the registration
            # ax = plt.axes(projection='3d')
            # ax.scatter3D(cloud_registered[:,0], cloud_registered[:,1], cloud_registered[:,2], cmap='Greens')
            # ax.scatter3D(pcd_target[:,0], pcd_target[:,1], pcd_target[:,2], cmap='Reds')
            # plt.legend(["Transformed Source Point Cloud", "Target Point Cloud"])
            # ax.set_title('Point Clouds After Registration')
            #plt.show()

        eval_funcs_20.append(eval_func_20_i)
        eval_funcs_15.append(eval_func_15_i)
        eval_funcs_10.append(eval_func_10_i)
        eval_funcs_5.append(eval_func_5_i)
        eval_funcs_1.append(eval_func_1_i)
        pose_diffs.append(pose_diff_i)
        translation_diffs.append(translation_diff_i)
        rand_inits.append(rand_init)
        ##########################################################################################################

    eval_funcs_20_np = np.array(eval_funcs_20)
    eval_funcs_15_np = np.array(eval_funcs_15)
    eval_funcs_10_np = np.array(eval_funcs_10)
    eval_funcs_5_np = np.array(eval_funcs_5)
    eval_funcs_1_np = np.array(eval_funcs_1)
    pose_diffs_np = np.array(pose_diffs)
    translation_diffs_np = np.array(translation_diffs)

    eval_funcs_20_sort = np.argsort(eval_funcs_20_np)
    eval_funcs_15_sort = np.argsort(eval_funcs_15_np)
    eval_funcs_10_sort = np.argsort(eval_funcs_10_np)
    eval_funcs_5_sort = np.argsort(eval_funcs_5_np)
    eval_funcs_1_sort = np.argsort(eval_funcs_1_np)
    pose_diffs_sort = np.argsort(pose_diffs_np)
    translation_diffs_sort = np.argsort(translation_diffs_np)

    poses_np = np.stack(rand_inits,axis=0)

    eval_funcs_20_sorted = eval_funcs_20_np[eval_funcs_20_sort]
    eval_funcs_15_sorted = eval_funcs_15_np[eval_funcs_15_sort]
    eval_funcs_10_sorted = eval_funcs_10_np[eval_funcs_10_sort]
    eval_funcs_5_sorted = eval_funcs_5_np[eval_funcs_5_sort]
    eval_funcs_1_sorted = eval_funcs_1_np[eval_funcs_1_sort]
    pose_diffs_sorted = pose_diffs_np[pose_diffs_sort]
    translation_diffs_sorted = translation_diffs_np[translation_diffs_sort]

    poses_eval_funcs_20 = np.take(poses_np, eval_funcs_20_sort, axis=0)
    poses_eval_funcs_15 = np.take(poses_np, eval_funcs_15_sort, axis=0)
    poses_eval_funcs_10 = np.take(poses_np, eval_funcs_10_sort, axis=0)
    poses_eval_funcs_5 = np.take(poses_np, eval_funcs_5_sort, axis=0)
    poses_eval_funcs_1 = np.take(poses_np, eval_funcs_1_sort, axis=0)
    poses_pose_diffs = np.take(poses_np, pose_diffs_sort, axis=0)
    poses_translation_diffs = np.take(poses_np, translation_diffs_sort, axis=0)

    poses_eval_funcs_20_list = list(poses_eval_funcs_20)
    poses_eval_funcs_15_list = list(poses_eval_funcs_15)
    poses_eval_funcs_10_list = list(poses_eval_funcs_10)
    poses_eval_funcs_5_list = list(poses_eval_funcs_5)
    poses_eval_funcs_1_list = list(poses_eval_funcs_1)
    poses_pose_diffs_list = list(poses_pose_diffs)
    poses_translation_diffs_list = list(poses_translation_diffs)

    output_files = ["20.txt","15.txt","10.txt","5.txt","1.txt","pose_diff.txt","translations_diff.txt"]
    array_lists = [poses_eval_funcs_20_list,poses_eval_funcs_15_list,poses_eval_funcs_10_list,poses_eval_funcs_5_list,poses_eval_funcs_1_list,poses_pose_diffs_list,poses_translation_diffs_list]
    eval_list = [eval_funcs_20_sorted,eval_funcs_15_sorted,eval_funcs_10_sorted,eval_funcs_5_sorted,eval_funcs_1_sorted,pose_diffs_sorted,translation_diffs_sorted]
    
    # Open the file for writing
    for idx in range(7):
        output_file = output_files[idx]
        array_list = array_lists[idx]
        data = eval_list[idx]
        with open(output_file, "w") as file:
            for array in array_list:
                # Use numpy.savetxt to save each array to the file
                np.savetxt(file, array, delimiter='\t', fmt='%0.10f')
                file.write('\n')  # Add a newline to separate arrays
            np.savetxt(file, data, delimiter=', ', fmt='%0.10f')

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