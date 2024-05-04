import rclpy
import numpy as np
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.point_subscription = self.create_subscription(
            PoseArray,
            'Fixed_World_Points',
            self.fixed_point_callback,
            10)
        self.point_subscription  # prevent unused variable warning

        self.pose_array_pub = self.create_publisher(PoseArray,'Moved_World_Points', 10)

        self.poses_list = []
        self.got_poses = False
        self.deform_transform = None
        self.deform_transform_update = 0
        self.moved_update = 0
        self.x_bound = 256
        self.y_bound = 144
        self.world_to_img_rot = np.array([[r1,r2,r3],[r4,r5,r6],[r7,r8,r9]])
        self.world_to_img_trans = np.array([[t1],[t2],[t3]])
        self.world_to_img = np.hstack((self.world_to_img_rot,self.world_to_img_trans))
        self.world_to_img_full = np.vstack((self.world_to_img, np.array([[0,0,0,1]])))
        
        self.img_to_world = np.linalg.inv(self.world_to_img_full)
        self.img_to_world = self.img_to_world[:3,:]

        self.fixed_image = None
        self.moving_image = None
        self.vxm_model = None

    def fixed_world_point_callback(self, msg):
        self.poses_list.append(msg)
        self.got_poses = True

    def fixed_world_to_moved_image(self):
        if (self.moved_update < self.deform_transform_update) and self.got_poses == True:
            for poses in self.poses_list:
                name = poses.header.frame_id
                moved_poses_msg = PoseArray()

                for pose in poses.poses:
                    point = np.array([[pose.position.x],[pose.position.y],[pose.position.z]])
                    trans_point = np.dot(self.world_to_img,point)
                    depth = trans_point[2,0]
                    trans_point = trans_point/depth
                    img_point = trans_point[:2,0]

                    if (0 <= img_point[0] <= self.x_bound):
                        x = int(img_point[0,0])
                        y = int(img_point[1,0])

                        displacement = self.deform_transform[y,x,:]
                        moved_img_point = np.array([[(x+displacement[0])*depth],[(y+displacement[1])*depth],[depth]])
                        moved_img_point_world = np.dot(self.img_to_world,moved_img_point)
                        moved_pose = Pose()
                        moved_pose.header.frame_id = name
                        moved_pose.position.x = moved_img_point_world[0,0]
                        moved_pose.position.y = moved_img_point_world[1,0]
                        moved_pose.position.z = moved_img_point_world[2,0]
                        moved_poses_msg.poses.append(moved_pose)
                    else:
                        moved_poses_msg.poses.append(pose)
                
                self.moved_update += 1
                moved_poses_msg.header.frame_id = name + "_moved_" + str(self.moved_update)
                self.pose_array_pub.publish(moved_poses_msg)

    def update_deform_transform(self):
        while rclpy.ok():
            self.deform_transform = self.vxm_model.predict(self.moving_img)
            self.deform_transform_update += 1
            

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # create a new thread
    update_transform = threading.Thread(target=minimal_subscriber.update_deform_transform)

    # start the thread
    update_transform.start()
    
    while rclpy.ok():
        minimal_subscriber.fixed_world_to_moved_image()
        rclpy.spin_once(minimal_subscriber)

    # join the thread (wait for it to complete)
    update_transform.join()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()