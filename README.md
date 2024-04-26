# RSN-Final

# Final_project_5554

# dxf_to_pointcloud
Function:
This package is used to convert RGBD data into dense point cloud images and publish these point cloud data through ROS topics.

accomplish:
Get data by reading dxf.
Convert dxf data to pointclod2.
Publish dense point cloud data to the specified ROS topic.

What is read:
DXF dataset


Posted topics:
/pointcloud: Dense point cloud data

# pointcloud_viewer
Function:
Browse pointcloud2 msg in the specified topic.

Subscribed topics:
/pointcloud

# rgbd_to_pointcloud_pkg

Function:
Merge the color image and depth image in RGBDrosbag to generate a dense color point cloud image.

Subscribed topics:
rosbag

Posted topics:
/pointcloud
