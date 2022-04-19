Header header
geometry_msgs/Pose map_pose #map上での前方車両位置
geometry_msgs/Twist velocity_mps #前方車両の絶対速度
float64 distance #自車両からの距離
geometry_msgs/Point rel_pose_ave #直近データの相対位置平均
geometry_msgs/Point map_pose_ave #直近データの絶対位置平均
geometry_msgs/Point rel_pose_var_ave #直近データの相対位置分散
derived_object_msgs/ObjectWithCovariance orig_data
