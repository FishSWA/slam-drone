#相机校准
kalibr_calibrate_cameras --target ./april_s.yaml --bag ./camera_calration.bag --bag-from-to 05 30 --models pinhole-radtan pinhole-radtan  --topics /infra1_image_rect_ra/infra2_image_rect_raw --approx-sync 0.01
rosbag record -O camera_calibration /infra1_image_rect_raw /infra2_image_rect_raw

#话题重发布
rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 4.0 /infra1_image_rect_raw
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 4.0 /infra2_image_rect_raw

#录制cam-imu包
rosbag record -O camera_imu_calibration /infra1_image_rect_raw /infra2_image_rect_raw /mavros/imu/data_raw

#cam-imu 标定
kalibr_calibrate_imu_camera --target ./april_s.yaml  --cam camera_calibration-camchain.yaml --imu ./imu_param.yaml --bag ./camera_imu_calibration.bag  --bag-from-to 10 110 --max-iter 1
