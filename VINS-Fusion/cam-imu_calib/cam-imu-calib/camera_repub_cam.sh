rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 4.0 /infra1_image_rect_raw &
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 4.0 /infra2_image_rect_raw &
wait