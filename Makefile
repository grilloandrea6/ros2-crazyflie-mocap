build:
	docker build  . -t ros2_crazyflie

run:
	@if command -v xhost >/dev/null 2>&1; then xhost +local:docker; fi
	touch .bash_history
	docker run -it --rm --name ros2_crazyflie \
	--privileged \
	--env="DISPLAY" \
	--env="ROS_DOMAIN_ID=132" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="/home/$(USER)/.Xauthority:/root/.Xauthority" \
	--privileged \
	--volume="./ws:/root/ros2_ws" \
	--volume="./.bash_history:/root/.bash_history" \
	--network host \
	ros2_crazyflie

attach:
	docker exec -it ros2_crazyflie /bin/bash
