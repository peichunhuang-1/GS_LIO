docker run --rm -it -v $(pwd):/ros2_ws/src/gs-lio --mount type=bind,source=/dev/shm,target=/dev/shm --network host --shm-size=1g -e DISPLAY=:10.0 -v /tmp/.X11-unix:/tmp/.X11-unix --ipc=host gs-lio 
