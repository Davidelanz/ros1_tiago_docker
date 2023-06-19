# Docker enabled TIAGo++ ROS workspace

[![Ubuntu Bionic](https://img.shields.io/badge/Ubuntu-bionic-blue)](https://releases.ubuntu.com/18.04/)
[![ROS Melodic](https://img.shields.io/badge/ROS-melodic-blue)](http://wiki.ros.org/melodic)
[![Robot TIAGo++](https://img.shields.io/badge/Robot-TIAGo++-blue)](https://wiki.ros.org/Robots/TIAGo++)

> Based on https://github.com/Davidelanz/ros1_gazebo_docker

To spin up the development environment (use `--build` to freshly build the docker image):
```sh
docker compose up
```

The noVNC GUI is available at `http://localhost:8080/vnc.html`


To launch a `.launch` file, e.g. `tiago_dual_gazebo`:
```sh
docker exec -it ros_tiago_docker-catkin_ws-1 bash -c '\
    source /opt/ros/melodic/setup.bash \
    && source /root/catkin_ws/devel/setup.bash \
    && roslaunch \
        tiago_dual_gazebo \
        tiago_dual_gazebo.launch \
        public_sim:=true \
        end_effector_left:=pal-gripper \
        end_effector_right:=pal-gripper \
        world:=tutorial_office \
        gzpose:="-x 1.40 -y -2.79 -z -0.003 -R 0.0 -P 0.0 -Y 0.0" \
        use_moveit_camera:=true'
```

To launch `rviz`:
```sh
docker exec -it ros_tiago_docker-catkin_ws-1 '\
    source /opt/ros/melodic/setup.bash \
    && source /root/catkin_ws/devel/setup.bash \
    && rosrun rviz rviz'
```

To launch `rqt_graph`:
```sh
docker exec -it ros_tiago_docker-catkin_ws-1 '\
    source /opt/ros/melodic/setup.bash \
    && source /root/catkin_ws/devel/setup.bash \
    && rosrun rqt_graph rqt_graph'
```

To add a new package, add the respective volume in the `docker-compose.yml` file:
```yml
volumes:
    - ./catkin_ws/src/<package_name>:/root/catkin_ws/src/<package_name>
```
