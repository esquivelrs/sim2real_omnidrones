# sim2real_omnidrones


## Docker container:
Create image:

```
cd docker
make image
```

Run image:

```
bash docker-run.sh ./../ws
```

Open a new terminal:
```
docker exec -it dtu_sim2real bash
```

### Build ROS2 Packages

```
colcon build --symlink-install
```

Run simple task follow the instructions in: [link](https://imrclab.github.io/crazyswarm2/usage.html#python-scripts) with flow deck or for motion_capture: [link](https://imrclab.github.io/crazyswarm2/usage.html#positioning)



