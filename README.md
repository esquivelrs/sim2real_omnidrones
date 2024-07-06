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


# Build ROS2 Package
 build image:

 ```
 colcon build
 ```
