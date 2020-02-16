# teaching agents to navigate in a map-less environment

## Getting Started

### Docker: How to check existing running algorithm

```
sudo docker ps -a
```

```
sudo docker attach <container ID>
```

```
sudo docker attach <container ID>
```

```
tmux a
```

### Docker: How to run the code

Make sure you are on the correct environment

```
$ conda activate tf-gpu-1-5
```
```
open a tmux session
$ tmux
$ ctrl+b
%
cd /usr/local/gym/gym-gazebo/examples/scripts_turtlebot

run -xvfb

python smarthome_turtlebot_lidar_dqn.py

on another terminal

$ cd /usr/local/gym/tf-pose-estimation#

$ python run_with_ros.py --model=mobilenet_thin

```


##### Get the docker:

```
sudo docker pull erlerobotics/gym-gazebo:latest
```

#### Run the docker:

```
sudo docker run -it erlerobotics/gym-gazebo
```

https://medium.com/the-code-review/top-10-docker-run-command-options-you-cant-live-without-a-reference-d256834e86c1


#### Create a volume:
```
docker volume create erlerobot_test_v1
```

```
sudo docker volume ls
```

```
docker volume rm my-vol
```

https://docs.docker.com/engine/reference/commandline/run/
https://docs.docker.com/storage/volumes/

#### Running volume with docker:

```
sudo docker run -it -v erlerobot_test_v1:/data gym-gazebo_v1.1
```

#### List dockers:

```
sudo docker image ls
```

#### List started/running dockers:
```
sudo docker ps -a
```

#### Start a running docker:
```
sudo docker start 5cc0c631786d
```
```
sudo docker attach 5cc0c631786d
```
#### Get details of the docker:

```
sudo docker info
```
#### Docker locations (maybe):

```
sudo ls /var/lib/docker
```

#### Removing a docker image from disk:
```
sudo docker rmi nvidia/cuda:8.0
```
```
sudo docker image ls
```
#### For starting another terminal:
```
sudo docker exec -it c6dbab3a187a bash
```

```
sudo docker exec -it <container-id> bash
```


### Commiting a docker:
```
$ sudo docker commit 5cc0c631786d gym-gazebo_v1.1
```
Create a compressed docker image to transfer to another system:
```
$ sudo docker save gym-gazebo_v1.1 > gym-gazebo_v1.1.tar.gz
```


### How to load a docker from images
```
sudo docker image load --input gym_gazebo_v1.2.0.tar.gz
```

Then check `$ sudo docker images`

After checking now run the docker `$ sudo docker run -it gym_gazebo_v1.2.0`
Please store run commands in a readme file

### To get files from the docker to hard disk:
```
$ sudo docker cp c6dbab3a187a:/data/turtlebotExperiments_v1/. turtlebotExperiments_v1/.
```
Vice Versa:
```
$ sudo docker cp models.zip 0cccde0d68ff:/usr/local/gym/
```


### For removing a docker which is not running

Check dockers using `$ docker ps -a`
Remove using `$ docker rm image_id`

### To view the robot acting in the environment using gzserver do the following:
```
$ export GAZEBO_ID=`sudo docker ps | grep gym-gazebo | awk '{print $1}'`
```
```
$ export GAZEBO_MASTER_IP=$(sudo docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $GAZEBO_ID)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
```
Note: With sudo (just copy and make sure the grep is able to get the name of the current docker)
```
$ export GAZEBO_ID=`sudo docker ps | grep gym_gazebo | awk '{print $1}'`
```


Then run
```
$ gzclient
```
