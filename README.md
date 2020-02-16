# Teaching agents to navigate in a map-less environment

<b> Authors: </b>  
Shivam Goel shivam.goel@wsu.edu  
Abhijay abhijay@pdx.edu  

Based on:  
* [erlebot/gym-gazebo](https://github.com/erlerobot/gym-gazebo)
* [Tf Pose Estimation](https://github.com/ildoonet/tf-pose-estimation)

**Table of Contents**<br>
1. [Getting Started](#getting-started)  
   a. Docker link  
   b. Requirements
2. [Running the experiments](#running-the-experiments)   
   a. Run docker  
   b. Clear all previously running processes  
   c. Experiment
3. [Visual on Gazebo](#visual-on-gazebo)
4. [Useful Docker Commands](#useful-docker-commands)

## Getting Started

<b>Docker link</b>

Wil be updated soon  

<b> Requirements </b>  
Host machine: Ubuntu 16, CUDA Version: 10.1  

### Running the experiments   

<b> Run the docker </b>

```
sudo docker run --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -it gym-gazebo_v6.1
```

<b> Clear all previously running processes </b>  

```
ps -ef | grep ros | awk '{print $2}' | xargs kill -9
ps -ef | grep python | awk '{print $2}' | xargs kill -9
ps -ef | grep xvfb | awk '{print $2}' | xargs kill -9
```

<b> Experiment </b> 

```
# Install conda
conda activate tf-gpu-1-5

# For entering docker  
# Open two different terminals and run  
sudo docker exec -it ef6dd4c9b971 bash  
# One is for running pose estimation and the other for dqn

# On terminal 1
cd /usr/local/gym/gym-gazebo/examples/scripts_turtlebot
run -xvfb
nohup python -u smarthome_turtlebot_lidar_dqn_withAttn_v3.py > results_withAtt_Feb16.out &

# On terminal 2 (simultaneously)
cd /usr/local/gym/tf-pose-estimation#
nohup python run_with_ros.py --model=mobilenet_thin > pose.out &

# Monitor results on terminal 1
tail -f results_withAtt_Feb16.out
```

### Visual on Gazebo
To view the robot acting in the environment using gzserver do the following  
```
export GAZEBO_ID=`sudo docker ps | grep gym-gazebo | awk '{print $1}'`
export GAZEBO_MASTER_IP=$(sudo docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $GAZEBO_ID)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345

# Note: With sudo (just copy and make sure the grep is able to get the name of the current docker)  

export GAZEBO_ID=`sudo docker ps | grep gym_gazebo | awk '{print $1}'`

gzclient
```

### Useful Docker Commands

Check running containers  

```
sudo docker ps -a
```

Start a running docker  
```
sudo docker start <container-id>
```
```
sudo docker attach <container-id>
```

Creating volume
```
docker volume create erlerobot_test_v1
sudo docker volume ls
docker volume rm my-vol
```

Commit docker changes  
```
sudo docker commit <container-id> gym-gazebo_v1.1
```

Create a compressed docker image to transfer to another system  
```
sudo docker save gym-gazebo_v1.1 > gym-gazebo_v1.1.tar.gz
```

Load a docker image  
```
sudo docker image load --input gym_gazebo_v1.2.0.tar.gz
```

Get files from the docker to hard disk  
```
sudo docker cp c6dbab3a187a:/data/turtlebotExperiments_v1/. turtlebotExperiments_v1/.
```
Vice Versa:
```
sudo docker cp models.zip 0cccde0d68ff:/usr/local/gym/
```

Removing docker
```
docker rm <container-id>
```


