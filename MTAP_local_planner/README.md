# MTAP — Memory-based Trajectory and Action Planning
This repository contains the code of the end-to-end autonomous driving approach presented in [End-To-End Imitation Learning for Socially Aware Navigation using Memory-based Trajectory and Action Planning](https://aurova-projects.github.io/MTAP/). Here are the instructions of training and using the network, to use it with the CARLA simulator and the presented socially aware benchmark, follow the instructions of [app_MTAP](https://github.com/AUROVA-LAB/applications/tree/main/app_MTAP).

Firstly, build the docker image:
```
docker build -t aurova_mtap .
```

## Training
Run the docker, setting your path to this repository and the dataset:
```
docker run --gpus all --rm -it --shm-size=500gb --net=host --ulimit memlock=-1 --ulimit stack=67108864 -v /path/to/MTAP_local_planner:/MTAP_local_planner -v /path/to/CARLA_dataset:/dataset --name aurova_mtap aurova_mtap
```

Start training the neural network:
```
cd MTAP_local_planner
python train.py
```

To see all the training options, use `python train.py --help`

## Executing the agent
To start the MTAP agent, run in previous docker container:

```
python agent.py /path/to/weights.pth 
```

The agent use zmq to communicate with the [listener_agent.py](https://github.com/AUROVA-LAB/applications/blob/main/app_MTAP/scripts/listener_agent.py) and control the robot in the CARLA simulator.
