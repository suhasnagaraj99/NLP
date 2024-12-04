# NLP

## Training and Evaluation

- Please use the `nlp_train.py` to train the model. Please ensure correct path for dataset.
- The model weights will get saved in `trained_model` folder.
- Please use the `eval.py` to get the validation accuracy. Please ensure correct path for dataset and trained model.

## Trained Model
A t5-small has been trained (finetuned) on the dataset for 25 epochs. The trained model weights can be downloded from here : https://drive.google.com/drive/folders/1ihN6l--ssXrgF_nRLTyhyJI40tR30n7w?usp=sharing

## Docker Setup Instructions

1. **Run the following lines of code to set up the docker image and build a container**
   ```bash
   docker pull suhasnagaraj1999/umd:nlp 
   ```
   ```bash
   docker run -it --gpus all --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix --name nlp_container suhasnagaraj1999/umd:nlp
   ```
2. **To open multiple shells**:
   ```bash
   docker exec -it --env="DISPLAY=$DISPLAY"  nlp_container /bin/bash
   ```
4. **Source the ros workspace in all shells/terminals**:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ```
   
## Running the Simulation

1. **Launch Turtlebot3 Waffle in Gazebo (Custom World) and on RVIZ (with map)**:
   ```bash
   ros2 launch final_project final_project.launch.py use_sim_time:=True
   ```
   
2. **Set the initial pose of the robot by clicking on `2D Pose Estimate` on RVIZ**:

![alt text](https://github.com/suhasnagaraj99/Autonomous-Mobile-Robot-Navigation/blob/main/initial_pose.png?raw=false)
   
3. **Launch the file `tbot_nodes.launch.py` to start the battery broadcasting nodes**:
   ```bash
   ros2 launch group5_final tbot_nodes.launch.py use_sim_time:=True
   ```

4. **Run the `list_pub` node to launch the publisher**:
   ```bash
   ros2 run llm_package list_pub 
   ```
   
5. **Give the text input in the terminal and wait for the node to get the battery order**:
   
6. **Launch the file `tbot_get_goals` (node) to get the waypoints/goals**:
   ```bash
   ros2 run py_nlp get_goals
   ```

7. **Run the node `tbot_through_poses` to call action `NavigateThroughPoses` for navigation**
   ```bash
   ros2 run py_nlp waypoints
   ``` 
