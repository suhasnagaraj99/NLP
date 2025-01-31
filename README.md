# **Adaptive Text-to-Command Translation for Robot Navigation**

## **Overview**
This project extends [Autonomous Mobile Robot Navigation](https://github.com/suhasnagaraj99/Autonomous-Mobile-Robot-Navigation) by integrating **Natural Language Processing (NLP)** for command-based navigation in a simulated environment. The system fine-tunes a **T5-Small model** to convert natural language instructions into structured navigation commands, enabling a **TurtleBot3** robot to execute multi-step navigation tasks.

![alt text](https://github.com/suhasnagaraj99/NLP/blob/main/images/poster.png?raw=false)

### **Key Features**
- **Fine-tuned T5-Small model** for converting text commands into waypoints.
- **LoRA-based adaptation** for efficient fine-tuning with minimal computational cost.
- **ROS2-based simulation** using Gazebo and RViz for real-world validation.
- **Task-specific synthetic dataset** with over 24,581 navigation instructions.

---

## **Training and Evaluation**

### **1. Finetune the NLP Model**

- **Fine-Tuning the Entire T5 Model:**  
  [Download the Notebook](https://github.com/suhasnagaraj99/NLP/blob/main/Scripts/Traning%20%26%20Validation/Fine_Tuning_Entire_T5.ipynb)

- **LoRA-Based Fine-Tuning:**  
  [Download the Notebook](https://github.com/suhasnagaraj99/NLP/blob/main/Scripts/Traning%20%26%20Validation/LORA_fine_tuning_T5.ipynb)

### **2. Evaluate the Finetuned Model**

- **Evaluation for Fine-Tuned T5:**  
  [Download the Notebook](https://github.com/suhasnagaraj99/NLP/blob/main/Scripts/Testing/Fine_Tuning_Entire_T5_Test.ipynb)

- **Evaluation for LoRA-Tuned T5:**  
  [Download the Notebook](https://github.com/suhasnagaraj99/NLP/blob/main/Scripts/Testing/LORA_fine_tuning_T5_Test.ipynb)

### **Note:**
- The dataset can be downloaded from the current repository: [Dataset](https://github.com/suhasnagaraj99/NLP/tree/main/Dataset)
- Ensure that the dataset and pre-trained models are correctly referenced before running these notebooks.
- Ensure required libraries and dependencies are installed before running these files on local system.
  
---

## **Trained Model**

Download the the trained T5 Models from the following links:

- [T5 Model](https://drive.google.com/drive/folders/1_g3VE7xYBWhKx2ryHSWstqdw_KEgPix1?usp=sharing)
- [LoRA based T5 Model](https://drive.google.com/drive/folders/14s9XCch4XDVkprtSODFsP0ClbifRkEkR?usp=sharing)

## **ROS2-Based Simulation**

### **1. Setup Docker Environment**
Pull and run the Docker image:
```bash
docker pull suhasnagaraj1999/umd:nlp 
docker run -it --gpus all --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix --name nlp_container suhasnagaraj1999/umd:nlp
```

To open multiple shells inside the container:
```bash
docker exec -it --env="DISPLAY=$DISPLAY" nlp_container /bin/bash
```

### **2. Source the ROS2 Workspace**
```bash
cd ~/ros2_ws
source install/setup.bash
```

### **3. Launch the Simulation**
Start TurtleBot3 Waffle in Gazebo and RViz:
```bash
ros2 launch final_project final_project.launch.py use_sim_time:=True
```
- Set the initial pose using **`2D Pose Estimate`** in RViz:

![alt text](https://github.com/suhasnagaraj99/Autonomous-Mobile-Robot-Navigation/blob/main/initial_pose.png?raw=false)

### **4. Run NLP-Based Navigation**

- Start battery broadcasting nodes:
  ```bash
  ros2 launch group5_final tbot_nodes.launch.py use_sim_time:=True
  ```
- Start the publisher node:
  ```bash
  ros2 run llm_package list_pub
  ```
- Enter text commands in the terminal and wait for processing:
![alt text](https://github.com/suhasnagaraj99/NLP/blob/main/images/text_input.png?raw=false)
- Run the goal extraction node:
  ```bash
  ros2 run py_nlp get_goals
  ```
- Execute waypoint-based navigation:
  ```bash
  ros2 run py_nlp waypoints
  ```
![alt text](https://github.com/suhasnagaraj99/NLP/blob/main/images/moving_robot.png?raw=false)

### **Note:**
- Alternatively you can download the ROS2 packages from the following link: [Packages](https://drive.google.com/drive/folders/1U7xWgvSILd7OoGJMpwQPp-CUejlRGhaR?usp=sharing)
- Please ensure that the trained models are downloaded and correctly referenced before running the packages outside docker

---

## **Results & Performance**

The **T5-Small model** was fine-tuned for **25 epochs** on the synthetic dataset. Performance comparison:

| Model                  | Sequence Accuracy | Position Accuracy | Training Time |
|------------------------|------------------|------------------|---------------|
| Baseline T5-Small     | 0%                | 0%               | N/A           |
| Few-Shot Learning     | ~0%               | ~0%              | 336s          |
| Full Fine-Tuning      | **100%**          | **100%**         | 4577s         |
| LoRA Fine-Tuning      | **98.5%**         | **98.8%**        | 3415s         |

> *LoRA-based fine-tuning achieved near-perfect results while reducing training time by 25%.*

For more details, refer to our full **[Project Report](https://github.com/suhasnagaraj99/NLP/blob/main/NLP_Final_Project_Report.pdf).**

---

## **Acknowledgments**
This project was completed as part of the **final project for CMSC723** under the guidance of **Professor Dr. Jordan Boyd-Graber** and **Dr. Naomi Feldman**. We sincerely thank them for their insights and support throughout the course.

We also extend our gratitude to **Dr. Zied Kootbally** for allowing us to reuse some of his **ROS2 packages** for this project. Additionally, we appreciate the **University of Maryland (UMD)** for providing essential resources and support that enabled the successful completion of this work.

---



