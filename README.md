# Traffic Intersection Simulation
### FAU EGN4952C-001 Group 10 Project
##### Members: 
Thomas Cargill (tcargill2021@fau.edu)  
Jordan Sarig (jsarig2021@fau.edu)  
Cole Narbonne (cnarbonne2021@fau.edu)  
Zafar Bacchus (zbacchus2022@fau.edu)  

##### Summary/Motivation
This project aims to create a realistic simulation of the passage of autonomous vehicles through traffic intersections. With the rising prevalence of AI-driven vehicles, it has become increasingly important to develop and test algorithms that ensure their functionality and safety. Thus, Florida Atlantic University has requested a proposal from our group for a software-based simulation of autonomous vehicles driving through traffic intersections in the presence of other vehicles and pedestrians.

##### Methods
In order to complete the task specified above, our group has used the CARLA simulator. CARLA is an open source software designed specifically for the development and testing of algorithms for autonomous driving. In addition to CARLA, our group has chosen to use MATLAB's RoadRunner software to create custom 3D testing environments containing different types of intersections. Once these environments were imported into CARLA, we were able to create various Python scripts to alter traffic conditions and control the behavior of autonomous vehicles. We have also used YOLO12, a segmentation model used for the detection of traffic-related objects such as vehicles, pedestrians, and road signs.

##### Figures
The following figures display the correlations between the detection model's confidence rating and various factors in the simulation, including resolution, speed, and the class of object being detected.

![alt text](https://github.com/Cole0903/EMS-Traffic-Simulator/blob/main/Figures/Class-Specific%20Confidence%20Distributions.png "Class Specific Confidence Distributions")

![alt text](https://github.com/Cole0903/EMS-Traffic-Simulator/blob/main/Figures/Confidence%20Distribution%20Across%20Speed%20Ranges.png "Confidence Distribution Across Speed Ranges (km/h)")

![alt text](https://github.com/Cole0903/EMS-Traffic-Simulator/blob/main/Figures/Overall%20Confidence%20Distribution%20by%20Resolution.png "Overall Confidence Distribution by Resolution")
