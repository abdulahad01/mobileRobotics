# mobileRobotics
Assignments and related course material completed by me as part of the 'Online Training: Mobile Robotics' from Dr. Cyril Stachniss

1. Implemented a **Recursive Bayesian filter** on a non-cyclic world. The probability histogram is attached below.  
   ![Bayes Filter](img/bayesian.png?raw=true "Output of Bayes filter for localization")
2. **Occupancy grid mapping** algorithm was explored. A data file consisting of raw ranges and robot pose values was used. The raw values were converted into cell values of the grid cell by using bresenhams line algorithm and the map resolution was set as 0.25  
   ![Occupancy grid](img/occupancy.png?raw=true "Output occupany grid map")  
3. **Motion model** The motion model of a robot gives the probability of the state given the previous state and control inputs. The given assignemnt utilises odometry motion model which summarises the motion as a combination of a rotation, translation and rotation vector. Since the odometry model uses a lot of computation we performed sampling based odometry to estimate the motion of a robot (output attached below)  
   ![The motion model of a robot executing a given motion](img/motion.png?raw=true "Motion model")  
4. **Observation model** gives the porbability of the observationn given the robot state and the map (or position of landmark). In this assignment an observation modelof a range finder sensor was developed. The model is composed of a gaussian noise distribution, a random noise, a max range noise and a dynamic obstacle noise.  
   ![Measurement probability over a 20x20 cell for given sensor locations and range measurements](img/observation.png?raw=true "Measurement probability over a 20x20 cell for given sensor locations and range measurements")  

5. **Kalman filter** is an estimation algorithm used for linear models and assumes gaussian noise. It has a prediction and a correction (or update) step which finally outputs the estimate of the state by calculating the Kalman gain. In the given assignment the KF was used to estimate the height of a falling object with given observations. The algorithm was tested on a perfect sensor and a very noisy sensor and observations are attached.  
   ![Normal estimate](img/kf1.png?raw=true "Normal estimate")  
   ![Noisy sensor estimate](img/kf2.png?raw=true "Noisy sensor estimate")  
   ![Perfect sensor estimate](img/kf3.png?raw=true "Perfect sensor estimate")  


