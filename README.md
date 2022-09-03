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

6. **Extended Kalman filter based localization** Localization is the task of estimating the robots pose given the observations and Map information. Since most real world robot models use non linear models for motions, the common Kalman filter cannot be used. Instead we use the Extended Kalman Filter. In EKF we use first order taylor expansion to linearise the function (local linearization) and then perform the prediction and update steps similar to the Kalman Filter.  
   In the given excercise the odometry and range measurements from a differential drive robot was given and the task was to localize the robot in a feature based map with known correspondence.    
   ![Final Trajectory after EKF Localization](img/ekf.png?raw=true "Final Trajectory")   

7. **Path planning** The path planning problem basically estimates a path from a given node to a goal node in a given mapped terrain. The path planning algorithms are of two types:  
      1. Informed :  
   In informed search the algorithm has some information about the cost to the goal node, usually in the form of heruristics.  
   eg. A*, Greedy search etc.
      2. Uninformed :  
   In uninformed the algorithm doesnt have a heuristic usually estimates costs from origin node to current node.  
   eg. Djikstra, BFS,DFS etc.  
   
   In the following assignment path planning was performed on an occupancy grid map using greedy search and A*. It was found that the computational speed can be significantly improved by applying a gaussian blur over the map provided.  
   ![Greedy path planning](img/greedy.png?raw=true "greedy planner")

   ```
   A star algorithm

   open = []
   closed =[]
   neighbors = []

   while open:
      open.sort()
      current = open[0]
      if current == goal:
         return path
         break
      else:
         closed.append(current)
         for all valid neighbors of current:
            if neighbor not in closed list:
               new cost = cost to node + cost from node to goal
               if neighbor in open:
                  if cost > new cost:
                     cost = new cost
                     parent = current
               else:
                  open.append(neighbor)
                  cost = new cost
                  parent = current

   ```
   ![A* path planner](img/astar.png?raw=true "A* planner")  

8. **Iterative Closest Point** : The point cloud data from Lidar or depth scans is not aligned. So for SLAM and other applications we need to match the cordinate frames of corresponding clouds. This can be done for both cases, known correspondeces and unknown correspondence between points.  
   The algorithm for ICP is fairly straight forward.
   ```
   1. Define an initial guess
   2. Initialize initial error to infinity
   3. While Error is decreasing or Max iterations not reached 
      3.1  Calcualte correspondences
      3.2  Compute the Rotation and Translation vectors.
      3.3 Calculate Error and New guess
   ```

   In the given assignment, KDtree algorithm was used to find correspondence between points.
   ![Input](img/icp_input.png?raw=true "Input")   
   ![Output](img/icp_output.png?raw=true "Output") 
