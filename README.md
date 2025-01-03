# Robot_Navigation_Algorithm

## Path Tracking

### 1. Pure Pursuit Algorithm

The Pure-pursuit algorithm is a simple and efficient path-tracking algorithm used to determine the steering angle of a vehicle as it follows a specific path.  
This is based on the vehicle's current position and a target point on the path.  
In this algorithm, the distance to the target point that the vehicle aims to track is defined by a parameter called the Look-ahead distance,  
represented as the variable Lfc in the code. Below are the observed results when varying the Lfc parameter.  

<img src="https://github.com/user-attachments/assets/41af6fcc-2699-41bb-b90a-7da55bfacf08" alt="Description2" style="width: 60%; height: 600px;">  

#### Discussion

In the results above, it was observed that setting the parameter value to 2.0 produced the smoothest path-following behavior.  
As the value was gradually increased beyond 2.0, the tracking trajectory was examined further,  
revealing that larger values caused the vehicle to exhibit increasingly unnecessary wide turns at corners while following the path.
<hr style="border-top: 3px solid #bbb;">

### 2. Stanley Controller Algorithm

The Stanley Controller algorithm adjusts the vehicle's position and errors based on its front axle as the reference point to follow a given path.  
The Stanley Controller corrects two primary types of errors to track the path: Crosstrack error and Heading error.  
Crosstrack error refers to the lateral error between the vehicle's current position and the path,  
while Heading error is the angular difference between the vehicle's heading and the direction of the path.
  
Upon examining the code, it was found that the adjustable parameter, control gain, is defined as the variable k,  
and it is associated with the crosstrack error. The results of varying the k parameter were observed as described below.

<img src="https://github.com/user-attachments/assets/d8fe5f07-34db-41a6-bc4a-9cf231093842" alt="Description2" style="width: 60%; height: 400px;">  
<img src="https://github.com/user-attachments/assets/50129580-bdf7-460d-ac79-79cb9b49234f" alt="Description2" style="width: 60%; height: 200px;">  

#### Discussion

The results were analyzed by gradually increasing the control gain parameter starting from a small value of 0.01.  
At 0.01, the tracking path showed a similar direction and shape to the original path, but the distance error remained unchanged.  
As the control gain value was increased, the tracking path progressively aligned more closely with the course.  
When the control gain k was set to 1.0, the course and the tracking path overlapped perfectly. 
Further increasing the value revealed that the vehicle approached the course at increasingly steeper angles during the initial correction phase.  
<hr style="border-top: 3px solid #bbb;">

### 3. LQR_Steer_Control Algorithm

LQR steering control is a control method designed to optimize the behavior of linear systems,  
addressing the limitations of PID control.  
It employs an algorithm that predicts the next state based on the current state and control input.  
The maximum steering angle parameter is defined in the code as the variable max_steer.  
By adjusting the value of max_steer, the upper limit of the steering angle was modified to analyze its impact on the algorithm.  

<img src="https://github.com/user-attachments/assets/a0362ca3-3baa-492e-a2d4-81b7df9cc7d4" alt="Description2" style="width: 60%; height: 600px;">  
<img src="https://github.com/user-attachments/assets/3d6265ff-964d-472a-8a34-be79b074fef5" alt="Description2" style="width: 60%; height: 200px;">  

#### Discussion

Initially, the code was executed with the default setting of max_steer = 45°, and the vehicle closely followed the given path with high accuracy.  
When the value was set to 5° and the code was executed, the limited maximum steering angle of 5° resulted in insufficient turning,  
causing the vehicle to deviate significantly from the path and trace a large elliptical trajectory before reaching the destination.  
When the value was set to 25° or higher, the vehicle effectively followed the given path, except for slight deviations at corners.
<hr style="border-top: 3px solid #bbb;">

### Optional

Comparison of Pure Pursuit Algorithm and Stanley Controller Algorithm on a Single Screen  

<img src="https://github.com/user-attachments/assets/7dbc1a70-9c63-4767-b16b-e627fe80b5c0" alt="Description2" style="width: 60%; height: 400px;">  
<hr style="border-top: 3px solid #bbb;">






### System Architecture
<img src="https://github.com/user-attachments/assets/f71ccd54-e052-4c49-92e8-f7874f00a6e1" alt="Description2" style="width: 50%; height: 200px;">  

### Hardware Images
<div style="display: flex; justify-content: space-around;">
  <img src="https://github.com/user-attachments/assets/9d6a52ba-993f-44bc-ac42-0f91d5d575c0" alt="Description1" style="width: 23%; height: 300px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/040bea4c-3475-4842-9532-60c6ee922d72" alt="Description2" style="width: 23%; height: 300px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/8365030e-4028-4324-a1d5-a86413f4f0c2" alt="Description2" style="width: 40%; height: 300px;">
</div>
<hr style="border-top: 3px solid #bbb;">

## Load inertia
<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="https://github.com/user-attachments/assets/edf8146a-45c5-4cba-8799-0b88b8bb135a" alt="Description1" style="width: 40%; height: 250px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/e4f0d2ac-1bcc-4d2f-aac7-40b35eef1a09" alt="Description" alt="Description2" style="width: 20%; height: 100px;">
</div>
<hr style="border-top: 3px solid #bbb;">

## Cascade Controller
A current controller was designed, and based on its frequency response, a speed controller was subsequently designed.  
Similarly, considering the frequency response of the speed controller, a position controller was designed.  
  
For the position controller, PD control was employed to ensure precision and stability.  
Both the speed and current controllers utilized PI control, and to prevent error accumulation, values were compensated using anti-windup gain.  
  
Saturation limits were set for each controller to ensure the outputs did not exceed predefined thresholds.  
  

- (position-velocity-Torque controller)    
<img src="https://github.com/user-attachments/assets/b905271a-ae75-46ed-a70a-7e286d39e1bf" alt="Description" alt="Description2" style="width: 50%; height: 250px;">

- current controller (PI Control)  
<img src="https://github.com/user-attachments/assets/a2ddba02-634a-4b72-bdf9-844ebdfdbfd0" alt="Description" alt="Description2" style="width: 50%; height: 340px;">

- velocity controller (PI Control)  
<img src="https://github.com/user-attachments/assets/74367bee-66de-4c85-b4d7-b36f4a8a59a2" alt="Description" alt="Description2" style="width: 50%; height: 340px;">

- position controller (PD Control)  
<img src="https://github.com/user-attachments/assets/28e2d020-805c-4d31-87fc-ce5408452f86" alt="Description" alt="Description2" style="width: 50%; height: 250px;">  
<hr style="border-top: 3px solid #bbb;">

## Simulation Results

### UI Interface
<img src="https://github.com/user-attachments/assets/d8d3bbbf-5393-422d-9adc-e709926b01cc" alt="Description" alt="Description2" style="width: 60%; height: 400px;">

### Forward Kinematics
- join1=90, joint2=90 (Forward) & join1=-150, joint2=90 (Forward)

<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="https://github.com/user-attachments/assets/7f04d3ae-e375-4fcc-8658-3fc1ebefe3f1" alt="Description1" style="width: 25%; height: 300px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/40308629-b068-4745-b54c-153a5258b18c" alt="Description" alt="Description2" style="width: 25%; height: 300px;">
</div>

### Inverse Kinematics
- x=1.0, y=-1.0 (Inverse)        & x=-1.0, y=0.8 (Inverse)

<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="https://github.com/user-attachments/assets/e3b4127f-e730-46c0-bbaa-aa522fa5b958" alt="Description1" style="width: 25%; height: 300px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/d2f1470f-51b6-4072-8c95-e3da94707ef0" alt="Description" alt="Description2" style="width: 25%; height: 300px;">
</div>
<hr style="border-top: 3px solid #bbb;">

## Results

- Infinite Mode, +280 degree
<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="https://github.com/user-attachments/assets/cff134b8-51b5-427d-b83e-6b3553db7fe5" alt="Description1" style="width: 45%; height: 250px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/ccce71b2-3e86-4d05-8218-e67d206c3d54" alt="Description2" style="width: 25%; height: 250px;">
</div>  
<br>

- Infinite Mode, -90 degree  
Due to hardware discrepancies, the system cannot precisely point to -90 degrees.  
<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="https://github.com/user-attachments/assets/ac66b008-a3bf-4bbb-9ab1-02916b19f947" alt="Description1" style="width: 45%; height: 250px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/63c5d350-3e79-4e40-a964-076f0a8fc470" alt="Description2" style="width: 25%; height: 250px;">
</div>  
<br>

- Joint Mode, +190 degree (it moves -170 degree)
<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="https://github.com/user-attachments/assets/3a3262ef-8d10-4e57-bcce-2e9fe36ef741" alt="Description1" style="width: 30%; height: 250px; margin-right: 2%;">
  <img src="https://github.com/user-attachments/assets/a7c600c5-4caa-47a2-a05b-8702a5260adc" alt="Description2" style="width: 20%; height: 250px;">
</div>  
<hr style="border-top: 3px solid #bbb;">



## Gain Tuning

During the project, individual tuning was performed using MATLAB simulation to achieve effective control by considering the physical characteristics of the motor and the properties of the control system.  
The values obtained from MATLAB were applied to the actual DC motor for control, and the control was successfully achieved.  
Additionally, the PID gains can be further fine-tuned based on the desired characteristics.




