# Writeup Content
1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.
3. Implement all of the elements of the prediction step for the estimator.
4. Implement the magnetometer update.
5. Implement the GPS update.
6. Flight Evaluation

### Implement Estimator

#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.

I calculated the standard deviation with calc_sd.py python script using numpy.std() function to get MeasuredStdDev_GPSPosXY = 0.69497378191, MeasuredStdDev_AccelXY = 0.492090871504. The measured GPS X data and Accelorometer X data from the text files config/log/Graph1.txt and config/log/Graph2.txt were used.

#### 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.

I implemented a Nonlinear Complementary Filter using the Quaternion class initialized with current attitude estimate (rollEst, pitchEst and ekfState(6)). Next I Integrated the quaternion by rotation rates from gyro, converted back from quaternion to Euler angles. Finally, the Gyro and Accelerometer was fused.

```
Quaternion<float> qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
    
  qt.IntegrateBodyRate(gyro,dtIMU);
    
  V3D eulerRPY = qt.ToEulerRPY();

  float predictedPitch = eulerRPY.y;
  float predictedRoll  = eulerRPY.x;
  ekfState(6) = eulerRPY.z;

  // normalize yaw to -pi .. pi
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
```

#### 3. Implement all of the elements of the prediction step for the estimator.The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.

I implemented the seven state (x,y,z,dx,dy,dz) prediction. I predicted x, y, z states as below:
```
  predictedState(0) = curState(0) + curState(3) * dt; 
  predictedState(1) = curState(1) + curState(4) * dt; 
  predictedState(2) = curState(2) + curState(5) * dt; 
```
And computed the dx,dy,dz states as below:
```  
  V3F accel_I = attitude.Rotate_BtoI(accel);
  
  predictedState(3) = curState(3) + accel_I.x * dt; 
  predictedState(4) = curState(4) + accel_I.y * dt; 
  predictedState(5) = curState(5) + accel_I.z * dt; 
```
Computed the Rbg_prime as the partial derivative of the Rbg rotation matrix.
```
float cPhi = cosf(roll);
    float sPhi = sinf(roll);   
    float cThe = cosf(pitch);
    float sThe = sinf(pitch);
    float cPsi = cosf(yaw);
    float sPsi = sinf(yaw);
    
    RbgPrime(0,0) = -cThe * sPsi; 
    RbgPrime(0,1) = -sPhi * sThe * sPsi - cPhi * cPsi; 
    RbgPrime(0,2) = -cPhi * sThe * sPsi + sPhi * cPsi;  
    RbgPrime(1,0) = cThe * cPsi; 
    RbgPrime(1,1) = sPhi * sThe * cPsi - cPhi * sPsi; 
    RbgPrime(1,2) = cPhi * sThe * cPsi + sPhi * sPsi; 
```
Predicted the current covariance forward
```
gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;
  gPrime(3, 6) = (RbgPrime(0) * accel).sum()*dt;
  gPrime(4, 6) = (RbgPrime(0) * accel).sum()*dt;
  gPrime(5, 6) = (RbgPrime(0) * accel).sum()*dt;
  ekfCov = gPrime *ekfCov*gPrime.transpose() + Q;
```


#### 4. Implement the magnetometer update.The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).

I implemented the hPrime and zt of the magnetometer. Used the difference between the current state and the magnetometer value to determine when it is out the range, then change the zt:
```
  hPrime(6) = 1;
  zFromX(0) = ekfState(6);
  float delta = magYaw - ekfState(6);
  if (delta > F_PI) 
  {
	  zFromX(0) += 2.f*F_PI;
  }
  else if (delta < -F_PI) 
  {
	  zFromX(0) -= 2.f*F_PI;
  }
```

#### 5. Implement the GPS update.The estimator should correctly incorporate the GPS information to update the current state estimate.

I implemented the hPrime and zt as below:
```
 hPrime(0, 0) = 1;
  hPrime(1, 1) = 1;
  hPrime(2, 2) = 1;
  hPrime(3, 3) = 1;
  hPrime(4, 4) = 1;
  hPrime(5, 5) = 1;
  
  zFromX(0) = ekfState(0);
  zFromX(1) = ekfState(1);
  zFromX(2) = ekfState(2);
  zFromX(3) = ekfState(3);
  zFromX(4) = ekfState(4);
  zFromX(5) = ekfState(5);
```


### Flight Evaluation
#### 1 Meet the performance criteria of each step.For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.
Yes. the estimator met the performance criteria with the controller.


#### 2 De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).

Yes, the controller successfully meet the performance criteria of the final scenario (<1m error for entire box flight).
