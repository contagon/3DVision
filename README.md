# 3DVision
A simple stereo vision application

# Idea	generation
The idea we chose to impliment was an extention of the problem introduced in HW4. Our goal was to track the trajectory of a ball traverling through the air and show the covariance of the calculated trajectory. To do this, we used two webcameras screwed to a rail of 80/20. We calibrated those cameras using the same method as used in HW4. Using the calibration parameters, we then found the 3D pose of the ball. We were then able to use a Kalman filter to estimate covariance of the trajectory of the ball. We then plotted the calculated position and a group of 5 circles, representing the covariance, onto one of the stereo images. 

# Challenges
Some of the challenges we faced with this project had to do with calibration and setup of the cameras. We found that putting the cameras close together (less than 3 inches apart) we were not able ot reliably calculate the z distance of the ball from the cameras. We also found that calibrating the cameras in a small room and then throwing the ball with in 5 feet of the camera made it difficult to accuratly follow the ball. And finally, we also had a hard time getting a useful T matrix from our stereo calibration function. Eventually we found that occocasionaly our code would reverse the order of the lists of corners used for calibration. 

# Solutions
To fix our calibration and setup issues we simpley made sure that the cameras we were using were about 4 inches apart and calibrated the cameras in a larger room. We also made sure to keep the ball at least 6 feet away from the cameras. This allowed us to consitantly track the ball and return more accuarate z distances. To solve the challenge we had with obtaining a T matrix from the stereo calibratoin function, we had to resolve the issue with our corner lists. Once that issues was ammended, we were able to calibrate and calcuate 3D pose smoothly. 
