# GPS_Tracking_System

The current repository represents my Capstone project developed to university in order to accomplish a Automotive Tracking System for a car fleet used by the local police in Espírito Santo, Brazil. 

![alt text](https://github.com/gabriellopes007/GPS_Tracking_System/blob/main/gps_tracking_system_arquitetura.png)

The explanation on the files are as follows:
- GPSKalmanFilter: This is the developed library for this project. It can be embedded in many devices used in Arduino enviroment. Later it is interesting to adapt it for several diferent MCU architecture. The link to the repository itself is: https://github.com/gabrielbklopes/GPSKalmanFilter
- HMC5883L is the library to access the magnetometer. Some changes were applied.
- Haversine is the library that calculates the distance between two points in the globe and the bearing angle.
- MPU6050 access the sensor that incorporates a gyroscope and a accelerometer. Several changes had to be applied in order to math the current project.
- arduino-lmic-master is responsible for the LoRa communication. Be aware this is a highly complicated library and can be trick to understand and work with. Also it was not exactly made for ESP32 MCU. The library was not developed by me, the credit for the work belongs to the original developer. But I had to execute some chances inside the code. 
- GPSFilter.ino is the program itself. It covers the LoRa communication, the gps aquisition, the sensors first calibration and applies the Madgwick fuction to calculate the system attitude. In the future this function can migrate to a library or even to the Kalman Filter library.
- KFSimulationGPS.ipynb is a Google Colab notebook that covers the Kalman Filter applied in datasets obtained from the embedded system data.
- PathSimulationGPS.ipynb simulates the car path using trigonometric functions for the position, velocity and acceleration. This is a good kickoff point to start understanding the functioning of the system itself.
- calibration_mag.ino is used to calibrate the magnetometer against Hard and Soft Iron interference.

The image bellow shows the results for the filter considering the mathematical calculation of the bearing angle. 
![gps_bearing_all](https://user-images.githubusercontent.com/30734909/168810791-bddce8c2-160b-4c14-9955-28e75f3ca5e3.png)

Using the bearing angle provided by the magnetometer an error occours due to the electromagnetic interference from the car.
![gps_final_nolost_meters](https://user-images.githubusercontent.com/30734909/168812057-8ed65f9a-d6e1-419c-aa1a-ae3e2fa25be8.png)

![gps_final_nolost_errN](https://user-images.githubusercontent.com/30734909/168812476-e1e4e3a4-e888-4da8-a20f-d027db7fc16d.png)
![gps_final_nolost_errE](https://user-images.githubusercontent.com/30734909/168812514-8af78b88-07ac-40d7-954f-8beb5bd59475.png)


Gabriel Lopes!
