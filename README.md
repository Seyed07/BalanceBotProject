# Two-Wheeled Balancing Robot

This project is about building a **two-wheeled balancing robot** using **Arduino UNO**, **L298N motor driver**, and **25GA 33RPM DC motors**. The robot is designed to maintain balance and navigate through its environment using a modular structure and control algorithms.
![Robot Image](images/robot.jpg)
## Structure of the Robot

The robot's frame is built using three lightweight plates, which house the core components:

###  **Three Plates**
- **Top Plate**: Houses the **Arduino UNO**, **L298N motor driver**, and other electronics.
- **Middle Plate**: Supports the power supply and helps distribute weight evenly.
- **Bottom Plate**: Mounts the **DC motors** and **wheels**, ensuring stability for movement.
![Robot Image](images/robot.jpg)

##  **Metal/Plastic Rods**
- **Four rods** connect the plates and provide structural support.
- They help maintain the height and balance of the robot, allowing easy adjustments to the structure.
![Robot Image](images/robot.jpg)

##  **Wheels and Motors**
- **2x 25GA 33RPM DC Motors** drive the wheels and provide movement.
- The **L298N motor driver** controls their direction and speed, ensuring smooth operation.

![Robot Image](images/robot.jpg)

## **Arduino UNO**
- The **Arduino UNO** is the microcontroller that executes the robot's code. It processes sensor data, controls the motors, and ensures the robot maintains balance.

![Arduino UNO](images/Arduino_UNO.png)

## **L298N Motor Driver**
- The **L298N** motor driver controls the direction and speed of the **DC motors**, translating the signals from the **Arduino UNO** into movement.
![Motor Driver](images/L298N.png)

## **2x 25GA 330RPM DC Motors**
- The **DC motors** drive the wheels of the robot, allowing it to move, balance, and adjust its position as needed.
<img src="images/photo_۲۰۲۴-۱۲-۲۳_۱۵-۳۸-۲۹.jpg" alt="DC Motors" width="300" height="300">

## MPU6050 Sensor
- The MPU6050 is a 6-axis motion tracking sensor that combines a 3-axis accelerometer and a 3-axis gyroscope. It measures both acceleration and angular velocity in three dimensions, providing critical data for balancing the robot. By continuously monitoring the robot's tilt and rotational speed, it helps maintain balance and stability during movement.

## **Power Supply (Adapter)**
- The robot is powered using an **adapter** instead of batteries, ensuring stable power for both the electronics and motors.
![Robot Image](images/robot.jpg)

