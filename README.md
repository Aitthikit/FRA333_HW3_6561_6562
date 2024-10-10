
# HW3 Kinematic
จัดทำโดย
- อสมา_6561
- อิทธิกฤต_6562


## Please install following library before run my code

To install, run the following command

Install sympy
```bash
pip install sympy
```
Install roboticstoolbox for python
```bash
pip install roboticstoolbox-python
```
Install numpy version 1.24.4
```bash
pip install numpy==1.24.4
```
## Clone project

To Clone, run the following command

Clone to you directory
```bash
git clone https://github.com/Aitthikit/FRA333_HW3_6561_6562.git
```
You will see the file name 
- FRA333_HW3_6561_6562.py for Answer
- testScript.py for Test
-  HW3_utils.py for Forward Kinematic function
## Run Answers
```bash
python FRA333_HW3_6561_6562.py
```
you will see

![alt text](https://github.com/aitthikit/FRA333_HW3_6561_6562/blob/main/Picture/Answer.png?raw=true)
- Jacobian matrix reference to end-effector frame (6x3 Matrix)
- Is it near singularity (1)
- tau value [joint 1,joint 2,joint 3]
  
you can edit configulation space in this code at Input zone

![alt text](https://github.com/aitthikit/FRA333_HW3_6561_6562/blob/main/Picture/Input.png?raw=true)

This code is Calculation Jacobian Matrix from this Equation

![alt text](https://github.com/aitthikit/FRA333_HW3_6561_6562/blob/main/Picture/Jacobian.jpg?raw=true)
![alt text](https://github.com/aitthikit/FRA333_HW3_6561_6562/blob/main/Picture/JacobianCal.png?raw=true)
## Run Test Scripts
```bash
python testScript.py
```
you will see

![alt text](https://github.com/aitthikit/FRA333_HW3_6561_6562/blob/main/Picture/TestScriptsAns.png?raw=true)
- Focus at All True or False this testScript are check by use roboticstoolbox for python
  
This code is Calculation Jacobian Matrix from this DHparameter

![alt text](https://github.com/aitthikit/FRA333_HW3_6561_6562/blob/main/Picture/DHrobot.png?raw=true)

you can edit configulation space in this code at Input zone same line as Run Answers

![alt text](https://github.com/aitthikit/FRA333_HW3_6561_6562/blob/main/Picture/Input.png?raw=true)
