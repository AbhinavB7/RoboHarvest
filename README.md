# RoboHarvest

Modeling‬ ‭and‬ ‭simulation‬ ‭of‬ ‭an‬ ‭automated‬ ‭fruit‬ ‭picker‬ ‭robot‬ RoboHarvest‬ ‭a‬ ‭mobile‬ ‭serial‬ ‭manipulator‬‬ ‭designed‬ ‭to‬ ‭automate‬ ‭the‬ ‭task ‬‭of‬ plucking‬‭ fruits ‬‭from‬‭ trees‬‭ and‬‭ collecting‬‭ them‬ ‭in‬‭ the‬‭ bin‬‭ on‬‭ the‬ ‭robot using **INVERSE KINEMATICS**.

Using the RGBD camera to identify the distance to the fruits.
![Screenshot from 2024-01-30 02-41-09](https://github.com/AbhinavB7/RoboHarvest/assets/87815926/d2fc32ef-4e12-40f2-8340-7b9c533f31d8)


The robot can identify the fruits using the camera attached to the front.
![Screenshot from 2023-12-31 18-44-50](https://github.com/AbhinavB7/ROBOHARVEST-A-6DOF-Serial-Manipulator-Robot/assets/87815926/83b15485-64e2-4619-a9fc-e7f49475bd54)

The robot arm will then move towards the fruit to pick the fruit.
![Screenshot from 2023-12-31 19-03-30](https://github.com/AbhinavB7/ROBOHARVEST-A-6DOF-Serial-Manipulator-Robot/assets/87815926/1e6c4d74-a7b7-46ee-b44b-f1d72cab4d72)
![Screenshot from 2023-12-31 19-03-44](https://github.com/AbhinavB7/ROBOHARVEST-A-6DOF-Serial-Manipulator-Robot/assets/87815926/8641e910-85a1-47c5-9f93-152918d1c71c)

The robot will then place the fruit back into the basket. 
The movement of the ur10 arm is controlled using inverse kinematics. To overcome singularities, I calculated Psuedo Inverse with Damping factor instead of directly computing inverse of the Jacobian. To achieve faster motion I have used lamdify, which transforms SymPy expressions to lambda functions which can be used to calculate numerical values very fast. 

![Screenshot from 2023-12-31 17-50-02](https://github.com/AbhinavB7/ROBOHARVEST-A-6DOF-Serial-Manipulator-Robot/assets/87815926/3af0a78e-59e1-4c5e-b0a9-a9676e0f2d3f)
Robot in gazebo world.

The robot is designed and modeled on Solidworks.
![image](https://github.com/AbhinavB7/ROBOHARVEST-A-6DOF-Serial-Manipulator-Robot/assets/87815926/1ec27fa3-7122-4878-81ef-a50930190388)
