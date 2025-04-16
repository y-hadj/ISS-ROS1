# Input Sampling Scheme for a Differential Drive Robot

This repository presents a package containing an Input Sampling Scheme Algorithm applied to a Two-Wheeled Differential Drive Robot in a Static Environment.
The robot's vision is derived from a Lidar's data, and the static environment is represented by a hospital floor.

The implemented algorithm detects the obstacles in the robot's Field of View (Fov) then generates trajectories that are safe in terms of the robot's geometry. After that, a trajectory is choosen and the node keeps on iterating this process until no safe trajectory can be generated.

Please note the robot is navigating through this static environment using dry starting and dry braking, so it's maximum velocity is set with the robot's dynamic constraints taken into consideration.


## Project Files
The project includes the following components:

- **The Package :** One (1) Package containing the different files to run the code smoothly. You can acess it [here](./ISS_hospital)

- **Simulation Video :** A video demonstrating the application of the ISS approach on the robot in the hospital floor. The generated safe trajectories are visualised before choosing one and following it. You can acess it [here](./demo_video.mp4)


## Additional Resources
Feel free to explore the package. If you have any technical questions or feedback, please don't hesitate to reach out :
- [Linkedin Profile](https://www.linkedin.com/in/yhadj/)
- [Email](mailto:yasser.hadj@g.enp.edu.dz)
