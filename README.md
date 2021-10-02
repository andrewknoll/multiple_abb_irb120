## Deformable object manipulation in multi-robot environments
### T.F.G. Andrés Otero García
Read the Spanish memory of this undergraduate thesis following this link: \
https://deposita.unizar.es/record/64356?ln=es

#### Build
~~~
catkin_make
source devel/setup.bash
~~~

#### Launch the environment
Launch file `multiple_spawner_gazebo_script.bash` with the number of desired robots (recommended >2)
~~~
./multiple_spawner_gazebo_script.bash 2
~~~

#### Launch the MoveIt controller
Launch file `moveit_planning_execution_gazebo.launch` with command `roslaunch` and change parameter `robot_name` to the name of the robot to be controlled.
~~~
roslaunch multiple_abb_irb120 moveit_planning_execution_gazebo.launch robot_name:=robot1
~~~

#### Launch a demo
You can launch any of the following demos:
- `robots_moving_demo` : Asynchronous independent movement for 2 robots.
- `robots_waving_demo` : Synchronous movement in a sine wave for a parameter defined number of robots. **REQUIRES AN EXTRA PARAMETER**
- `grid_demo` : Manipulation with 2 robots of a grid which is close to the robots but **not on the floor**.
- `grid_wave_demo` : Manipulation with 2 robots of a grid which is **on the floor** next to the robots.
- `small_grid_manipulation` : Manipulation of a small grid with 2 robots. **REQUIRES SMALL GRID VERSION**

To launch these, you must use the following command:
~~~
rosrun multiple_abb_irb120 <name of the demo> <parameters>
~~~
You have to replace `<name of the demo>` with the name of any of the previous demos, and you can leave the `<parameters>` field empty **except for `robots_waving_demo`**, for which you'll need to specify the number of robots you wish to use.

#### Other demos
`multiple_spawner_gazebo_script.bash` has three possible additional parameters:
- `-t` : Adds a table to the simulation
- `-s` : Uses the **small version** of the grid.
- `-d` : Uses no robots at all and spawns a bigger version of the grid, with its corners fixed to their initial positions.
