# Team Hunt - PSR 2021/2022

## The game
This project consists of two basic modes: <br>
- Manual mode
- Automatic mode (Team Hunt)

Manual mode allows you to drive the robot using your own keyboard (just don't expect it to be like Gran Turismo). This also includes a visualization of what the robot is detecting (camera, laser), using RVIZ.<br>

Automatic mode or Team Hunt is a game where 1 team out of 3 chases the others and tries to touch them. The game works by score losing points or winning points accordingly with the team you represent and if you catch or get caught. <br>

## Constitution of the Repository
This repository has three packages: <br>
- p_g11_bringup - This is responsible for launching the other files, because it has all the launch files necessary to run this program;
- p_g11_description - Has the information about the building of the robots and the worlds;
- p_g11_core - In this package we can find the code for the functioning of the robots. This is what make them move.

## Functionalities

## Initializing the Program
To start the program first we need to launch the gazebo world where the game (and other functionalities) will take place. This can be done by typing in the terminal the following command: <br>
<code>roslaunch p_g11_bringup gazebo.launch</code>

## Spawning the Robots
To spawn just one robot, for example to the manual driving, it is necessary to type in the terminal: <br>
<code>roslaunch p_g11_bringup bringup.launch</code><br><br>
It's possible to vizualize the robot in rviz with this code:<br>
<code>roslaunch p_g11_bringup bringup.launch visualize:=true</code><br><br>
If you want to run the team hunt game, you can use the following code:<br>
<code>roslaunch p_g11_bringup game_bringup.launch</code><br><br>
This will spawn in the gazebo world 3 robots for each team (red, green and blue).<br>
You can see one of these robots in rviz by typing the following code instead:<br>
<code>roslaunch p_g11_bringup game_bringup.launch visualize:=red1</code><br><br>

## Manual Driving
After spawning just one robot you it's possible to drive it manualy by running the following command:<br>
<code>roslaunch p_g11_bringup myteleop.launch</code><br><br>
After this you can drive the robot with the keys shown in this table:

| Key         | Action                          |
| ----------- | -----------                     |
| w           | Increase the speed              |
| x           | Decrease the speed              |
| a           | Turn Left                       |
| d           | Turn Right                      |
| s           | Stop Turning                    |
| space       | Stop moving the robot           |
| q           | Exit the program                |

## GMapping
GMapping allows for data collection from the LIDAR sensor of our robot and form a map out of that.<br>
If you want to see the results just use:<br>
<code>rosrun gmapping slam_gmapping scan:=p_g11/scan _base_frame:=/p_g11/base_link _map_frame:=/p_g11/map _odom_frame:=/p_g11/odom</code><br><br>

## Team Hunt Game

### Starting the game
Starting the game means you have to put the robots in automatic mode, for that just insert the code:<br>
<code>roslaunch p_g11_bringup auto_driver.launch</code><br><br>




## Referee
There couldn't be a game without a referee, so there is node provided by our teacher which evaluates multiple parameters of the game like for example, if a player gets caught, and gives/takes points accordingly.<br>
To run this program just run this command:<br>
<code>rosrun th_referee th_referee</code><br><br>


## Contributors
<ul>
<li>Miguel Da Costa Pereira</li>
<li>João Pedro Tira Picos Costa Nunes</li>
<li>João Diogo Alves Matias</li>
<li>Bartosz Bartosik</li>
</ul>