# Team Hunt - PSR 2021/2022

This repository was developed for the course of PSR. The goal was to simulate a team hunt game with robots and also be able to drive a robot manually.

## Constitution of the Repository
This repository has three packages: <br>
- p_g11_bringup - This is responsible for launching the other files, because it has all the launch files necessary to run this program;
- p_g11_description - Has the information about the building of the robots and the worlds;
- p_g11_core - In this package we can find the code for the functioning of the robots. This is what make them move.

## Functionalities
<ol>
<li> Manual Driving </li><br>
<li> Gmapping </li><br>
<li> Team Hunt Game </li> 
</ol>

## Initializing the Program
To start the program first we need to launch the gazebo world where the game (and other functionalities) will take place. This can be done by typing in the terminal the following command: <br>
<code>roslaunch p_g11_bringup gazebo.launch</code><br><br>
You can change the world by adding <code>world:=world_name</code> to the previous command.
You can choose these three worlds:
- fnr
- th_arena_1
- th_arena_2
- th_arena_3

Example:<br>
<code>roslaunch p_g11_bringup gazebo.launch world:=fnr</code><br><br>

## Spawning the Robots
### One Robot
To spawn just one robot, for example to the manual driving, it is necessary to type in the terminal: <br>
<code>roslaunch p_g11_bringup bringup.launch</code><br><br>
#### Visaulize it with RVIZ
It's possible to vizualize the robot in rviz with this code:<br>
<code>roslaunch p_g11_bringup bringup.launch visualize:=true</code><br><br>

### 9 Robots, 3 for each team
If you want to run the team hunt game, you can use the following code:<br>
<code>roslaunch p_g11_bringup game_bringup.launch</code><br><br>
This will spawn in the gazebo world 3 robots for each team (red, green and blue).<br>
#### Visaulize one robot in RVIZ
You can see one of these robots in rviz by typing the following code instead:<br>
<code>roslaunch p_g11_bringup game_bringup.launch visualize:=red1</code><br><br>
And substitute **red1** by the name of the player you want.

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

In this [link](https://youtu.be/gR2uT4AY4Go) you can see this working in a simulation.

## GMapping
GMapping allows for data collection from the LIDAR sensor of our robot and form a map out of that.<br>
If you want to see the results just use:<br>
<code>rosrun gmapping slam_gmapping scan:=p_g11/scan _base_frame:=/p_g11/base_link _map_frame:=/p_g11/map _odom_frame:=/p_g11/odom</code><br><br>

## Team Hunt Game
### Before Starting the Game
Before starting the game you need to have the gazebo world opened and the robots spawned using the command shown previously for multiple robots. 

### Starting the game
Starting the game means you have to put the robots in automatic mode, for that just insert the code:<br>
<code>roslaunch p_g11_bringup auto_driver.launch</code><br><br>

###




## Referee
There couldn't be a game without a referee, so there is node provided by our teacher which evaluates multiple parameters of the game like for example, if a player gets caught, and gives/takes points accordingly.<br>
To play the game you need to run this program using this command:<br>
<code>rosrun th_referee th_referee</code><br><br>

## The Game
Now that you have everithing working you can just watch the game. In the terminal where you are running the referee is where the results will appear. There you can see the team scores, individual and also the best hunter and best survivor.<br>
By pressing this link you can see an example of the game working.


## Contributors
<ul>
<li>Miguel Da Costa Pereira</li>
<li>João Pedro Tira Picos Costa Nunes</li>
<li>João Diogo Alves Matias</li>
<li>Bartosz Bartosik</li>
</ul>