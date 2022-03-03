# Team Hunt - Group 11 - PSR 2021/2022

This repository was developed for the course of PSR. The goal was to simulate a team hunt game with robots and also be able to drive a robot manually.

## The game
This project consists of two basic modes: <br>
- Manual mode
- Automatic mode (Team Hunt)

Manual mode allows you to drive the robot using your own keyboard (just don't expect it to be like Gran Turismo). This also includes a visualization of what the robot is detecting (camera, laser), using RVIZ.<br>

Automatic mode or Team Hunt is a game where 1 team out of 3 chases the others and tries to touch them. The game works by score, losing points or winning points, accordingly with the team you represent and if you catch or get caught. <br>

## Constitution of the Repository
This repository has three packages: <br>
- p_g11_bringup - This is responsible for launching the other files, because it has all the launch files necessary to run this program;
- p_g11_description - Has the xacro files and the information about the building of the robots and the worlds;
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

It's also possible to choose the number of players per team. For example, if we want two player per team, 
we could write, in te terminal:<br>
<code>roslaunch p_g11_bringup game_bringup.launch number_players:=2</code><br><br>

#### Visaulize one robot in RVIZ
You can see one of these robots in rviz by typing the following code instead:<br>
<code>roslaunch p_g11_bringup game_bringup.launch visualize_red1:=true</code><br>
And substitute **red1** by the name of the player you want.

### Define goal in RVIZ
Also, if we want to give specific goal to a robot it's possible to run the following, 
in the terminal:<br>
<code>rosrun p_g11_core lidar_data.py __name:=red1</code><br>
And substitute **red1** by the name of the player you want.

## Manual Driving
After spawning just one robot you it's possible to drive it manualy by running the following command:<br>
<code>roslaunch p_g11_bringup myteleop.launch</code><br><br>
After this you can drive the robot with the keys shown in this table:

| Key         | Action                                  |
| ----------- |-----------------------------------------|
| w           | Increase the speed                      |
| x           | Decrease the speed                      |
| a           | Turn Left                               |
| d           | Turn Right                              |
| s           | Stop Turning, speed continuous the same |
| space       | Stop moving the robot                   |
| q           | Exit the program                        |

In this [link](https://youtu.be/gR2uT4AY4Go) you can see this working in a simulation.

## GMapping
GMapping allows for data collection from the LIDAR sensor of our robot and form a map out of that.<br>
If you want to see the results just use:<br>
<code> roslaunch p_g11_bringup gmapping.launch </code> <br><br>
This command must be inserted after spawning one robot visualizing it in rviz.

## Team Hunt Game
### Before Starting the Game
Before starting the game you need to have the gazebo world opened and the robots spawned using the command shown previously for multiple robots. 

### Starting the game
Starting the game means you have to put the robots in automatic mode, for that just insert the code:<br>
<code>roslaunch p_g11_bringup auto_driver.launch</code><br><br>


### Referee
There couldn't be a game without a referee, so there is node provided by our teacher which evaluates multiple parameters of the game like for example, if a player gets caught, and gives/takes points accordingly.<br>
To play the game you need to run this program using this command:<br>
<code>rosrun th_referee th_referee</code><br><br>

### The Game
Now that you have everithing working you can just watch the game. In the terminal where you are running the referee is where the results will appear. There you can see the team scores, individual and also the best hunter and best survivor. <br><br>
By pressing this [link](https://youtu.be/_iph7fTkby4) you can see an example of the game working.<br>
The simulation time of the game lasts around 9 min, but the real time is 80 seconds. This happens because the program runs very slowly on our computers.


## Contributors
<ul>
<li>Miguel Da Costa Pereira</li>
<li>João Pedro Tira Picos Costa Nunes</li>
<li>João Diogo Alves Matias</li>
<li>Bartosz Bartosik</li>
</ul>