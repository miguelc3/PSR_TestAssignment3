# Team Hunt - PSR 2021/2022

## Index
Generate Index


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


## 


# command for gmapping
rosrun gmapping slam_gmapping scan:=p_g11/scan _base_frame:=/p_g11/base_link _map_frame:=/p_g11/map _odom_frame:=/p_g11/odom



## Contributors
<ul>
<li>Miguel Da Costa Pereira</li>
<li>João Pedro Tira Picos Costa Nunes</li>
<li>João Diogo Alves Matias</li>
<li>Bartosz Bartosik</li>
</ul>