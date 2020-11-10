# Robot-arm-of-6-DOF-ABB-IRB7600-Kinematics-

This is the final project of the Robotics I module at the "Universidad Nacional de Cuyo", Mendoza - Argentina. (2018)

In this project, we worked on the kinematic aspects of an industrial robot with 6 degrees of freedom. It was about an ABB commercial robot model IRB 7600-400/2.55..
We guided the project towards a real industrial application, such as the pouring of molten steel into crucibles. We considered it to be a risky application for a human operator.

Basically, the idea was that the robot was able to take the containers with molten steel that are in the oven and then pour it into crucibles that are located on a table that is located on the side.

<p align="center">
<img src="/img/RobotArm7600.jpg" alt="Robot"
	title="Robot Arm ABB" width="350" height="400" />
</p>



To carry out the project, we used Matlab as a tool (to develop the code) and Peter Cork's toolbox available to link it to Maltab.

More specifically, the topics that were treated:

<ul>
<li>Axis assignment</li>
<li>Determination of the parameters of the Denavit - Hartenberg matrix</li>
<li>Direct kinematics</li>
<li>Inverse kinematics (Geometric method) -- Validation with numerical method provided by the toolbox</li>
<li>Direct and inverse Jacobian matrix</li>
<li>Work space</li>
<li>Ellipse manipulability -- Singularities</li>
<li>Trajectory planning</li>
<li>Simulation</li>
</ul>

<p align="center">
<b1>____________________________________________________________________________________________________________________________</b1>
</p>

<h2>Some results obtained:</h2>

<h3>Axis assignment</h3>


<img src="/img/axis_assignement.PNG" alt="Axis"
	title="Robot Arm ABB" width="300" height="250" />

<h3>Work space</h3>

<table>
<tbody>
<tr>
<td>
<p align="center">
<img src="/img/work_space.PNG" alt="Work space"
	title="Robot Arm ABB" width="200" height="230" />
</p>
</td>
<td>
<p align="center">
<img src="/img/work_space1.PNG" alt="Work space"
	title="Robot Arm ABB" width="300" height="250" />
</p>
</td>
</tr>
</tbody>
</table>

<h3>Ellipse manipulability -- Singularities </h3>

<table>
<tbody>
<tr>
<td>
<p align="center">
<img src="/img/3D_model_singularity2.PNG" alt="Singularity1"
	title="Robot Arm ABB" width="250" height="200" />
</p>
</td>
<td>
<p align="center">
<img src="/img/3D_model_singularity.PNG" alt="Singularity2"
	title="Robot Arm ABB" width="250" height="200" />
</p>
</td>
</tr>
</tbody>
</table>

<h3>Simulation (3D Model) of a trajectory. Putting steel into the furnace</h3> 

<b>[Click on the image below to watch the video]</b>

[![IMAGE ALT TEXT](https://img.youtube.com/vi/jbBpdFrF7ac/hqdefault.jpg)](https://youtu.be/jbBpdFrF7ac "Simulation")

	
<p align="center">
<b1>____________________________________________________________________________________________________________________________</b1>
</p>

<h3>Authors:</h3>

CORREA, Elias

CANTALOUBE, Adrian

ELORGA, Eliseo

<h3>Guidance:</h3>
	
DIAZ, Carolina 

SANCHEZ, Eric

