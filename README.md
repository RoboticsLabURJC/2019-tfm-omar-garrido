# 2019-tfm-omar-garrido

This TFM aims to obtain a SLAM algorithm using a RGBD sensor like realsense D435 for a drone, in order to get the location of the drone in the environment and a dense mapping of the environment.

## Index

- [Week 1-4 Reading State of the Art SLAM Papers](#week1)


<a name="week1"></a>
## Week 1-4 Reading State of the Art SLAM Papers

### Step 1
First of all, I started by reading some of the advancements in SLAM algorithms done by the JdeRobot group. I read the doctoral thesis of **Eduardo Perdices** [Técnicas para la localización visual robusta de robots en tiempo real con y sin mapas](https://gsyc.urjc.es/jmplaza/students/phd-eduardo_perdices-2017.pdf).

In this thesis I learn about the basics of SLAM algorithm and the differents approaches that have been made over time to solve the problem of simultaneous location and mapping.

### Step 2
After that, since this is a problem using an RGBD camera, I started reading the TFM of **Alberto Martín** [Autolocalización visual 3D usando mapas
RTAB-Map](https://gsyc.urjc.es/jmplaza/students/tfm-visualslam-alberto_martin-2017.pdf).

In this project the SLAM problem is attacked using a pre-generated 3D map of the environment and using a RGB image to get the location in this map. Also RTAB-map tool is used to generate the 3D map.

### Step 3
Finally I proceed to the installation of the [SD-SLAM](https://github.com/JdeRobot/slam-SD-SLAM) software of the JdeRobot group.
