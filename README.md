# SLAM-using-GTSAM
Simultaneous Mapping and Localization framework for drone

Introduction:
This project is aimed at implementing a vision-based
Simultaneous Localization and Mapping (SLAM) to
genreate a 3D map of the environment using a GTSAM
and iSAM packages from GTSAM matlab toolbox. The
premise of the project is to find the survivors in a power
plant that has been destroyed by a recent earthquake.
Due to the hazardous materials, drones (quadcopters)
will be used for this mission. There are two parts to the
mission. First, building the map of the current condition
of the plant using a light weight drone with nothing but
a single camera and an IMU. Then use a second drone is
capable of carrying other needed supplies for the rescue
mission. The second drone will use the map from the
first drone and go to the rescue target location with the
supplies.

Algorithm: 
The algorithm for estimating 3D world co-ordinates
of camera poses and AprilTags using GTSAM involves
3 steps:
1) Initialisation
For GTSAM packages to optimize the world co-
ordinates properly, the world co-ordinated are to
be initialized properly. Knowing that bottom-left
corner of tag 10 will be origin in world co-
ordinates, the poses of the camera frames con-
sisting of tag 10 can be computed with RPnP
approach.

2) Building the Factor Map
The factor graph is then built using GTSAM pack-
ages for MATLAB, with the initial estimates, mea-
surements in pixel co-ordinates and noise models.

3) Optimising the factor graph
The factor graph, thus created, is optimized using
Levenberg-Marquadt non-linear optimizer.

The iSAM part of the project too follows a similar
approach. However, the factor graph are created and
optimized in batches (usually 1 frame at a time), as
opposed to all the frames at once.
