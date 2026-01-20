* Sequential, or batch the measurements?
  * https://math.stackexchange.com/questions/4011815/kalman-filtering-processing-all-measurements-together-vs-processing-them-sequen
  * Claude's take: https://claude.ai/chat/fb00d44d-0b88-47e7-9bc5-51ac5886b525
  
* Gazebo TB4 odometry: noisy, or ground-truth?
  * Seems-like "ground-truth": 
    * https://github.com/gazebosim/gz-math/blob/gz-math9/src/DiffDriveOdometry.cc
    * Covariances are all-zero in ROS for the `/odom` topic.
  * Looks like there's an `OdometryPublisher" in Gazebo, with a `gaussian_noise` parameter:
    https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html#details
  * Might be easier to simply add the noise at the start of `odom_cb()`, rather than screw-around 
  with SDF files and restarting the simulation to change noise params.
  * This is actually good, because it means I can experiment faster.

* Command for looking at ground-truth TB4 pose from `gz`: `gz topic -e -t /world/depot/pose/info | grep turtlebot4 -B1 -A13`
* Topic rates (hz):
  * `/odom`: 27
  * `/tag_detections`: 10

* Interesting discussion of implementing a blocking, bounded queue:
https://morestina.net/blog/1400/minimalistic-blocking-bounded-queue-for-c
