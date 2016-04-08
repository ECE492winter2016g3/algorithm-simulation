
# Java SLAM Library

The java Simultaneous Localization and Mapping library included allows a Robot whose motion is limited to linear moves and on-the-spot rotations to localize its position relative to previously observed features and build a map of java. The library is accessed by creating an instance of the "mapping" class in mapping.java, and calling its API as data is recorded from the environment and the Robot moves.

The library also includes a testbed on which to render datasets for testing and debugging purposes, in test.java.

# Prerequisites

* Standard Java instalation

* ImageMagik for testbed usage

# Running the Testbed

Running

> make runjava

Will compile and run the testbed.

# Using the library

First, create an instance of the mapping class, then call the following functions:

1) mapping.init();

Resets the Robot back to a fresh empty state, unrotated, at the origin of the map.

2) mapping.initialScan(float[] angles, float[] distances)

Records the initial local geometry around the Robot to use as a starting point for localization.

3) mapping.updateRot/mapping.updateLin(float[] angles, float distances[], float hint)

Updates the robot position or rotation using a new data set. hint must be set to a guess of the
change in rotation or position based on odometry fromt the Robot. The hint does not have to be
particularly accurate past the sign being correct.

4) mapping.getAngle() / mapping.getPosition()

Get the current angle or position of the Robot to make use of in other code.

As an examlple, test.java includes two examples: testDataSet, and testShape. testDataSet shows testing with data logged from the mobile phone app in a Robot, as angle/distance pairs in a csv file. testShape shows testing with a shape defined in code.
