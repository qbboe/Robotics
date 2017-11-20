# Robotics

This Project was developed for the Robotics exam in Summer 2016 (Mark: 30/30).
It consists of a robot navigation algorithm: the robot has to reach a goal in a labyrinth without knowing the target location, using proximity sensors and the current distance to the target. We used the well-known BUG-1 algorithm.
In particular, I focused on the trilateration algorithm logic and the decision making problem regarding whether switching between "boundary following" and "motion to goal", given the trilateration data acquired.
The algorithm is written in C and we used MORSEE Simulator to test and optimize it.
