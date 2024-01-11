# Duration defines a period between two time points.
# Messages of this datatype are of ROS Time following this design:
# https://design.ros2.org/articles/clock_and_time.html

# Seconds component, range is valid over any possible int32 value.
int32 sec

# Nanoseconds component in the range of [0, 10e9).
uint32 nanosec
