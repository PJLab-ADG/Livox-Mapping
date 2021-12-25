# Header include seq, timestamp(last node pub time), and frame_id(sensor model)
std_msgs/Header header

# The time of pose measurement, in nano second.
uint64 measurement_time_ns

# Report localization status
# when INVALID or CONVERGING, localization result is not reliable and should not be used.
string status_info

# For debug
string debug_info

# Position of the Vehicle Reference Point(VRP) in the Map reference frame(East/North/Up).
# The VRP is the car center(Forward/Left/Up).
# x for East, y for North, z for Up Height, in meters.
geometry_msgs/Point position_enu
# Position uncertainty(standard deviation in meters)
geometry_msgs/Point position_enu_std_dev

# Position of the Vehicle Reference Point(VRP) in the WGS84 reference ellipsoid coordinate system.
# longitude in degrees, ranging from -180 to 180.
# latitude in degrees, ranging from -90 to 90.
# ellipsoid height in meters.
float64 longitude
float64 latitude
float64 altitude

# Attitude in euler angle form to describe the orientation of a VRP frame with respect to a Map reference frame(ENU).
# The roll, in (-pi/2, pi/2), corresponds to a rotation around the Vehicle Forward-axis.
# The pitch, in [-pi, pi), corresponds to a rotation around the Vehicle Left-axis.
# The yaw, in [-pi, pi), corresponds to a rotation around the Vehicle Up-axis.
# The yaw is the angle of the Vehicle head w.r.t the ENU reference frame, East = 0, North = pi/2, anti-clockwise in radius.
# The direction of rotation follows the right-hand rule.
float64 roll
float64 pitch
float64 yaw

# Attitude uncertainty(standard deviation in radians)
geometry_msgs/Point attitude_std_dev

# Linear velocity of the VRP in the Vehicle reference frame
# x for Forward, y for Left, z for Up, in meters per second
geometry_msgs/Vector3 linear_velocity

# Linear velocity of the VRP in the ENU reference frame
# x for East, y for North, z for Up, in meters per second
geometry_msgs/Vector3 linear_velocity_global

# Linear acceleration of the VRP in the Vehicle reference frame
# x for Forward, y for Left, z for Up, in meters per power second
geometry_msgs/Vector3 linear_acceleration

# Linear acceleration of the VRP in the ENU reference frame
# x for East, y for North, z for Up, in meters per power second
geometry_msgs/Vector3 linear_acceleration_global

# Angular velocity of the VRP in the Vehicle reference frame
# x across Forward axes, y across Left axes,
# z across Up axes, in radians per second
geometry_msgs/Point angular_velocity
