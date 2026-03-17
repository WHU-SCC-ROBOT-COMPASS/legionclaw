PointENU   position     # Position of the vehicle reference point (VRP) in the map reference frame.   The VRP is the center of rear axle.
Quaternion   orientation     # A quaternion that represents the rotation from the IMU coordinate (Right/Forward/Up) to the world coordinate (East/North/Up).
Point3D   linear_velocity     # Linear velocity of the VRP in the map reference frame.East/north/up in meters per second.
Point3D   linear_acceleration     # Linear acceleration of the VRP in the map reference frame. East/north/up in meters per square second.
Point3D   angular_velocity     # Angular velocity of the vehicle in the map reference frame.Around east/north/up axes in radians per second.
float64   heading     # The heading is zero when the car is facing East and positive when facing North
Point3D   linear_acceleration_vrf     # Linear acceleration of the VRP in the vehicle reference frame.Right/forward/up in meters per square second.
Point3D   angular_velocity_vrf     # Angular velocity of the VRP in the vehicle reference frame. Around right/forward/up axes in radians per second.
Point3D   euler_angles     # // Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.   // in world coordinate (East/North/Up)   // The roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.   // The pitch, in [-pi, pi), corresponds to a rotation around the x-axis.   // The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.   // The direction of rotation follows the right-hand rule.
