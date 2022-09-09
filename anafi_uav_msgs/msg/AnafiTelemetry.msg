# Telemetry data from Anafi drone. Published at 5 Hz as this is the
# update frequency of the drone output

Header header

# Attitude in Euler angles 
float64 roll    # [rad]
float64 pitch   # [rad]
float64 yaw     # [rad] 0 = north, pi/2 = east

# Body velocity
float64 u       # [m/s] positive forward
float64 v       # [m/s] positive right
float64 w       # [m/s] positive down

# Relative altitude
float64 relative_altitude # negative, as in NED

# Gimbal attitude
float64 gimbal_roll
float64 gimbal_pitch # 0 at horizon, positive upwards
float64 gimbal_yaw

# Battery information
float64 battery_percentage
string battery_health
string battery_serial_number

# Flying state
string flying_state
