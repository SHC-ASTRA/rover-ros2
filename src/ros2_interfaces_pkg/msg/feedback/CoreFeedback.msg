# Topic: /core/feedback

# GPS Data
float64 gps_lat       # GPS latitude
float64 gps_long      # GPS longitude
int32 gps_sats        # Number of active GPS satellites

# BNO055 Sensor Data
geometry_msgs/Vector3 bno_gyro   # x, y, z gyroscope readings from BNO055
geometry_msgs/Vector3 bno_accel  # x, y, z acceleration readings from BNO055

# Rover Orientation
float64 orientation   # Orientation of the rover relative to North (0 - 359 degrees)

# BMP Sensor Data
float64 bmp_temp      # Temperature reading in Celsius
float64 bmp_alt       # Altitude reading in meters
float64 bmp_pres      # Pressure reading in millibars (might be hPa)

# Voltage Readings
float64 bat_voltage   # Current voltage of the battery
float64 voltage_12    # Current voltage of the 12V rail
float64 voltage_5     # Current voltage of the 5V rail
float64 voltage_3     # Current voltage of the 3.3V rail