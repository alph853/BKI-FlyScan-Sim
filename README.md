param set COM_ARM_IMU_ACC 0    # Disable accel calibration check
param set COM_ARM_MAG_STR 0    # Disable compass calibration check
param set COM_ARM_MAG_ANG 0    # Disable compass angle check

param set SYS_HAS_MAG 0    # Disable compass angle check
param set SYS_HAS_BARO 0    # Disable baro

param set SYS_HAS_GPS 0           # Tell system GPS not required
param set COM_ARM_WO_GPS 1


# Enable external vision as position source
param set EKF2_EV_CTRL 15         # Enable vision pos, vel, yaw fusion
param set EKF2_HGT_REF 3          # Use vision for height reference
param set EKF2_GPS_CTRL 0         # Disable GPS completely

# Vision sensor configuration
param set EKF2_EV_DELAY 10        # Vision delay (ms)
param set EKF2_EV_NOISE_MD 0      # Vision noise mode
param set EKF2_EV_POS_X 0.0       # Camera position relative to IMU
param set EKF2_EV_POS_Y 0.0
param set EKF2_EV_POS_Z 0.0

# Tune vision noise parameters
param set EKF2_EVP_NOISE 0.1      # Vision position noise
param set EKF2_EVV_NOISE 0.1      # Vision velocity noise
param set EKF2_EVA_NOISE 0.05     # Vision angle noise

# Allow arming with vision
param set COM_ARM_WO_GPS 1

