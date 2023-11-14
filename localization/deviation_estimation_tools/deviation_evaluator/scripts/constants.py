# Threshold used for detecting invalid values for expected errors which tends to have significantly large values at the beginning of the bag file
# which is around 1e8 [m]. This value is used to ignore the initial part of the bag file.
THRESHOLD_FOR_INITIALIZED_ERROR = 100.0  # [m]
