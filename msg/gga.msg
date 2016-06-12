time stamp

# degrees WGS84
float64 latitude

# degrees WGS84
float64 longitude

uint8 INVALID = 0
uint8 GPS_FIX = 1
uint8 DPGS_FIX = 2
uint8 PPS_FIX = 3
uint8 REAL_TIME_KINEMATIC = 4
uint8 FLOAT_RTK = 5
uint8 DEAD_RECKONING = 6
uint8 MANUAL_INPUT = 7
uint8 SIMULATION = 8
uint8 fix_quality

uint16 num_satellites

# Horizontal dilution of position
float64 hdop

# Meters above mean sea level
float64 altitude

# Height of mean sea level above WGS84
float64 geoid_height


bool time_since_last_dgps_valid

# Time since last DGPS update
time time_since_last_dgps


bool dgps_station_id_valid

uint16 dgps_station_id
