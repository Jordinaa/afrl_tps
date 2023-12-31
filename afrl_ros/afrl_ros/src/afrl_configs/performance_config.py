"""airspeed config in m/s"""
# FOR TIMBER 
# AIRSPEED_CONFIG = {
#     "MINAirspeed": 11, 
#     "CRUISEAirspeed": 13,
#     "MAXAirspeed": 25
# }

# """altitude config, connects to waypoint frame"""
# AGL_CONFIG = {
#     "75m" : 0,
#     "100m": 5,
#     "125m": 10,
#     "150m": 15
# }

# AGL_LEVEL = {
#     "75m" : [0,1,2,3,4],
#     "100m": [5,6,7,8,9],
#     "125m": [10,11,12,13,14],
#     "150m": [15,16,17,18,19]
# }

# FOR ESPARRO #################################################
AIRSPEED_CONFIG = {
    "MINAirspeed": 14, 
    "CRUISEAirspeed": 18,
    "MAXAirspeed": 30
}

"""altitude config, connects to waypoint frame"""
# altitude information is contained in the waypoints itself from the afrl_waypoint
AGL_CONFIG = {
    "75m" : 0,
    "100m": 5,
    "125m": 10,
    "150m": 15
}

AGL_LEVEL = {
    "75m" : [0,1,2,3,4],
    "100m": [5,6,7,8,9],
    "125m": [10,11,12,13,14],
    "150m": [15,16,17,18,19]
}