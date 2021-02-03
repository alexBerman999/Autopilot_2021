import pymavlink
from pymavlink import mavutil
from collections import defaultdict

def get_location(sampling_rate,time, channel): 
    connection = mavutil.mavlink_connection(channel)
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_system))
    iteration = time/sampling_rate
    location_array = defaultdict(list)
    connection.mav.request_data_stream_send(connection.target_system, connection.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, sampling_rate, 1)
    for i in range(iteration): 
        message = connection.recv_match(type = 'GPS_RAW_INT')
        if not message: 
            continue 
        elif message.get_type() == "BAD_DATA": 
            print("received poor data")
            continue 
        else: 
            location_array[i] = message 
    
    return location_array
