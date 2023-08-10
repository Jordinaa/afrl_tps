#!/usr/bin/env python

from multiprocessing import current_process
import socket
from xmlrpc.client import boolean
import threading

import time

from pymavlink import mavutil

AIRSPEED_CONFIG = {
    "MINAirspeed": 14,
    "CRUISEAirspeed": 18,
    "MAXAirspeed": 39
}

AGL_CONFIG = {
    "75m": 0,
    "100m": 5,
    "125m": 10,
    "150m": 15
}

AGL_LEVEL = {
    "75m": [0, 1, 2, 3, 4],
    "100m": [5, 6, 7, 8, 9],
    "125m": [10, 11, 12, 13, 14],
    "150m": [15, 16, 17, 18, 19]
}

MAV_WRAPPER_PORT = '14530'
FLIGHT_CARD_PORT = '14520'


class FlightCardListener():
    def __init__(self):
        self.port = PORT
        self.max_connections = MAXCONNECTIONS
        self.correct_msg_len = 6
        self.correct_perf_len = 5

    def is_msg_correct(self, client_msg) -> boolean:
        if len(client_msg) == self.correct_msg_len:
            return True
        else:
            return False

    def is_perform_msg_correct(self, client_msg) -> boolean:
        if len(client_msg) == self.correct_perf_len:
            return True
        else:
            return False

    def parse_perform_msg(self, client_msg) -> list:
        time_duration = float(client_msg[1])
        velocity = float(client_msg[2])
        airspeed_key = client_msg[3]
        altitude_key = client_msg[4]

        return [airspeed_key, altitude_key]


def close_socket(socket: socket):
    print("closing socket", socket)
    socket.close()


def send_airspeed_command(airspeed, master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        0,
        airspeed,
        -1,
        0, 0, 0, 0
    )
    print("change airspeed")


if __name__ == '__main__':
    print('Listening to Mavlink Ports')
    master = mavutil.mavlink_connection('udpin:0.0.0.0:' + FLIGHT_CARD_PORT)

    master.wait_heartbeat()
    print("Heartbeat Recieved")

    PORT = 9876
    MAXCONNECTIONS = 2

    THIS_IP = "192.168.1.123"
    BUFFSIZE = 1024

    listensocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listensocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listensocket.bind((THIS_IP, PORT))
    listensocket.listen(MAXCONNECTIONS)

    fc_listener = FlightCardListener()

    while True:
        clientsocket = None

        try:
            clientsocket, clientaddress = listensocket.accept()
            message_packet = clientsocket.recv(BUFFSIZE).decode()
            clientsocket.setblocking(0)
            message = message_packet.split()
            print("message is", message)

            if "PERFORMANCE" in message:
                print("its performance")

                if fc_listener.is_perform_msg_correct(message):
                    airspeed_key, alt_key = fc_listener.parse_perform_msg(message)
                    print("retrieved messages: ", airspeed_key, alt_key)

                    if alt_key not in AGL_CONFIG:
                        print("no alt key")
                        continue

                    if airspeed_key not in AIRSPEED_CONFIG:
                        print("no airspeed")
                        continue

                    print("alt key ", AGL_CONFIG[alt_key])
                    print("airspeed key ", AIRSPEED_CONFIG[airspeed_key])

                    airspeed_val = AIRSPEED_CONFIG[airspeed_key]
                    des_wp = AGL_CONFIG[alt_key]
                    curr_wp = master.waypoint_current()

                    send_airspeed_command(airspeed_val, master)

                    curr_wp = master.waypoint_current()
                    des_level = AGL_LEVEL[alt_key]

                    if curr_wp in des_level:
                        print("at desired level", curr_wp, des_level)
                        print("\n---------------------")
                        continue

                    else:
                        while True:
                            time.sleep(0.2)
                            master.waypoint_set_current_send(des_wp)
                            curr_wp = master.waypoint_current()

                            if curr_wp == des_wp:
                                print("going to waypoint")
                                break

            else:
                print("bad inputs")

            print("\n---------------------")

        except IOError:
            print("io error")
            continue
