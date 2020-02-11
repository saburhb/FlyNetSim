from __future__ import print_function
import time
import math
import zmq
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil # Needed for command message definitions


class UAV:
    def __init__(self, uavid, sitl_port, tel_socket, control_socket, verbose=False):

        self.telemetry_freq = 1

        #Parameters for uav movements
        self.set_initial_alt = 10
        self.left_right_metres = 10
        self.forward_backward_metres = 10
        self.up_down_metres = 10
        self.groundspeed = 10

        self.uav_id = uavid
        self.home_location = None
        self.control_msg_count = 0
        self.tel_msg_count = 0
        self.verbose = verbose
        self.status = "NONE"
        self.last_heartbeat = 0
        self.last_seq = 0
        self.prefix = "[UAV_" + self.uav_id + "]"

        self.tel_last_heartbeat = None
        self.tel_battery = None
        self.tel_velocity = None
        self.tel_attitude = None
        self.tel_location_global = None
        self.tel_location_global_relative = None
        self.current_location = None
        self.tel_airspeed = None
        self.tel_groundspeed = None
        self.tel_heading = None
        self.tel_timestamp = time.time()

        # Setting up ZMQ related parameters
        # self.zmq_tel_port = tel_port
        # self.zmq_control_port = control_port
        # self.zmq_tel_connection_str = "tcp://127.0.0.1:" + str(self.zmq_tel_port)
        # self.zmq_control_connection_str = "tcp://127.0.0.1:" + str(self.zmq_control_port)
        # self.zmq_tel_socket = self.create_zmq("PUB", self.zmq_tel_connection_str, "", verbose=self.verbose)
        # self.zmq_control_socket = self.create_zmq("SUB", self.zmq_control_connection_str, verbose=self.verbose)
        self.zmq_tel_socket = tel_socket
        self.zmq_control_socket = control_socket

        # Connecting to the Vehicle
        self.sitl_port = sitl_port
        self.sitl_connection_str = "tcp:127.0.0.1:" + str(self.sitl_port)
        self.vehicle = None
        #self.vehicle = self.connect_sitl(self.sitl_connection_str, self.verbose)
        #self.condition_yaw(90)



        # Thread for sending sensor (NOT IMPLEMENTED FOR NOW)
        thread_tel = threading.Thread(target=self.send_sensor_data)
        thread_tel.daemon = True
        thread_tel.start()

        self.get_data(self.zmq_control_socket, verbose)


        self.connection_close(self.verbose)

    def connect_sitl(self, connection_string, verbose):
        if verbose:
            print("%s Connecting to vehicle on: %s" % (self.prefix, connection_string))
        vehicle = connect(connection_string)
        # Setting callback on telemetry attribute change
        return vehicle

    def telemetry_add_attr(self, vehicle, verbose):
        if verbose:
            print(self.prefix + " Adding telemetry attribute callback to detect any change")
        vehicle.add_attribute_listener("*", self.telemetry_update)
        return True

    def telemetry_remove_attr(self, vehicle, verbose):
        if verbose:
            print(self.prefix + " Removing telemtry attribute observer")
        vehicle.remove_attribute_listener("*", self.telemetry_update)

    def telemetry_update(self, self1, attr_name, value):
        #if self.verbose:
        #    print("%s TELEMETRY: (%s): %s" % (self.prefix, attr_name, value))
        if "last_heartbeat" in str(attr_name):
            self.tel_last_heartbeat = str(attr_name) + "|" + str(value)
        elif "battery" in str(attr_name):
            self.tel_battery = str(attr_name) + "|" + str(value)
        elif "velocity" in str(attr_name):
            self.tel_velocity = str(attr_name) + "|" + str(value)
        elif "attitude" in str(attr_name):
            self.tel_attitude = str(attr_name) + "|" + str(value)
        elif "global_relative" in str(attr_name):
            self.tel_location_global_relative = str(attr_name) + "|" + str(value)
        elif "global_frame" in str(attr_name):
            self.current_location = value
            self.tel_location_global = str(attr_name) + "|" + str(value)
        elif "airspeed" in str(attr_name):
            self.tel_airspeed = str(attr_name) + "|" + str(value)
        elif "groundspeed" in str(attr_name):
            self.tel_groundspeed = str(attr_name) + "|" + str(value)
        elif "heading" in str(attr_name):
            self.tel_heading = str(attr_name) + "|" + str(value)
        if (time.time() - self.tel_timestamp) > self.telemetry_freq:
            msg = ""
            if self.tel_location_global_relative is not None:
                msg += self.tel_location_global_relative + "#"
                self.tel_location_global_relative = None
            if self.tel_location_global is not None:
                msg += self.tel_location_global + "#"
                self.tel_location_global = None
            if self.tel_last_heartbeat is not None:
                msg += self.tel_last_heartbeat + "#"
                self.tel_last_heartbeat = None
            if self.tel_battery is not None:
                msg += self.tel_battery + "#"
                battery = self.tel_battery
                self.tel_battery = None
            if self.tel_velocity is not None:
                msg += self.tel_velocity + "#"
                self.tel_velocity = None
            if self.tel_attitude is not None:
                msg += self.tel_attitude + "#"
                self.tel_attitude = None
            if self.tel_airspeed is not None:
                msg += self.tel_airspeed + "#"
                self.tel_airspeed = None
            if self.tel_groundspeed is not None:
                msg += self.tel_groundspeed
                self.tel_groundspeed = None
            if self.tel_heading is not None:
                msg += self.tel_heading + "#"
                self.tel_heading = None
            if len(msg) > 0:
                self.send_data("TELEMETRY#" + msg, self.zmq_tel_socket, self.verbose)
                if self.home_location is not None and self.current_location is not None:
                    x, y = self.get_distance_metres(self.home_location, self.current_location)
                    file_w = open("plot.csv", "a")
                    file_w.write(str(x) + '\t' + str(y) + '\t' + str(battery[battery.find("level=")+6:]) + '\n')
                    file_w.close()
                    print(str(x) + '\t' + str(y) + '\t' + str(battery[battery.find("level=")+6:]))
                    self.zmq_tel_socket.send("@@@U_" + self.uav_id + "***" + "DISTANCE***" + str(x) + "***" + str(y) + "***")
                self.tel_timestamp = time.time()

    def create_zmq(self, zmq_type, con_string, prefix="", verbose=False):
        context = zmq.Context()
        if "PUB" in zmq_type:
            if verbose:
                print(self.prefix + " [ZMQ] Binding publisher started " + con_string)
            sock_new = context.socket(zmq.PUB)
            sock_new.bind(con_string)
            if verbose:
                print(self.prefix + " [ZMQ] Publisher bound complete " + con_string)
        elif "SUB" in zmq_type:
            if verbose:
                print(self.prefix + " [ZMQ] Subscriber connect started " + con_string)
            sock_new = context.socket(zmq.SUB)
            sock_new.connect(con_string)
            sock_new.setsockopt(zmq.SUBSCRIBE, prefix)
            if verbose:
                print(self.prefix + " [ZMQ] Subscriber connect complete " + con_string + " Prefix " + prefix)
        else:
            return None
        return sock_new

    def send_data(self, message, sock, verbose):
        try:
            self.tel_msg_count += 1
            msg_send = "@@@U_" + self.uav_id + "***" + str(self.tel_msg_count) + "***" +\
                       str(time.time()) + "***" + message + "***"
            if verbose:
                print(self.prefix + " TELEMETRY: sending " + msg_send)
            sock.send(msg_send)
        except:
            print(self.prefix + " TELEMETRY: Exception occurred while sending data")

    def get_data(self, socket, verbose):
        while True:
            data = socket.recv()
            if "TERMINATE" in data:
                break
            if data:
                if verbose:
                    print(self.prefix + " CONTROL: Message received :" + data)
                d_list = data.split('***')
                self.control_msg_count += 1
                gcs_msg_size = len(d_list)
                print(">>>>>> LENGTH OF LIST FROM GCS: " + str(gcs_msg_size))

                #Information received from the command message
                self.last_seq = int(d_list[1])
                seq_no = int(d_list[1])
                gcs_t1_timestamp = float(d_list[2])
                gcs_t2_timestamp = time.time()
                command = d_list[3].split(":")

                if (gcs_msg_size > 5):
                    ns_t1_timestamp = float(d_list[4])
                    ns_t2_timestamp = float(d_list[5])
                else:  #for DIRECT communication bypassing ns-3
                    ns_t1_timestamp = gcs_t2_timestamp
                    ns_t2_timestamp = gcs_t2_timestamp

	        real_time_precision_tolerance = 0.05 


		###### Do Synchronization between flysim and netsim ###################
		delta = (ns_t2_timestamp - ns_t1_timestamp)/1000
		print("GCS Send Timestamp: " + repr(gcs_t1_timestamp))
		print("GCS Current Timestamp: " + repr(time.time()))
		print("Simulated Network Delay: " + repr(delta))
		if (gcs_t1_timestamp + delta) > gcs_t2_timestamp:
		    time.sleep((gcs_t1_timestamp + delta) - gcs_t2_timestamp)
		elif (gcs_t1_timestamp + delta + real_time_precision_tolerance) < gcs_t2_timestamp:
		    print("Receiving time : " + repr(gcs_t2_timestamp) + " exceeds estimated arrival time: "+ repr(gcs_t1_timestamp + delta))
		    continue # Skip and do not perform action when network simulation is delayed


                if "HEARTBEAT" in command[1]:
                    if self.verbose:
                        print("HEARTBEAT ", command[1])
                    self.last_heartbeat = time.time()

                elif "DISCONNECT" in command[1]:
                    status = self.disconnect_vehicle(verbose)
                    self.status = "DISCONNECT"
                    self.send_data("STATUS#DISCONNECT|"+str(status), self.zmq_tel_socket, self.verbose)

                elif "CONNECT" in command[1]:
                    self.vehicle = self.connect_sitl(self.sitl_connection_str, self.verbose)
                    self.condition_yaw(90)
                    status = self.telemetry_add_attr(self.vehicle, self.verbose)
                    time.sleep(2)
                    self.status = "CONNECT"
                    self.send_data("STATUS#CONNECT|"+str(status), self.zmq_tel_socket, self.verbose)

                elif "DISARM" in command[1]:
                    status = self.arm_disarm_throttle(self.vehicle, "DISARM", verbose)
                    self.status = "DISARM"
                    self.send_data("STATUS#DISARM|"+str(status), self.zmq_tel_socket, self.verbose)

                elif "ARM" in command[1]:
                    self.arm_disarm_throttle(self.vehicle, "ARM", verbose)
                    status = self.arm_disarm_throttle(self.vehicle, "ARM", verbose)
                    self.status = "ARM"
                    self.send_data("STATUS#ARM|"+str(status), self.zmq_tel_socket, self.verbose)

                elif "TAKEOFF" in command[1]:
                    self.set_mode(self.vehicle, "GUIDED", self.verbose)
                    self.set_groundspeed(self.vehicle, self.groundspeed, self.verbose)
                    self.vehicle.airspeed = 10
                    status = self.takeoff(self.vehicle, self.set_initial_alt, self.verbose)
                    self.home_location = self.current_location
                    self.status = "TAKEOFF"
                    self.send_data("STATUS#TAKEOFF|"+str(status), self.zmq_tel_socket, self.verbose)

                elif "RTL" in command[1]:
                    status = self.return_to_launch(self.vehicle, self.verbose)
                    self.status = "RTL"
                    self.send_data("STATUS#RTL|"+str(status), self.zmq_tel_socket, self.verbose)

                elif "LAND" in command[1]:
                    status = self.land(self.vehicle, self.verbose)
                    self.status = "LAND"
                    self.send_data("STATUS#LAND|"+str(status), self.zmq_tel_socket, self.verbose)

                elif "GO_UP" in command[1]:
                    self.go_up(self.vehicle, command[1], verbose)

                elif "GO_DOWN" in command[1]:
                    self.go_down(self.vehicle, command[1], verbose)

                elif "GO_FORWARD" in command[1]:
                    self.go_forward(self.vehicle, command[1], verbose)

                elif "GO_BACKWARD" in command[1]:
                    self.go_backward(self.vehicle, command[1], verbose)

                elif "GO_LEFT" in command[1]:
                    self.go_left(self.vehicle, command[1], verbose)

                elif "GO_RIGHT" in command[1]:
                    self.go_right(self.vehicle, command[1], verbose)

                elif "GO_TO" in command[1]:
                    self.go_to(self.vehicle, command[1], verbose)

                else:
                    print("UNKOWN COMMAND")

        self.connection_close(self.verbose)

    def send_sensor_data(self):
        while True:
            if "TAKEOFF" in self.status:
                self.send_data("SENSOR|", self.zmq_tel_socket, self.verbose)
            time.sleep(5)

    def set_mode(self, vehicle, mode, verbose):
        if verbose:
            print(self.prefix + " Setting the vehicle mode to " + mode)
        vehicle.mode = VehicleMode(mode)
        if vehicle.mode == VehicleMode(mode):
            return True
        else:
            return False

    def set_groundspeed(self, vehicle, speed, verbose):
        if verbose:
            print(self.prefix + " Setting the vehicle groundspeed")
        vehicle.groundspeed = int(speed)
        if verbose:
            print(self.prefix + " Vehicle groundspeed speed set to " + str(speed))
        return True

    def arm_disarm_throttle(self, vehicle, cmd, verbose):
        if cmd == "ARM":
            if verbose:
                print(self.prefix + " Basic pre-arm-checks")
            while not vehicle.is_armable:
                if verbose:
                    print(self.prefix + " Waiting for vehicle to initialise...")
                    time.sleep(1)
            vehicle.armed = True
            while not vehicle.armed:
                if verbose:
                    print(self.prefix + " Waiting for arming...")
                vehicle.armed = True
                time.sleep(1)
            print(self.prefix + " Vehicle armed")
            return True
        elif cmd == "DISARM":
            if verbose:
                print(self.prefix + " Starting vehicle disarm")
            vehicle.armed = False
            while vehicle.armed:
                if verbose:
                    print(self.prefix + " Waiting for disarm...")
                vehicle.armed = False
                time.sleep(1)
            print(self.prefix + " Vehicle disarmed")
            return True
        else:
            return False

    def takeoff(self, vehicle, target_altitude, verbose):
        if verbose:
            print(self.prefix + " Vehicle taking off!")
        target_altitude = int(target_altitude)
        vehicle.simple_takeoff(target_altitude)
        while True:
            if verbose:
                print(self.prefix + " Altitude: ", vehicle.location.global_relative_frame.alt)
            if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                if verbose:
                    print(self.prefix + " Reached target altitude")
                break
            time.sleep(1)
        return True

    def land(self, vehicle, verbose):
        if verbose:
            print(self.prefix + " Vehicle Landing!")
        vehicle.mode = VehicleMode("LAND")
        if verbose:
            print(self.prefix + " Vehicle Lander!")
        return True

    def return_to_launch(self, vehicle, verbose):
        if verbose:
            print(self.prefix + " Vehicle Landing!")

        vehicle.mode = VehicleMode("RTL")

        if verbose:
            print(self.prefix + " Vehicle Lander!")

        return True

    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_roi(self, location):
        """
        Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a
        specified region of interest (LocationGlobal).
        The vehicle may also turn to face the ROI.

        For more information see:
        http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
        """
        # create the MAV_CMD_DO_SET_ROI command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            location.lat,
            location.lon,
            location.alt
        )
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    """
    Functions to make it easy to convert between the different frames-of-reference. In particular these
    make it easy to navigate in terms of "metres from the current position" when using commands that take 
    absolute positions in decimal degrees.

    The methods are approximations only, and may be less accurate over longer distances, and when close 
    to the Earth's poles.

    Specifically, it provides:
    * get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
    * get_distance_metres - Get the distance between two LocationGlobal objects in metres
    * get_bearing - Get the bearing in degrees to a LocationGlobal
    """

    def get_location_metres(self, original_location, dNorth, dEast, alt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(newlat, newlon, alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(newlat, newlon, alt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation;

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
        x = dlat * 1.113195e5
        y = dlong * 1.113195e5
        return x, y

    def get_bearing(self, aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing;

    """
    Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

    The methods include:
    * goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
    * goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
        MAV_FRAME_BODY_NED frame
    * goto - A convenience function that can use Vehicle.simple_goto (default) or 
        goto_position_target_global_int to travel to a specific position in metres 
        North and East from the current location. 
        This method reports distance to the destination.
    """

    def goto_position_target_global_int(self, aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

        For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,  # type_mask (only speeds enabled)
            aLocation.lat * 1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon * 1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt,
            # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0,  # X velocity in NED frame in m/s
            0,  # Y velocity in NED frame in m/s
            0,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def goto_position_target_local_ned(self, north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.

        It is important to remember that in this frame, positive altitudes are entered as negative
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.

        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see:
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.

        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def goto(self, dNorth, dEast, gotoFunction):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """

        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = self.get_location_metres(currentLocation, dNorth, dEast)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        gotoFunction(targetLocation)

        # print "DEBUG: targetLocation: %s" % targetLocation
        # print "DEBUG: targetLocation: %s" % targetDistance

        while self.vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            # print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = self.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance <= targetDistance * 0.01:  # Just below target, in case of undershoot.
                print("Reached target")
                break;
            time.sleep(2)

    """
    Functions that move the vehicle by specifying the velocity components in each direction.
    The two functions use different MAVLink commands. The main difference is
    that depending on the frame used, the NED velocity can be relative to the vehicle
    orientation.

    The methods include:
    * send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
    * send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
    """

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.

        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
        velocity components
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.

        This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only
        velocity components
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).

        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x,  # X velocity in NED frame in m/s
            velocity_y,  # Y velocity in NED frame in m/s
            velocity_z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def go_to(self, vehicle, location, verbose):
        if verbose:
            print(self.prefix + " Moving the vehicle to ", str(location))
        point = LocationGlobalRelative(location.lat, location.lon, self.set_initial_alt)
        vehicle.simple_goto(point)
        return True

    def go_up(self, vehicle, command, verbose):
        if verbose:
            print(self.prefix + " Vehicle going up")
        self.set_initial_alt += self.up_down_metres
        loc = self.get_location_metres(self.current_location, 0, 0, self.current_location.alt + 10)
        self.go_to(vehicle, loc, verbose)
        #self.goto_position_target_local_ned(0, 0, -self.up_down_metres)

    def go_down(self, vehicle, command, verbose):
        if verbose:
            print(self.prefix + " Vehicle going down")
        self.set_initial_alt -= self.up_down_metres
        loc = self.get_location_metres(self.current_location, 0, 0, self.current_location.alt - 10)
        self.go_to(vehicle, loc, verbose)
        #self.goto_position_target_local_ned(0, 0, self.up_down_metres)

    def go_left(self, vehicle, command, verbose):
        if verbose:
            print(self.prefix + " Vehicle going left")
        loc = self.get_location_metres(self.current_location, 0, -self.left_right_metres, self.current_location.alt)
        self.go_to(vehicle, loc, verbose)
        #self.goto_position_target_local_ned(0, self.left_right_metres, 0)

    def go_right(self, vehicle, command, verbose):
        if verbose:
            print(self.prefix + " Vehicle going right")
        loc = self.get_location_metres(self.current_location, 0, self.left_right_metres, self.current_location.alt)
        self.go_to(vehicle, loc, verbose)
        #self.goto_position_target_local_ned(0, -self.left_right_metres, 0)

    def go_forward(self, vehicle, command, verbose):
        if verbose:
            print("Vehicle going forward")
        loc = self.get_location_metres(self.current_location, self.forward_backward_metres, 0, self.current_location.alt)
        self.go_to(vehicle, loc, verbose)
        #self.goto_position_target_local_ned(self.forward_backward_metres, 0, 0)

    def go_backward(self, vehicle, command, verbose):
        if verbose:
            print(self.prefix + " Vehicle going backward")
        loc = self.get_location_metres(self.current_location, -self.forward_backward_metres, 0, self.current_location.alt)
        self.go_to(vehicle, loc, verbose)
        #self.goto_position_target_local_ned(-self.forward_backward_metres, 0, 0)

    def disconnect_vehicle(self, verbose):
        if verbose:
            print(self.prefix + " Disconnecting vehicle")
        # Remove observer added with `add_attribute_listener()
        self.telemetry_remove_attr(self.vehicle, self.verbose)
        self.vehicle.close()
        return True

    def connection_close(self, verbose):
        if verbose:
            print(self.prefix + " Closing connections")
        self.zmq_control_socket.close()
        self.zmq_tel_socket.close()
        self.disconnect_vehicle(verbose)
