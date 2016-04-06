from dsrc.dsrc_dispatcher import DsrcEventDispatcher
from radar.radar_dispatcher import RadarEventDispatcher
import json
import math
import logging
from multiprocessing import Queue
from Queue import Empty


class Combiner(object):
    """ Takes DSRC+Radar information and forms a single model of the environment.

    This package is responsible for taking data from each source, normalizing the
    format, and combining that information into a single model of the car's
    environment. Once the combination is complete the new model gets passed to
    the registered callback function, which in our case feeds data into our collision
    avoidance system.
    """

    def __init__(self, collision_avoid_queue, log_dsrc=True, log_radar=True,
                 dsrc_log_file=None, radar_log_file=None, dsrc_enabled=True,
                 radar_enabled=True, log_level="DEBUG"):
        """ Setup Combiner, initialize DSRC+Radar event dispatcher. """
        self.dsrc_data = None
        self.radar_data = None

        self.data_count = 0

        self.dsrc_enabled = dsrc_enabled
        self.radar_enabled = radar_enabled

        self.data_queue = Queue()
        self.combined_data_queue = collision_avoid_queue
        # self.callback = callback


        if self.dsrc_enabled:
            self.dsrc_event_dispatcher = DsrcEventDispatcher(self.data_queue, log=log_dsrc, log_file=dsrc_log_file, log_level=log_level)
        if self.radar_enabled:
            self.radar_event_dispatcher = RadarEventDispatcher(self.data_queue, log=log_radar, log_file=radar_log_file, log_level=log_level)

        self.logger = logging.getLogger('debug_combined')

    def start(self):
        """ Start running the event dispatcher threads (we are ready to recieve data). """

        if self.dsrc_enabled:
            self.dsrc_event_dispatcher.start()
        if self.radar_enabled:
            self.radar_event_dispatcher.start()

        while self.dispatchers_alive():
            if self.data_queue.qsize() > 1:
                print 'WARNING: The queue depth is %s, we are behind real time!' % self.data_queue.qsize()

            try:
                dispatcher_data = self.data_queue.get(timeout=0.5)

                if dispatcher_data['from'] == 'dsrc':
                    self.dsrc_data_callback(dispatcher_data['data'])
                else:
                    self.radar_data_callback(dispatcher_data['data'])
            except Empty:
                pass
                # print 'Timeout'

        if self.dsrc_enabled:
            self.dsrc_event_dispatcher.terminate()
        if self.radar_enabled:
            self.radar_event_dispatcher.terminate()

    def dispatchers_alive(self):
        if self.dsrc_enabled and not self.dsrc_event_dispatcher.is_alive():
            return False

        if self.radar_enabled and not self.radar_event_dispatcher.is_alive():
            return False

        if not self.dsrc_enabled and not self.radar_enabled:
            return False

        return True

    def hex_to_int(self, h, d):
        # h = number to convert (can be integer)
        # d = number of bits in the number
        i = h
        if i >= 2**(d-1):
            i -= 2**d
        return i

    def dsrc_data_callback(self, data):
        """ Callback for when new DSRC data arrives. """
        data = self.data_normalize_dsrc(data)
        self.dsrc_data = data
        self._update_combined()

    def radar_data_callback(self, data):
        """ Callback for when new radar data arrives. """
        data = self.data_normalize_radar(data)
        self.radar_data = data
        self._update_combined()

    def data_normalize_radar(self, data):
        """
        Takes in raw radar data and normalizes the format.
        Dictionary that is going to have a bunch of metadata about the
        message, and then a list of entities that the radar data 'sees'

        Create a key in the dict called entities that is a list
        """
        new_data = {}

        # First we handle the status messages
        try:
            # Message 4E0
            new_data['scan_index'] = data['scan_index']
            new_data['vehicle_speed'] = data['vehicle_speed'] / 16
            new_data['yaw_rate'] = self.hex_to_int(data['yaw_rate'], 12) / 16
            new_data['radius_curvature'] = self.hex_to_int(data['radius_curvature'], 14)
            # Message 4E1
            new_data['steering_angle_ack'] = data['steering_angle_ack']
            new_data['radiating'] = data['radiating']
            new_data['maximum_tracks'] = data['maximum_tracks'] # Might not be necessary
            new_data['grouping_mode'] = data['grouping_mode']
            new_data['yaw_rate_bias'] = self.hex_to_int(data['yaw_rate_bias'], 8) / 8
            new_data['speed_comp_factor'] = (data['speed_comp_factor'] / 512) + 0.9375
            # Message 4E3
            new_data['auto_align_angle'] = self.hex_to_int(data['auto_align_angle'], 8) / 16
            # Message 5E8
            new_data['sideslip_angle'] = self.hex_to_int(data['sideslip_angle'], 10) / 8
        except KeyError:
            self.logger.debug("KeyError, printing data structure\n")
            self.logger.debug(json.dumps(data))

        # Now we deal with all of the tracks
        new_data['entities'] = list()
        track_id = 1
        vehicle_speed = new_data['vehicle_speed'] if 'vehicle_speed' in new_data else 0 # m/s
        for i in range(1,65):
            track_number = str(track_id)
            try:
                track_status = data[track_number + "_track_status"]
            except KeyError:
                track_id += 1
                continue
            # If we get here, then this track is valid, so add to the list
            try:
                track = {}
                track["track_number"] = track_number
                track[track_number + "_track_status"] = data[track_number + "_track_status"]
                track[track_number + "_track_range"] = data[track_number + "_track_range"] / 10
                track[track_number + "_track_range_rate"] = self.hex_to_int(data[track_number + "_track_range_rate"], 14) / 100
                track[track_number + "_track_range_accel"] = self.hex_to_int(data[track_number + "_track_range_accel"], 10) / 20
                track[track_number + "_track_angle"] = self.hex_to_int(data[track_number + "_track_angle"], 10) / 10
                track[track_number + "_track_width"] = data[track_number + "_track_width"] / 2
                track[track_number + "_track_oncoming"] = data[track_number + "_track_oncoming"]
                track[track_number + "_track_lat_rate"] = self.hex_to_int(data[track_number + "_track_lat_rate"], 6) / 4
                track[track_number + "_track_moving"] = data[track_number + "_track_moving"]
                track[track_number + "_track_power"] = -10 + data[track_number + "_track_power"]

                # Attempting to get absolute values for speeds
                # Note: Currently only basing this off range_rate and vehicle_speed
                absolute_speed = vehicle_speed # m/s
                angle = self.hex_to_int(track[track_number + "_track_angle"], 10) / 10
                lat_rate = self.hex_to_int(track[track_number + "_track_lat_rate"], 6) / 4
                range_rate = self.hex_to_int(track[track_number + "_track_range_rate"], 14) / 100

                radians = math.radians(angle)
                abs_speed_sin = lat_rate / math.sin(radians) if math.sin(radians) != 0 else lat_rate
                abs_speed_cos = range_rate / math.cos(radians) if math.cos(radians) != 0 else range_rate
                abs_speed_hypot = math.hypot(lat_rate, range_rate)

                track[track_number + "_track_absolute_rate"] = ((abs_speed_sin + abs_speed_cos + abs_speed_hypot) / 3) + absolute_speed
                #print("Angle: " + str(angle))
                #print("Lateral: " + str(lat_rate))
                #print("Rate: " + str(range_rate))
                #print("Speed: " + str(track[track_number + "_track_absolute_rate"]))
                if track[track_number + "_track_power"] > 2:
                    new_data['entities'].append(track)
                track_id += 1;
            except KeyError:
                # Shouldn't happen
                self.logger.debug("KeyError, printing data structure\n")
                self.logger.debug(json.dumps(data))
        #print(new_data['entities'])
        # self.logger.debug("FINISHED RADAR NORMALIZER, HERE IS NORMALIZED DATA\n")
        # self.logger.debug(json.dumps(new_data))

        return new_data

    def data_normalize_dsrc(self, data):
        """ Takes in raw dsrc data and normalizes the format. """
        # self.logger.debug("FINISHED DSRC NORMALIZER, HERE IS NORMALIZED DATA\n")
        # self.logger.debug(json.dumps(data))
        return data

    def _update_combined(self):
        """ Updates the combined model using the latest dsrc & radar data. """

        # Perform a simple union of data currently
        data = {'dsrc': None, 'radar': None}
        if self.dsrc_data:
            data['dsrc'] = self.dsrc_data
        if self.radar_data:
            data['radar'] = self.radar_data

        # Send updated information back to our callback function (Collision Avoidance)
        self.combined_data_queue.put(data)

        # sends logs to the combined file
        logging.getLogger('combined').info(json.dumps(data))

        self.data_count = self.data_count + 1
        if self.data_count > 20:
            if data['dsrc'] and data['radar']:
                msg = {
                    'dsrc': {
                        'speed': data['dsrc']['message']['speed'],
                        'lat': data['dsrc']['message']['lat'],
                        'long': data['dsrc']['message']['long'],
                        'remotes': len(data['dsrc']['remote_messages'])
                    },
                    'radar': {
                        'tracks': len(data['radar']['entities'])
                    }
                }
            else:
                msg = 'Missing data member dsrc or radar'
            self.logger.info(msg)
            self.data_count = 0
