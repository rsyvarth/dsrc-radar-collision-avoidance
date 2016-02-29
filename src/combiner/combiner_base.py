from dsrc.dsrc_dispatcher import DsrcEventDispatcher
from radar.radar_dispatcher import RadarEventDispatcher
import json
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

    def __init__(self, callback, log_dsrc=True, log_radar=True, dsrc_log_file=None, radar_log_file=None, dsrc_enabled=True, radar_enabled=True):
        """ Setup Combiner, initialize DSRC+Radar event dispatcher. """
        self.dsrc_data = None
        self.radar_data = None

        self.dsrc_enabled = dsrc_enabled
        self.radar_enabled = radar_enabled

        self.data_queue = Queue()
        self.callback = callback

        if self.dsrc_enabled:
            self.dsrc_event_dispatcher = DsrcEventDispatcher(self.data_queue, log=log_dsrc, log_file=dsrc_log_file)
        if self.radar_enabled:
            self.radar_event_dispatcher = RadarEventDispatcher(self.data_queue, log=log_radar, log_file=radar_log_file)

        self.logger = logging.getLogger('combined')

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
        print(data)

        # First we handle the status messages
        try:
            # Message 4E0
            new_data['scan_index'] = data['scan_index']
            new_data['vehicle_speed'] = data['vehicle_speed']
            new_data['yaw_rate'] = data['yaw_rate']
            new_data['radius_curvature'] = data['radius_curvature']
            # Message 4E1
            new_data['steering_angle_ack'] = data['steering_angle_ack']
            new_data['radiating'] = data['radiating']
            new_data['maximum_tracks'] = data['maximum_tracks'] # Might not be necessary
            new_data['grouping_mode'] = data['grouping_mode']
            new_data['yaw_rate_bias'] = data['yaw_rate_bias']
            new_data['speed_comp_factor'] = data['speed_comp_factor']
            # Message 4E3
            new_data['auto_align_angle'] = data['auto_align_angle']
            # Message 5E8
            new_data['sideslip_angle'] = data['sideslip_angle']
        except KeyError:
            print("KeyError, printing data structure:\n")
            print(data)

        # Now we deal with all of the tracks
        new_data['entities'] = list()
        track_id = 1
        for i in range(1,65):
            track_number = str(track_id)
            try:
                if (data[track_number + "_track_status"] == 0):
                    track_id += 1
                    continue
                track = {}
                track[track_number + "_track_status"] = data[track_number + "_track_status"]
                track[track_number + "_track_range"] = data[track_number + "_track_range"]
                track[track_number + "_track_range_rate"] = data[track_number + "_track_range_rate"]
                track[track_number + "_track_range_accel"] = data[track_number + "_track_range_accel"]
                track[track_number + "_track_angle"] = data[track_number + "_track_angle"]
                track[track_number + "_track_width"] = data[track_number + "_track_width"]
                track[track_number + "_track_oncoming"] = data[track_number + "_track_oncoming"]
                track[track_number + "_track_lat_rate"] = data[track_number + "_track_lat_rate"]
                track[track_number + "_track_moving"] = data[track_number + "_track_moving"]
                track[track_number + "_track_power"] = data[track_number + "_track_power"]
                new_data['entities'].append(track)
                track_id += 1;
            except KeyError:
                print("Keyerror on key: " + str(track_number))
                track_id += 1

        print(new_data['entities'])

        return new_data

    def data_normalize_dsrc(self, data):
        """ Takes in raw dsrc data and normalizes the format. """
        return [data]

    def _update_combined(self):
        """ Updates the combined model using the latest dsrc & radar data. """

        # Perform a simple union of data currently
        data = []
        if self.dsrc_data:
            data = data + self.dsrc_data
        if self.radar_data:
            data = data + self.radar_data

        # Send updated information back to our callback function (Collision Avoidance)
        self.callback(data)

        # sends logs to the combined file
        self.logger.debug(json.dumps(data))
