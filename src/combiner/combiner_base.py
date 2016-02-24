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

    def __init__(self, callback, log_dsrc=True, log_radar=True, dsrc_log_file=None, radar_log_file=None):
        """ Setup Combiner, initialize DSRC+Radar event dispatcher. """
        self.dsrc_data = None
        self.radar_data = None

        self.data_queue = Queue()

        self.callback = callback
        #self.dsrc_event_dispatcher = DsrcEventDispatcher(self.data_queue, log=log_dsrc, log_file=dsrc_log_file)
        self.radar_event_dispatcher = RadarEventDispatcher(self.data_queue, log=log_radar, log_file=radar_log_file)

        self.logger = logging.getLogger('combined')

    def start(self):
        """ Start running the event dispatcher threads (we are ready to recieve data). """
        #self.dsrc_event_dispatcher.start()
        self.radar_event_dispatcher.start()

        while self.dsrc_event_dispatcher.is_alive() and self.radar_event_dispatcher.is_alive():
            if self.data_queue.qsize() > 1:
                print 'WARNING: The queue depth is %s, we are behind real time!' % self.data_queue.qsize()

            try:
                dispatcher_data = self.data_queue.get(timeout=0.5)

                if dispatcher_data['from'] == 'dsrc':
                    self.dsrc_data_callback(dispatcher_data['data'])
                else:
                    print("Test")
                    self.radar_data_callback(dispatcher_data['data'])
            except Empty:
                print 'Timeout'
                # pass

        self.dsrc_event_dispatcher.terminate()
        self.radar_event_dispatcher.terminate()

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
        new_data['entities'] = list()
        track_id = 1
        for i in range(1,65):
            track_number = str(track_id)
            if (data[track_number + "_track_status"] == 0):
                track_id += 1
                continue
            track = {}
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

        print(new_data['entities'])

        return data

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

        # sends logs to the combined file
        self.logger.debug(str(
                    json.dumps(data,
                    separators=(',',':')))
                )

        # Send updated information back to our callback function (Collision Avoidance)
        self.callback(data)
