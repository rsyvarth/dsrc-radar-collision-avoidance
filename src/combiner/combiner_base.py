from dsrc.dsrc_dispatcher import DsrcEventDispatcher
from radar.radar_dispatcher import RadarEventDispatcher

class Combiner(object):
    """Combiner - takes DSRC+Radar information and forms a single model of the environment

    This package is responsible for taking data from each source, normalizing the
    format, and combining that information into a single model of the car's
    environment. Once the combination is complete the new model gets passed to
    the registered callback function, which in our case feeds data into our collision
    avoidance system.
    """

    def __init__(self, callback, log_dsrc=True, log_radar=True, dsrc_log_file=None, radar_log_file=None):
        """Setup Combiner, initialize DSRC+Radar event dispatcher"""
        self.dsrc_data = None
        self.radar_data = None

        self.callback = callback
        self.dsrc_event_dispatcher = DsrcEventDispatcher(self.dsrc_data_cb, log=log_dsrc, log_file=dsrc_log_file)
        self.radar_event_dispatcher = RadarEventDispatcher(self.radar_data_cb, log=log_radar, log_file=radar_log_file)

    def start(self):
        """Start running the event dispatcher threads (we are ready to recieve data)"""
        self.dsrc_event_dispatcher.start()
        self.radar_event_dispatcher.start()

    def dsrc_data_cb(self, data):
        """Callback for when new DSRC data arrives"""
        data = self.data_normalize_dsrc(data)
        self.dsrc_data = data
        self.update_combined()

    def radar_data_cb(self, data):
        """Callback for when new radar data arrives"""
        data = self.data_normalize_radar(data)
        self.radar_data = data
        self.update_combined()

    def data_normalize_dsrc(self, data):
        """Takes in raw dsrc data and normalizes the format"""
        return data

    def data_normalize_radar(self, data):
        """Takes in raw radar data and normalizes the format"""
        return data

    def update_combined(self):
        """Updates the combined model using the latest dsrc & radar data"""

        # Perform a simple union of data currently
        data = []
        if self.dsrc_data:
            data = data + self.dsrc_data
        if self.radar_data:
            data = data + self.radar_data

        # Send updated information back to our callback function (Collision Avoidance)
        self.callback(data)
