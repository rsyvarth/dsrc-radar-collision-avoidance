
from dsrc.dsrc_dispatcher import DsrcEventDispatcher
from radar.radar_dispatcher import RadarEventDispatcher

class Combiner(object):
    def __init__(self, callback, log_dsrc=True, log_radar=True, dsrc_log_file=None, radar_log_file=None):
        self.dsrc_data = None
        self.radar_data = None

        self.callback = callback
        self.dsrc_event_dispatcher = DsrcEventDispatcher(self.dsrc_data_cb, log=log_dsrc, log_file=dsrc_log_file)
        self.radar_event_dispatcher = RadarEventDispatcher(self.radar_data_cb, log=log_radar, log_file=radar_log_file)

    def start(self):
        self.dsrc_event_dispatcher.start()
        self.radar_event_dispatcher.start()

    def dsrc_data_cb(self, data):
        data = self.data_normalize_dsrc(data)
        self.dsrc_data = data
        self.update_combined()

    def radar_data_cb(self, data):
        data = self.data_normalize_radar(data)
        self.radar_data = data
        self.update_combined()

    def data_normalize_radar(self, data):
        return data

    def data_normalize_dsrc(self, data):
        return data

    def update_combined(self):
        # print "Combiner::new_data()"
        # Do magic data processing here
        data = []
        if self.dsrc_data:
            data = data + self.dsrc_data

        if self.radar_data:
            data = data + self.radar_data

        self.callback(data)
