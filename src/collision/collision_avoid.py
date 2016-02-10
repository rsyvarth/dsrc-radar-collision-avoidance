import logging

class CollisionAvoidance(object):
    """Collision Avoidance - takes combined data and displays predicted collisions

    This package is included mainly for the purpose of demonstration.
    It takes the information provided by our Combiner and calculates any predicted
    collisions displaying warnings in a simple UI.
    """

    def __init__(self):
        """Setup the CA class, just empty state for now"""
        self.current_state = None

    def new_data_handler(self, data):
        """Called whenever new data arrives from the Combiner"""
        logging.info("Collision_avoid::new_data_handler() %s" % data)
        self.current_state = data

        self.analyze_state()

    def analyze_state(self):
        """Eventually this will do cool math to detect collisions"""
        pass
