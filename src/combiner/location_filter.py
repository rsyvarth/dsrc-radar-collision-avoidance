import math

RADIUS = 0.5 # meters

def location_filter(data):
    radar_entities = data['radar']['entities']
    result = {}

    # Loop through all the ranges/angles to figure out objects in same vicinity
    length = len(radar_entities)
    items_to_remove = []
    for i, entity in enumerate(radar_entities):
        entity_number = entity['track_number']
        entity_range = entity[entity_number+'_track_range']
        entity_angle = entity[entity_number+'_track_angle']
        range_x1 = entity_range * math.cos(math.radians(entity_angle))
        range_y1 = entity_range * math.sin(math.radians(entity_angle))
        for j in range(i, length):
            if i == j:
                continue
            track = radar_entities[j]
            track_number = track['track_number']
            track_range = track[track_number+'_track_range']
            track_angle = track[track_number+'_track_angle']
            range_x2 = track_range * math.cos(math.radians(track_angle))
            range_y2 = track_range * math.sin(math.radians(track_angle))

            # Calculate the distance between the 2 vectors
            new_x = math.fabs(range_x2 - range_x1)
            new_y = math.fabs(range_y2 - range_y1)

            distance = math.sqrt(math.pow(new_x, 2) + math.pow(new_y, 2))
            if distance <= RADIUS:
                # Points should be combined
                track_power = entity[entity_number+'_track_power']
                entity[entity_number+'_track_power'] = track_power + track[track_number+'_track_power']
                items_to_remove.append(track)

        # Remove the items from the list before moving on
        radar_entities = [x for x in radar_entities if x not in items_to_remove]
        items_to_remove[:] = []

    data['radar']['entities'] = radar_entities
    return data
