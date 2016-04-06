import math

RADIUS = 0.5 # meters

def location_filter(data):
    radar_entities = data['radar']['entities']
    result = {}

    # Loop through all the ranges/angles to figure out objects in same vicinity
    items_to_remove = []
    for i, entity in enumerate(radar_entities):
        entity_number = entity['track_number']
        entity_range = entity[str(entity_number)+'_track_range']
        entity_angle = entity[str(entity_number)+'_track_angle']
        range_x1 = entity_range * math.cos(math.radians(entity_angle))
        range_y1 = entity_range * math.sin(math.radians(entity_angle))
        for j in range(i, len(radar_entities)):
            if i == j:
                continue
            track = radar_entities[j]
            track_number = track['track_number']
            track_range = track[str(track_number)+'_track_range']
            track_angle = track[str(track_number)+'_track_angle']
            range_x2 = track_range * math.cos(math.radians(track_angle))
            range_y2 = track_range * math.sin(math.radians(track_angle))

            # Calculate the distance between the 2 vectors
            new_x = math.fabs(range_x2 - range_x1)
            new_y = math.fabs(range_y2 - range_y1)

            distance = math.sqrt(math.pow(new_x, 2) + math.pow(new_y, 2))
            if distance <= RADIUS:
                # Points should be combined
                entity_power = entity[str(entity_number)+'_track_power'] + 10 # -10 - 21 -> 0 - 31 scale
                track_power = track[str(track_number)+'_track_power'] + 10
                entity[str(entity_number)+'_track_power'] = (entity_power + track_power) - 10
                items_to_remove.append(track)

        # Remove the items from the list before moving on
        radar_entities = [x for x in radar_entities if x not in items_to_remove]
        items_to_remove[:] = []

    data['radar']['entities'] = radar_entities
    return data

def test1():
    # 1 combine
    data = {'radar': {'entities': []}}
    radar_entities = data['radar']['entities']
    t1 = {}
    t1['track_number'] = 1
    t1['1_track_range'] = 15
    t1['1_track_angle'] = 15
    t1['1_track_power'] = -3
    radar_entities.append(t1)
    t2 = {}
    t2['track_number'] = 5
    t2['5_track_range'] = 15
    t2['5_track_angle'] = 14
    t2['5_track_power'] = -7
    radar_entities.append(t2)
    t3 = {}
    t3['track_number'] = 11
    t3['11_track_range'] = 3
    t3['11_track_angle'] = -22
    t3['11_track_power'] = 1
    radar_entities.append(t3)
    t4 = {}
    t4['track_number'] = 16
    t4['16_track_range'] = 31
    t4['16_track_angle'] = 35
    t4['16_track_power'] = -8
    radar_entities.append(t4)
    t5 = {}
    t5['track_number'] = 31
    t5['31_track_range'] = 16
    t5['31_track_angle'] = -20
    t5['31_track_power'] = -6
    radar_entities.append(t5)
    t6 = {}
    t6['track_number'] = 44
    t6['44_track_range'] = 15
    t6['44_track_angle'] = -20
    t6['44_track_power'] = 2
    radar_entities.append(t6)
    t7 = {}
    t7['track_number'] = 56
    t7['56_track_range'] = 43
    t7['56_track_angle'] = 0
    t7['56_track_power'] = -9
    radar_entities.append(t7)
    t8 = {}
    t8['track_number'] = 62
    t8['62_track_range'] = 44
    t8['62_track_angle'] = 1
    t8['62_track_power'] = -8
    radar_entities.append(t8)
    print str(radar_entities)
    data = location_filter(data)
    print str(data['radar']['entities'])

def test2():
    # 0 combines
    data = {'radar': {'entities': []}}
    radar_entities = data['radar']['entities']
    t1 = {}
    t1['track_number'] = 1
    t1['1_track_range'] = 15
    t1['1_track_angle'] = 15
    t1['1_track_power'] = -3
    radar_entities.append(t1)
    t2 = {}
    t2['track_number'] = 5
    t2['5_track_range'] = 15
    t2['5_track_angle'] = 12
    t2['5_track_power'] = -7
    radar_entities.append(t2)
    t3 = {}
    t3['track_number'] = 11
    t3['11_track_range'] = 3
    t3['11_track_angle'] = -22
    t3['11_track_power'] = 1
    radar_entities.append(t3)
    t4 = {}
    t4['track_number'] = 16
    t4['16_track_range'] = 31
    t4['16_track_angle'] = 35
    t4['16_track_power'] = -8
    radar_entities.append(t4)
    t5 = {}
    t5['track_number'] = 31
    t5['31_track_range'] = 16
    t5['31_track_angle'] = -20
    t5['31_track_power'] = -6
    radar_entities.append(t5)
    t6 = {}
    t6['track_number'] = 44
    t6['44_track_range'] = 15
    t6['44_track_angle'] = -20
    t6['44_track_power'] = 2
    radar_entities.append(t6)
    t7 = {}
    t7['track_number'] = 56
    t7['56_track_range'] = 43
    t7['56_track_angle'] = 0
    t7['56_track_power'] = -9
    radar_entities.append(t7)
    t8 = {}
    t8['track_number'] = 62
    t8['62_track_range'] = 44
    t8['62_track_angle'] = 1
    t8['62_track_power'] = -8
    radar_entities.append(t8)
    print str(radar_entities)
    data = location_filter(data)
    print str(data['radar']['entities'])

def test3():
    # decimal angles, combine with object not next to you in list
    data = {'radar': {'entities': []}}
    radar_entities = data['radar']['entities']
    t1 = {}
    t1['track_number'] = 1
    t1['1_track_range'] = 15
    t1['1_track_angle'] = 15
    t1['1_track_power'] = -3
    radar_entities.append(t1)
    t2 = {}
    t2['track_number'] = 5
    t2['5_track_range'] = 15
    t2['5_track_angle'] = 14
    t2['5_track_power'] = -7
    radar_entities.append(t2)
    t3 = {}
    t3['track_number'] = 11
    t3['11_track_range'] = 15
    t3['11_track_angle'] = -20.7
    t3['11_track_power'] = 1
    radar_entities.append(t3)
    t4 = {}
    t4['track_number'] = 16
    t4['16_track_range'] = 31
    t4['16_track_angle'] = 35
    t4['16_track_power'] = -8
    radar_entities.append(t4)
    t5 = {}
    t5['track_number'] = 31
    t5['31_track_range'] = 16
    t5['31_track_angle'] = -20.3
    t5['31_track_power'] = -6
    radar_entities.append(t5)
    t6 = {}
    t6['track_number'] = 44
    t6['44_track_range'] = 15
    t6['44_track_angle'] = -20
    t6['44_track_power'] = 2
    radar_entities.append(t6)
    t7 = {}
    t7['track_number'] = 56
    t7['56_track_range'] = 43
    t7['56_track_angle'] = 0
    t7['56_track_power'] = -9
    radar_entities.append(t7)
    t8 = {}
    t8['track_number'] = 62
    t8['62_track_range'] = 44
    t8['62_track_angle'] = 1
    t8['62_track_power'] = -8
    radar_entities.append(t8)
    print str(radar_entities)
    data = location_filter(data)
    print str(data['radar']['entities'])

if __name__ == "__main__":
    test1()
    test2()
    test3()
