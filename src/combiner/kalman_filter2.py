import copy

SET_TOTAL = 100
WEIGHT_NEW = 0.25

#copy.deepcopy(ent)
def empty_data_factory():
    return {
        'range': [None for k in range(SET_TOTAL)],
        'power': [None for k in range(SET_TOTAL)],
        'angle': [None for k in range(SET_TOTAL)],
        'last_entity_returned': None,
        'probability_correct': 0.0
    }    

historical_data = [empty_data_factory() for i in range(64)]

#currently this function is only set up for radar noise
def kalman_filter2(data):
    global historical_data

    current_entities_by_track_number = dict((e['track_number'], e) for e in data['radar']['entities'])

    new_entities = []

    for track_number in range(64):
        tn_str = str(track_number)
        
        # Update the historical data 
        hist_data_entry = historical_data[track_number]

        if tn_str in current_entities_by_track_number:
            entity = current_entities_by_track_number[tn_str]

            entity[tn_str + '_track_range'] = calc_decaying_avg(hist_data_entry['range'], entity[tn_str + '_track_range'])
            entity[tn_str + '_track_power'] = calc_decaying_avg(hist_data_entry['power'], entity[tn_str + '_track_power'])
            entity[tn_str + '_track_angle'] = calc_decaying_avg(hist_data_entry['angle'], entity[tn_str + '_track_angle'])
            
            hist_data_entry['probability_correct'] = (hist_data_entry['probability_correct'] * (1.0-WEIGHT_NEW)) + (1.0 * WEIGHT_NEW)

            hist_data_entry['last_entity_returned'] = copy.deepcopy(entity)
        else:
            last_entity = hist_data_entry['last_entity_returned']
            if last_entity:
                last_entity[tn_str + '_track_range'] = calc_decaying_avg(hist_data_entry['range'], None)
                last_entity[tn_str + '_track_power'] = calc_decaying_avg(hist_data_entry['power'], None)
                last_entity[tn_str + '_track_angle'] = calc_decaying_avg(hist_data_entry['angle'], None)

            hist_data_entry['probability_correct'] = (hist_data_entry['probability_correct'] * (1.0-WEIGHT_NEW)) + (0.0 * WEIGHT_NEW)
            
        if hist_data_entry['probability_correct'] > 0.5:
            new_entities.append(hist_data_entry['last_entity_returned'])

    # Update our current data off of the historical
    data['radar']['entities'] = new_entities

    return data

# weights = []
# for i in range(SET_TOTAL):
#     weights.append(10.0 * (i+1))
# total_parts = sum(weights)

weights = []
for i in range(SET_TOTAL):
    weights.append(1.5**i)
total_parts = sum(weights)

def calc_decaying_avg(elems, new_elem):
    elems.pop(0)
    elems.append(new_elem)

    val = 0.0
    parts_for_next_elem = 0;
    last_elem = 0

    for i, elem in enumerate(elems):
        parts_for_next_elem = parts_for_next_elem + weights[i]

        if elem is None:
            continue

        last_elem = elem
        val = val + (elem * parts_for_next_elem)/total_parts
        parts_for_next_elem = 0

    val = val + (last_elem * parts_for_next_elem)/total_parts

    return val

