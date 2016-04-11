import copy

def test_one():
    data = dict()
    data['radar'] = dict()
    data['radar']['entities'] = list()
    test_data = dict()
    track_one = dict()
    track_two = dict()
    track_three = dict()
    track_one["track_number"] = 1
    track_two["track_number"] = 2
    track_three["track_number"] = 3
    data['radar']['entities'].append(track_one)
    data = kalman_filter(data)
    print "Test one <1> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()
    data['radar']['entities'].append(track_one)
    data['radar']['entities'].append(track_two)
    data = kalman_filter(data)
    print "Test one <2> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()
    data['radar']['entities'].append(track_one)
    data['radar']['entities'].append(track_two)
    data['radar']['entities'].append(track_three)
    data = kalman_filter(data)
    print "Test one <3> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()

def test_two():
    data = dict()
    data['radar'] = dict()
    data['radar']['entities'] = list()
    test_data = dict()
    track_one = dict()
    track_two = dict()
    track_three = dict()
    track_one["track_number"] = 3
    track_two["track_number"] = 2
    track_three["track_number"] = 1
    data['radar']['entities'].append(track_one)
    data = kalman_filter(data)
    print "Test two <1> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()
    data['radar']['entities'].append(track_one)
    data['radar']['entities'].append(track_two)
    data = kalman_filter(data)
    print "Test two <2> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()
    data['radar']['entities'].append(track_one)
    data['radar']['entities'].append(track_two)
    data['radar']['entities'].append(track_three)
    data = kalman_filter(data)
    print "Test two <3> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()

def test_three():
    data = dict()
    data['radar'] = dict()
    data['radar']['entities'] = list()
    test_data = dict()
    track_one = dict()
    track_two = dict()
    track_three = dict()
    track_one["track_number"] = 1
    track_two["track_number"] = 1
    track_three["track_number"] = 4
    data['radar']['entities'].append(track_one)
    data = kalman_filter(data)
    print "Test three <1> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()
    #data['radar']['entities'].append(track_one)
    #data['radar']['entities'].append(track_two)
    data = kalman_filter(data)
    print "Test three <2> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()
    #data['radar']['entities'].append(track_one)
    #data['radar']['entities'].append(track_two)
    data['radar']['entities'].append(track_three)
    data = kalman_filter(data)
    print "Test three <3> " + str(data['radar']['entities'])
    data['radar']['entities'] = list()


def main():
    test_one()
    test_two()
    test_three()

#15/30 looks pretty chill
#SET_TOTAL = 30
#SETS_IN = 15
#SET_TOTAL = 20
#SETS_IN = 10
SET_TOTAL = 24
SETS_IN = 12

#list of dictionaries
kalman_old_data = [{} for k in range(SET_TOTAL)]

#currently this function is only set up for radar noise
def kalman_filter(data):
    global kalman_old_data
    unfiltered_data = dict()
    #filtered_data = {}, just going to modify data
    new_entities_list = list()
    new_entities_hash = dict()
    appended_list = list()

    for ent in data['radar']['entities']:
        track_number = ent['track_number']
        unfiltered_data[track_number] = ent


    kalman_old_data = [unfiltered_data] + kalman_old_data[:-1]
    #print "here is kalman_old_data " + str(kalman_old_data)

    for i in range(0,SET_TOTAL):
        appended_list = appended_list + list(kalman_old_data[i].values())

    #print "here is appended_list : " + str(appended_list)
    for ent in appended_list:
        count = 0
        #doing a deep copy so i dont store the averaged data in kalman_old_data
        track_number = ent['track_number']
        new_ent = copy.deepcopy(ent)
        new_ent[track_number + '_track_range'] = 0
        new_ent[track_number + '_track_angle'] = 0
        for i in range(0,SET_TOTAL):
            if track_number in kalman_old_data[i]:
                count += 1
                new_ent[track_number + '_track_range'] += ent[track_number + '_track_range']
                new_ent[track_number + '_track_angle'] += ent[track_number + '_track_angle']
                #new_ent[track_number + '_track_power'] += ent[track_number + '_track_power']
                #print "incrementing count to: " + str(count)

        if count >= SETS_IN and (not (track_number in new_entities_hash)):
            new_ent[track_number + '_track_range'] /= count
            new_ent[track_number + '_track_angle'] /= count
            #new_ent[track_number + '_track_power'] /= count
            new_entities_list.append(new_ent)
            new_entities_hash[track_number] = new_ent

    #construct a hash of unfilitered data, makes it easier to test for existence
    #in next function call


    data['radar']['entities'] = new_entities_list
    #kalman_old_data[1] = kalman_old_data[0]
    #kalman_old_data[0] = unfiltered_data
    return data


if __name__ == "__main__":
    main()


