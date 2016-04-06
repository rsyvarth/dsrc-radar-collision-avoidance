


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


#list of dictionaries
kalman_old_data = list()
kalman_old_data.append(dict())
kalman_old_data.append(dict())

#currently this function is only set up for radar noise
def kalman_filter(data):
    unfiltered_data = dict()
    #filtered_data = {}, just going to modify data
    new_entities_list = list()
    new_entities_hash = dict()


    #only need to iterate over one two of the lists
    appended_list = data['radar']['entities'] + list(kalman_old_data[0].values())
    for ent in appended_list :
        count = 1
        track_number = ent['track_number']
        #unfiltered_data[track_number] = ent;
        if track_number in kalman_old_data[0]:
            count += 1
        if track_number in kalman_old_data[1]:
            count += 1
        if count >= 2 and (not (track_number in new_entities_hash)):
            new_entities_list.append(ent)
            new_entities_hash[track_number] = 1

    #construct a hash of unfilitered data, makes it easier to test for existence
    #in next function call
    for ent in data['radar']['entities']:
        track_number = ent['track_number']
        unfiltered_data[track_number] = ent

    data['radar']['entities'] = new_entities_list
    kalman_old_data[1] = kalman_old_data[0]
    kalman_old_data[0] = unfiltered_data
    return data


if __name__ == "__main__":
    main()


