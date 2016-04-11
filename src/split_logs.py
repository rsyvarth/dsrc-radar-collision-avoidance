"""Log Splitter

Takes a path to a set of log files + videos and splits it into multiple parts
that can more easily used / analyzed.
"""
import time
import argparse
import datetime
import os
import json
import cv2

def main():
    """ Main application entry point. """
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', dest='visualize_dir', help="Path to directory to load all vizualization info from")
    args = parser.parse_args()
    if not args.visualize_dir:
        print "Missing required argument, --visualize"
        exit(-1)

    dsrc_log_file = args.visualize_dir + '/dsrc.log'
    radar_log_file = args.visualize_dir + '/radar.log'
    video_file = args.visualize_dir + '/video.mp4'
    log_config = args.visualize_dir + '/config.json'

    config = parse_config(log_config)
    print config

    for index, part in enumerate(config['parts']):
        part_path = args.visualize_dir + '/' + (part['name'] if 'name' in part else 'part_%s' % index)
        if not os.path.exists(part_path):
            os.makedirs(part_path)

        export_part_video(part, part_path, video_file)
        export_part_log(part, part_path + '/radar.log', radar_log_file, config['video_start'])
        export_part_log(part, part_path + '/dsrc.log', dsrc_log_file, config['video_start'])
        export_part_config(part_path + '/config.json', config)

def export_part_config(path, config):
    new_config = open(path, 'wt')
    new_config.write('{}')
    new_config.close()

def export_part_log(part, part_log_path, original_log_path, video_start):
    original_log = open(original_log_path, 'r')
    new_log = open(part_log_path, 'wt')

    for line in original_log:
        line_split = line.split('; ')
        date_str = line_split[0]
        date_obj = datetime.datetime.strptime(date_str, '%Y-%m-%d %H:%M:%S.%f')

        if (date_obj - video_start).total_seconds() < part['start']:
            continue

        if (date_obj - video_start).total_seconds() > part['end']:
            break

        new_log.write(line)

    original_log.close()
    new_log.close()

def export_part_video(part, part_path, video_file):
    cap = cv2.VideoCapture(video_file)

    fps = cap.get(cv2.CAP_PROP_FPS)
    video_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    video_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    out = cv2.VideoWriter(part_path + '/video.mp4', -1, fps, (int(video_width), int(video_height)))

    while cap.get(cv2.CAP_PROP_POS_MSEC) < part['start'] * 1000:
        cap.grab() # skip frames

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if cap.get(cv2.CAP_PROP_POS_MSEC) > part['end'] * 1000:
            break

        # write the frame
        out.write(frame)

    # Release everything if job is finished
    cap.release()
    out.release()

def parse_config(config_file):
    with open(config_file, 'r') as content_file:
        content = content_file.read()
        data = json.loads(content)

        if 'video_start' in data:
            data['video_start'] = datetime.datetime.strptime(data['video_start'], '%Y-%m-%d %H:%M:%S.%f')

        return data

if __name__ == "__main__":
    main()
