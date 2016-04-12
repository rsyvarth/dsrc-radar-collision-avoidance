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
import subprocess


def main():
    """ Main application entry point. """
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', dest='visualize_dir', help="Path to directory to load all vizualization info from")
    parser.add_argument('--overwrite', dest='overwrite', default=False, action='store_true', help="Overwrite existing logs parts if found")
    args = parser.parse_args()
    if not args.visualize_dir:
        print "Missing required argument, --visualize"
        exit(-1)

    dsrc_log_file = args.visualize_dir + '/dsrc.log'
    radar_log_file = args.visualize_dir + '/radar.log'
    video_file = args.visualize_dir + '/video.mp4'
    log_config = args.visualize_dir + '/config.json'

    config = parse_config(log_config)

    if 'parts_auto_enabled' in config and config['parts_auto_enabled']:
        cap = cv2.VideoCapture(video_file)
        fps = cap.get(cv2.CAP_PROP_FPS)
        frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        duration = float(frames) / fps
        cap.release()

        print 'Video duration: %s' % duration
        start = 0
        count = 1
        while start < duration:
            config['parts'].append({
                'start': start,
                'end': start + config['parts_auto_interval'],
                'name': 'auto_part_%s' % count
            })
            count = count + 1
            start = start + config['parts_auto_interval']

    print config 

    for index, part in enumerate(config['parts']):
        part_path = args.visualize_dir + '/' + (part['name'] if 'name' in part else 'part_%s' % (index+1))
        print "---------------------------------------"
        print " Writing log to %s" % part_path
        print "---------------------------------------"
        if not args.overwrite and os.path.exists(part_path):
            print "Log already exists, skipping..."
            continue

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

    codec = cv2.VideoWriter_fourcc('X','V','I','D')
    out = cv2.VideoWriter(part_path + '/video.avi', codec, fps, (int(video_width), int(video_height)))

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

    # Hack to convert the avi we just outputted into an mp4
    dir = part_path + '/video'
    command = "avconv -y -i %s.avi -c:v libx264 -c:a copy %s.mp4" % (dir, dir)
    subprocess.call(command.split())
    os.remove(part_path + '/video.avi')

def parse_config(config_file):
    with open(config_file, 'r') as content_file:
        content = content_file.read()
        data = json.loads(content)

        if 'video_start' in data:
            data['video_start'] = datetime.datetime.strptime(data['video_start'], '%Y-%m-%d %H:%M:%S.%f')

        return data

if __name__ == "__main__":
    main()
