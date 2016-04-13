import math
import numpy as np
try:
    import cv2
except ImportError:
    pass

IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3
COMBINED_RADIUS = 0.5
COMBINED_ANGLE = 0.5

class VideoOverlayVisualizer(object):
    def __init__(self, videofile, distance_behind_radar, distance_beside_radar, camera_angle, camera_field_of_view, focal_length, sensor_size):
        """
        videopath: path to video that we will be using for image processing
        distance_behind_radar: how far the camera is behind the radar in the vehicle (m)
        distance_beside_radar: how far the camera is to the side of the radar in the vehicle (m)
        camera_angle: angle of the camera relative to the radar (currently not used; degrees)
        camera_field_of_view: the angular extent of the scene imaged by your camera (degrees); how many degrees can your camera see
        focal_length: focal length of the camera while filming (mm; note that we do not account for focal length changes mid_video)
        sensor_size: size of the camera sensor (mm; believe we want the height)
        """
        self.camera = cv2.VideoCapture(videofile)
        self.fps = self.camera.get(cv2.CAP_PROP_FPS)
        self.video_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.video_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.distance_behind_radar = distance_behind_radar
        self.distance_beside_radar = distance_beside_radar
        self.camera_angle = camera_angle
        self.camera_field_of_view = camera_field_of_view
        self.focal_length = focal_length
        self.sensor_size = sensor_size
        img = np.zeros((IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS), np.uint8)
        cv2.imshow("Data Visualizer", img)

        self.prev_gps = None
        self.latest_dsrc_box = None

    def update(self, current_state):
        """
        To draw on the object:
        1. Get track info from track_objects: track_width, track_range, track_angle?
        2. Get info for the image: total image size (pixels)
        3. Calculate where in the image the object would be: camera vs. radar position, object sizes in terms of pixels at different positions, anything else?
        4. Based on projected size (height), calculate the middle point
        5. Calculate top left and bottom right corners of the track object
        6. Draw the appropriate rectangle
        """
        # Loop until the video is done
        ret, img = self.camera.read()

        if not ret:
            # We have reached the end of the video
            self.camera.release()
            cv2.destroyAllWindows()
            return False

        track_objects = []
        if (current_state and current_state["radar"]):
            track_objects = current_state["radar"]["entities"]

        if (current_state and current_state["dsrc"] and len(current_state["dsrc"]["remote_messages"])):
            remote = current_state["dsrc"]["remote_messages"][0]
            local = current_state["dsrc"]["message"]

            if not self.prev_gps:
                self.prev_gps = (local['long'], local['lat'])

            dist_from_prev = self.calc_gps_distance((local['long'], local['lat']), self.prev_gps)
            if dist_from_prev > 0.2 and local['long'] != self.prev_gps[0] and remote['long'] != local['long']:
                local_bearing = math.atan((local['lat'] - self.prev_gps[1])/(local['long'] - self.prev_gps[0]))
                if local['long'] - self.prev_gps[0] < 0:
                    local_bearing = local_bearing + math.pi/2

                bearing_relative_remote = math.atan((remote['lat'] - local['lat'])/(remote['long'] - local['long']))
                if remote['long'] - local['long'] < 0:
                    bearing_relative_remote = bearing_relative_remote + math.pi/2

                angle = math.degrees(bearing_relative_remote - local_bearing)
                dist = self.calc_gps_distance((local['long'], local['lat']), (remote['long'], remote['lat']))

                self.latest_dsrc_box = (dist - 4, angle)

            if self.latest_dsrc_box:
                self.draw_box_for_obj(img, 0.2, self.latest_dsrc_box[0], self.latest_dsrc_box[1], color = (255, 0, 0))

            # Update prev_gps for next round if necessary
            dist_from_prev = self.calc_gps_distance((local['long'], local['lat']), self.prev_gps)
            if dist_from_prev > 5.0:
                self.prev_gps = (local['long'], local['lat'])

        for track in track_objects:
            track_number = track["track_number"]
            #track_width = track[track_number+"_track_width"]
            track_width = 0.2
            track_range = track[track_number+"_track_range"]
            track_angle = track[track_number+"_track_angle"]
            if self.calc_distance((track_range, track_angle), self.latest_dsrc_box) < COMBINED_RADIUS:
                self.draw_box_for_obj(img, track_width, track_range, track_angle, \
                    (0,0,255))
            else:
                self.draw_box_for_obj(img, track_width, track_range, track_angle)

        cv2.imshow("Data Visualizer", img)
        return True

    def draw_box_for_obj(self, img, obj_width, obj_range, obj_angle, color = (0, 255, 0)):
        # HACK FOR A VER OFFSET FOR NOW
        vertical_offset = 70

        # Step 2
        img_height, img_width = img.shape[:2]

        # Step 3
        # Formula using: obj_width(pixels) = (focal length(mm) * obj width(mm) * img_width(pixels)) / (obj_range(mm) * sensor width(mm)?)
        obj_range, obj_angle = self.convert_obj_to_camera(obj_range, obj_angle, self.distance_behind_radar, self.distance_beside_radar, self.camera_angle)
        #print "obj_range: " + str(obj_range) + ", obj_range: "+str(obj_range)
        pixel_width = ((self.focal_length) * (obj_width) * img_width) / ((obj_range) * (self.sensor_size))
        #print "Pixel width: " + str((pixel_width, self.focal_length, img_width, obj_range, self.sensor_size))

        # Step 4
        img_midpoint = (img_width / 2) + ((obj_angle / self.camera_field_of_view) * img_width)
        #print "img_midpoint: " + str(img_midpoint) + " obj_angle: "+str(obj_angle)

        # Step 5
        # For now, drawing squares and drawing simply on the middle of the image
        # Top left corner of image is point (0, 0)
        img_left_side = img_midpoint - (pixel_width / 2)
        img_right_side = img_midpoint + (pixel_width / 2)
        img_top = (img_height / 2) - (pixel_width / 2)
        img_bottom = (img_height / 2) + (pixel_width / 2)
        # TODO: Add bounds checks
        top_left = (int(img_left_side), int(img_top + vertical_offset))
        bottom_right = (int(img_right_side), int(img_bottom + vertical_offset))

        # Step 6
        cv2.rectangle(img, top_left, bottom_right, color, 3)
        #print "Top left: " + str(top_left)
        #print "Bottom right: " + str(bottom_right)

    def convert_obj_to_camera(self, obj_range, obj_angle, distance_behind_radar, distance_beside_radar, camera_angle):
        triangle_opposite = distance_behind_radar + (obj_range * math.cos(math.radians(obj_angle)))
        triangle_adj = distance_beside_radar + (obj_range * math.sin(math.radians(obj_angle)))
        obj_range = math.sqrt(math.pow(triangle_opposite, 2) + math.pow(triangle_adj, 2))
        #obj_angle = math.atan(triangle_opposite / triangle_adj)
        obj_angle = math.acos(triangle_adj / obj_range)
        obj_angle = math.degrees(obj_angle) + camera_angle
        obj_angle = obj_angle - 90
        return obj_range, obj_angle

    def calc_gps_distance(self, new, old):
        lon1, lat1 = new
        lon2, lat2 = old

        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371 # Radius of earth in kilometers. Use 3956 for miles
        return c * r * 1000 # conv to meters

    def calc_distance(self, tuple_distance_angle_one, tuple_distance_angle_two):
        x1 = tuple_distance_angle_one[0] * math.cos(math.radians(tuple_distance_angle_one[1]))
        y1 = tuple_distance_angle_one[0] * math.sin(math.radians(tuple_distance_angle_one[1]))
        x2 = tuple_distance_angle_two[0] * math.cos(math.radians(tuple_distance_angle_two[1]))
        y2 = tuple_distance_angle_two[0] * math.sin(math.radians(tuple_distance_angle_two[1]))
        # Calculate the distance between the 2 vectors
        new_x = math.fabs(x2 - x1)
        new_y = math.fabs(y2 - y1)
        return math.sqrt(math.pow(new_x, 2) + math.pow(new_y, 2))
