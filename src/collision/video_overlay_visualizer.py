import math
import numpy as np
try:
    import cv2
except ImportError:
    pass

IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3

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

        for track in track_objects:
            self.draw_box_for_track(track, img)

        cv2.imshow("Data Visualizer", img)
        return True

    def draw_box_for_track(self, track, img):
        # HACK FOR A VER OFFSET FOR NOW
        vertical_offset = 70

        # Step 1
        track_number = track["track_number"]
        #track_width = track[track_number+"_track_width"]
        track_width = 0.2
        track_range = track[track_number+"_track_range"]
        track_angle = track[track_number+"_track_angle"]

        # Step 2
        img_height, img_width = img.shape[:2]

        # Step 3
        # Formula using: obj_width(pixels) = (focal length(mm) * obj width(mm) * img_width(pixels)) / (track_range(mm) * sensor width(mm)?)
        obj_range, obj_angle = self.convert_radar_to_camera(track_range, track_angle, self.distance_behind_radar, self.distance_beside_radar, self.camera_angle)
        #print "obj_range: " + str(obj_range) + ", track_range: "+str(track_range)
        pixel_width = ((self.focal_length) * (track_width) *
img_width) / ((obj_range) * (self.sensor_size))
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
        cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 3)
        #print "Top left: " + str(top_left)
        #print "Bottom right: " + str(bottom_right)

    def convert_radar_to_camera(self, track_range, track_angle, distance_behind_radar, distance_beside_radar, camera_angle):
            triangle_opposite = distance_behind_radar + (track_range * math.cos(math.radians(track_angle)))
            triangle_adj = distance_beside_radar + (track_range * math.sin(math.radians(track_angle)))
            obj_range = math.sqrt(math.pow(triangle_opposite, 2) + math.pow(triangle_adj, 2))
            #obj_angle = math.atan(triangle_opposite / triangle_adj)
            obj_angle = math.acos(triangle_adj / obj_range)
            obj_angle = math.degrees(obj_angle) + camera_angle
            obj_angle = obj_angle - 90
            return obj_range, obj_angle
