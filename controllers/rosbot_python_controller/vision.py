#vision.py - Handles all camera processing for the Rosbot Python Controller


import cv2
import numpy as np
from constants import *

class VisionProcessor:
    @staticmethod
    def get_hsv_image(camera):
        """Converts the Webots camera buffer into an OpenCV HSV image."""
        image_data = camera.getImage()
        if not image_data:
            return None
        
        width = camera.getWidth()
        height = camera.getHeight()
        
        # Convert Webots raw BGRA buffer into a usable Numpy array
        image = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
        
        # Convert to BGR (standard OpenCV), then to HSV (best for color filtering)
        frame = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return hsv

    @staticmethod
    def is_red_wall_ahead(camera):
        """Returns True if a red floating wall is blocking the camera view."""
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None:
            return False

        # Create masks to isolate only the RED pixels
        mask1 = cv2.inRange(hsv, np.array(RED_LOWER1), np.array(RED_UPPER1))
        mask2 = cv2.inRange(hsv, np.array(RED_LOWER2), np.array(RED_UPPER2))
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Calculate what percentage of the image is red
        height, width = red_mask.shape
        total_pixels = width * height
        red_pixels = cv2.countNonZero(red_mask)
        
        ratio = red_pixels / total_pixels
        return ratio > RED_PIXEL_THRESHOLD

    @staticmethod
    def is_green_ground_ahead(camera):
        """Returns True if the floor directly in front of the robot is green."""
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None:
            return False

        height, width, _ = hsv.shape
        # Crop the image to ONLY look at the bottom 50% (the floor)
        bottom_half = hsv[height // 2:, :]

        # Mask out the green pixels
        green_mask = cv2.inRange(bottom_half, np.array(GREEN_LOWER), np.array(GREEN_UPPER))

        # Calculate percentage of green on the floor
        green_pixels = cv2.countNonZero(green_mask)
        total_pixels = (height // 2) * width
        
        ratio = green_pixels / total_pixels
        return ratio > GREEN_PIXEL_THRESHOLD
    
    @staticmethod
    def get_green_ground_blobs(camera):
        """
        Detect ALL green floor regions and, for each one, return MANY sampled green
        pixels spread across the blob (multi-point sampling). The caller projects every
        sample through the depth camera so the patch is mapped at its true shape/extent
        — not approximated by a single centroid square.

        Returns a list of dicts, one per significant blob:
            {'u':       centroid_x  (px),
             'v':       centroid_y  (px),
             'area':    pixel_area,
             'samples': [(u, v), ...]  green pixels within the blob, strided}
        Blobs smaller than GREEN_MIN_BLOB_PIXELS are discarded as noise/specks.
        Returns [] when no green is visible.
        """
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None:
            return []

        green_mask = cv2.inRange(hsv, np.array(GREEN_LOWER), np.array(GREEN_UPPER))

        # Connected-components groups the green pixels into separate physical patches
        # and labels each pixel with its blob id, so we can sample within one patch only.
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(green_mask, connectivity=8)

        blobs = []
        for i in range(1, num_labels):  # 0 is the background
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < GREEN_MIN_BLOB_PIXELS:
                continue

            # All pixels belonging to THIS blob, subsampled by the stride.
            ys, xs = np.where(labels == i)
            sel = np.arange(0, len(xs), GREEN_SAMPLE_STRIDE)
            samples = list(zip(xs[sel].tolist(), ys[sel].tolist()))

            blobs.append({
                'u':       int(centroids[i][0]),
                'v':       int(centroids[i][1]),
                'area':    area,
                'samples': samples,
            })
        return blobs

    @staticmethod
    def detect_pillars(camera):
        """Returns 'blue', 'yellow', or None depending on what the robot sees."""
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None:
            return None

        # Check for Blue
        blue_mask = cv2.inRange(hsv, np.array(BLUE_LOWER), np.array(BLUE_UPPER))
        if cv2.countNonZero(blue_mask) > MIN_PILLAR_PIXELS:
            return 'blue'
            
        # Check for Yellow
        yellow_mask = cv2.inRange(hsv, np.array(YELLOW_LOWER), np.array(YELLOW_UPPER))
        if cv2.countNonZero(yellow_mask) > MIN_PILLAR_PIXELS:
            return 'yellow'

        return None
    
    @staticmethod
    def get_pillar_offset(camera, color):
        """Returns how far off-center the pillar is (in pixels). Returns None if not seen."""
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None: return None
        
        if color == 'blue':
            mask = cv2.inRange(hsv, np.array(BLUE_LOWER), np.array(BLUE_UPPER))
        elif color == 'yellow':
            mask = cv2.inRange(hsv, np.array(YELLOW_LOWER), np.array(YELLOW_UPPER))
        else:
            return None
            
        # Find the coordinates of all colored pixels
        pixels = cv2.findNonZero(mask)
        if pixels is None or len(pixels) < MIN_PILLAR_PIXELS:
            return None
            
        # Find the center X coordinate of the blob
        center_x = int(np.mean(pixels[:, 0, 0]))
        
        # Calculate how far it is from the center of the camera (width // 2)
        width = camera.getWidth()
        offset = center_x - (width // 2)
        return offset
    
    @staticmethod
    def get_wood_blobs(camera):
        """
        Like get_green_ground_blobs but for the WOOD-coloured planks/walls. Returns
        multi-point samples per blob so a FLOATING wall's full visible area can be mapped
        solidly (the caller depth-projects + height-filters each sample).
        Returns [] when no wood is visible.
        """
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None:
            return []
        mask = cv2.inRange(hsv, np.array(WOOD_LOWER), np.array(WOOD_UPPER))
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        blobs = []
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < FLOATING_MIN_BLOB_PIXELS:
                continue
            ys, xs = np.where(labels == i)
            sel = np.arange(0, len(xs), FLOATING_SAMPLE_STRIDE)
            blobs.append({'samples': list(zip(xs[sel].tolist(), ys[sel].tolist()))})
        return blobs

    @staticmethod
    def get_pillar_mask(camera, color):
        """Returns the binary colour mask of the pillar (uint8), or None if too few pixels.
        The mask is used to read depth from ONLY the pillar's own pixels, so the distance
        estimate isn't biased by far background around a thin pillar."""
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None:
            return None
        if color == 'blue':
            mask = cv2.inRange(hsv, np.array(BLUE_LOWER), np.array(BLUE_UPPER))
        elif color == 'yellow':
            mask = cv2.inRange(hsv, np.array(YELLOW_LOWER), np.array(YELLOW_UPPER))
        else:
            return None
        if cv2.countNonZero(mask) < MIN_PILLAR_PIXELS:
            return None
        return mask

    @staticmethod
    def centroid_of_mask(mask):
        """Returns the (u, v) centroid of a binary mask, or None if empty."""
        M = cv2.moments(mask, binaryImage=True)
        if M['m00'] == 0:
            return None
        return (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

    @staticmethod
    def get_pillar_centroid(camera, color):
        """Returns the exact (u, v) pixel coordinates of the pillar's center."""
        hsv = VisionProcessor.get_hsv_image(camera)
        if hsv is None: return None

        if color == 'blue':
            mask = cv2.inRange(hsv, np.array(BLUE_LOWER), np.array(BLUE_UPPER))
        elif color == 'yellow':
            mask = cv2.inRange(hsv, np.array(YELLOW_LOWER), np.array(YELLOW_UPPER))
        else:
            return None
            
        pixels = cv2.findNonZero(mask)
        if pixels is None or len(pixels) < MIN_PILLAR_PIXELS:
            return None
            
        # Get exact pixel coordinates of the center of the color blob
        u = int(np.mean(pixels[:, 0, 0]))
        v = int(np.mean(pixels[:, 0, 1]))
        return (u, v)