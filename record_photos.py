import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Enable streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:  # Add continuous loop
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            print("Failed to get frames")
            continue  # Skip this iteration instead of exiting

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

        # Adjust color image brightness (optional)
        color_image_bright = cv2.convertScaleAbs(color_image, alpha=1.5, beta=0)
        
        # Rotate images 90 degrees clockwise
        color_image_bright = cv2.rotate(color_image_bright, cv2.ROTATE_90_CLOCKWISE)
        depth_colormap = cv2.rotate(depth_colormap, cv2.ROTATE_90_CLOCKWISE)

        # Get timestamp for unique filenames
        timestamp = time.strftime("%Y%m%d-%H%M%S")

        # Save images
        cv2.imwrite(f'images/color_image.png', color_image_bright)
        # cv2.imwrite(f'images/depth_image.png', depth_colormap)
        
        print(f"Images saved as color_image_{timestamp}.png and depth_image_{timestamp}.png")
        
        time.sleep(2)  # Add delay between captures

except KeyboardInterrupt:  # Add keyboard interrupt handling
    print("\nStopping image capture...")

finally:
    # Stop streaming
    pipeline.stop()
