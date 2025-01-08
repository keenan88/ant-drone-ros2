import apriltag
import cv2

# Read the image in grayscale
img = cv2.imread('extreme_angle.png', cv2.IMREAD_GRAYSCALE)

# Initialize the detector
detector = apriltag.Detector()

# Detect AprilTags
results = detector.detect(img)

# Camera intrinsics (fx, fy, cx, cy)
camera_params = [600, 600, 320, 240]  # Replace with actual camera calibration values

# Physical tag size in meters
tag_size = 0.1  # Replace with the actual size of the tag

# Process each detection
for result in results:
    # Compute pose
    pose = detector.detection_pose(result, camera_params, tag_size)

    print(pose)

    corners = result.corners
    # Convert to integer values for drawing
    corners = [(int(x), int(y)) for x, y in corners]
    print(corners)
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Draw lines between the corners
    for i in range(4):
        pt1 = corners[i]
        pt2 = corners[(i + 1) % 4]  # Wrap around to the first corner
        cv2.line(img_color, pt1, pt2, (0, 0, 255), 2)  # Red color, thickness 2

# Display results
cv2.imshow('Detected AprilTags', img_color)
cv2.waitKey(0)
cv2.destroyAllWindows()
