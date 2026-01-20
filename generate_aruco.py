import cv2
import cv2.aruco as aruco
import numpy as np

# Define the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)

# Generate the marker using drawMarker
# ID 26, 156x156 pixels (to make total 256 with 50px border)
img = aruco.drawMarker(aruco_dict, 26, 156)

# Add a white border (50 pixels) -> Total 256x256
img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)

# Save the image
cv2.imwrite('/home/eugegeuge/Documents/autonomous_forklift/src/autonomous_forklift/models/aruco_marker/materials/textures/aruco_texture_id26.png', img)
print("ArUco marker ID 26 generated successfully (256x256).")
