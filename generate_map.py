import cv2
import numpy as np
import os

# Create a white image (representing free space)
# 400x400 pixels, at 0.05m/px resolution = 20x20 meters
# Center is at (200, 200) which corresponds to (0,0) meters
img = np.ones((400, 400), dtype=np.uint8) * 255

# Draw Walls (Black = Occupied)
# Top Wall (y=5m -> pixel y=100)
cv2.rectangle(img, (0, 98), (400, 102), (0, 0, 0), -1)
# Bottom Wall (y=-5m -> pixel y=300)
cv2.rectangle(img, (0, 298), (400, 302), (0, 0, 0), -1)
# Right Wall (x=10m -> pixel x=398)
cv2.rectangle(img, (398, 0), (400, 400), (0, 0, 0), -1)
# Left Wall (x=-10m -> pixel x=0)
cv2.rectangle(img, (0, 0), (2, 400), (0, 0, 0), -1)

# Draw ArUco Marker (Box at x=3.0, y=0.0)
# x=3.0 -> pixel x = 200 + 3.0/0.05 = 260
# y=0.0 -> pixel y = 200
cv2.rectangle(img, (258, 190), (262, 210), (0, 0, 0), -1)

# Save to the mvsim directory
output_path = 'src/autonomous_forklift/mvsim/empty_map.png'
os.makedirs(os.path.dirname(output_path), exist_ok=True)
cv2.imwrite(output_path, img)
print(f"Created {output_path}")
