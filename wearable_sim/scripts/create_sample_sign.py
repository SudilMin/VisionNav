#!/usr/bin/env python3
import cv2
import numpy as np

# Create a white background image
img = np.ones((400, 800, 3), dtype=np.uint8) * 255

# Add some text (simulating a warning sign with expiry date)
cv2.putText(img, "WARNING", (50, 100), cv2.FONT_HERSHEY_DUPLEX, 3, (0, 0, 200), 5)
cv2.putText(img, "Do not enter this area.", (50, 200), cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 0, 0), 2)
cv2.putText(img, "Expires: 12/2026", (50, 300), cv2.FONT_HERSHEY_DUPLEX, 1.2, (50, 50, 50), 2)

# Save it
cv2.imwrite("sample_sign.png", img)
print("Created sample_sign.png for testing!")
