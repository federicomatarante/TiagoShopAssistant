#!/usr/bin/env python3
import cv2
import numpy as np

def create_apriltag_pattern(tag_id, size=200):
    """Create a simple AprilTag-like pattern"""
    img = np.ones((size, size), dtype=np.uint8) * 255
    
    if tag_id == 1:  # Staff
        # Create pattern for tag 1
        cv2.rectangle(img, (50, 50), (150, 150), 0, -1)
        cv2.rectangle(img, (75, 75), (125, 125), 255, -1)
        cv2.putText(img, '1', (90, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 2)
    else:  # Customer (tag 2)
        # Create pattern for tag 2
        cv2.rectangle(img, (40, 40), (160, 160), 0, -1)
        cv2.rectangle(img, (60, 60), (140, 140), 255, -1) 
        cv2.rectangle(img, (80, 80), (120, 120), 0, -1)
        cv2.putText(img, '2', (90, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
    
    return img

# Generate tags
for tag_id in [1, 2]:
    img = create_apriltag_pattern(tag_id)
    filename = f'tag_{tag_id}.png'
    cv2.imwrite(filename, img)
    print(f'Created {filename}')

print('AprilTag images generated successfully!')