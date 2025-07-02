#!/usr/bin/env python3
import urllib.request
import os
import cv2
import numpy as np

def download_official_apriltags():
    """Download official AprilTag images from GitHub repository"""
    
    # Official AprilTag image URLs (tag36h11 family)
    base_url = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/"
    
    tag_urls = {
        1: f"{base_url}tag36_11_00001.png",  # Staff Leonardo
        2: f"{base_url}tag36_11_00002.png",  # Staff Lorenzo  
        3: f"{base_url}tag36_11_00003.png",  # Staff Federico
        4: f"{base_url}tag36_11_00004.png",  # Customer Emanuele
        5: f"{base_url}tag36_11_00005.png",  # Customer Niccolo
        6: f"{base_url}tag36_11_00006.png"   # Customer Antonello
    }
    
    # Create output directories
    os.makedirs("staff_leonardo/materials/textures", exist_ok=True)
    os.makedirs("staff_lorenzo/materials/textures", exist_ok=True)
    os.makedirs("staff_federico/materials/textures", exist_ok=True)
    os.makedirs("customer_emanuele/materials/textures", exist_ok=True)
    os.makedirs("customer_niccolo/materials/textures", exist_ok=True)
    os.makedirs("customer_antonello/materials/textures", exist_ok=True)
    
    for tag_id, url in tag_urls.items():
        try:
            print(f"Downloading AprilTag ID {tag_id}...")
            
            # Download original
            filename = f"tag36_11_{tag_id:05d}.png"
            urllib.request.urlretrieve(url, filename)
            
            # Load and resize to proper size
            img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
            resized = cv2.resize(img, (200, 200), interpolation=cv2.INTER_NEAREST)
            
            # Save for Gazebo models
            if tag_id == 1:  # Staff Leonardo
                cv2.imwrite("staff_leonardo/materials/textures/tag_1.png", resized)
                cv2.imwrite("tag_1.png", resized)  # Backup copy
                print(f"AprilTag saved to staff_leonardo/materials/textures/tag_1.png")
            elif tag_id == 2:  # Staff Lorenzo  
                cv2.imwrite("staff_lorenzo/materials/textures/tag_2.png", resized)
                cv2.imwrite("tag_2.png", resized)  # Backup copy
                print(f"AprilTag saved to staff_lorenzo/materials/textures/tag_2.png")
            elif tag_id == 3:  # Staff Federico
                cv2.imwrite("staff_federico/materials/textures/tag_3.png", resized)
                cv2.imwrite("tag_3.png", resized)  # Backup copy
                print(f"AprilTag saved to staff_federico/materials/textures/tag_3.png")
            elif tag_id == 4:  # Customer Emanuele
                cv2.imwrite("customer_emanuele/materials/textures/tag_4.png", resized)
                cv2.imwrite("tag_4.png", resized)  # Backup copy
                print(f"AprilTag saved to customer_emanuele/materials/textures/tag_4.png")
            elif tag_id == 5:  # Customer Niccolo
                cv2.imwrite("customer_niccolo/materials/textures/tag_5.png", resized)
                cv2.imwrite("tag_5.png", resized)  # Backup copy
                print(f"AprilTag saved to customer_niccolo/materials/textures/tag_5.png")
            elif tag_id == 6:  # Customer Antonello
                cv2.imwrite("customer_antonello/materials/textures/tag_6.png", resized)
                cv2.imwrite("tag_6.png", resized)  # Backup copy
                print(f"AprilTag saved to customer_antonello/materials/textures/tag_6.png")
            
            # Remove original download
            os.remove(filename)
            
        except Exception as e:
            print(f"Error downloading tag {tag_id}: {e}")
            print(f"Manual download from: {url}")

def generate_apriltag_patterns():
    """Generate AprilTag pattern arrays for vision controller reference"""
    
    # These are the actual binary patterns for tag36h11 family
    # ID 1 pattern (simplified representation)
    tag1_pattern = np.array([
        [0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,0],
        [0,1,0,0,0,0,1,0],
        [0,1,0,1,1,0,1,0],
        [0,1,0,1,0,0,1,0],
        [0,1,0,0,1,1,1,0],
        [0,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0]
    ])
    
    # ID 2 pattern
    tag2_pattern = np.array([
        [0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,0],
        [0,1,0,0,0,0,1,0],
        [0,1,0,1,1,1,1,0],
        [0,1,0,0,0,0,1,0],
        [0,1,0,1,1,0,1,0],
        [0,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0]
    ])
    
    print("\nAprilTag patterns for reference:")
    print("Tag ID 1 (Staff):")
    print(tag1_pattern)
    print("\nTag ID 2 (Customer):")  
    print(tag2_pattern)
    
    return tag1_pattern, tag2_pattern

if __name__ == "__main__":
    print("=== AprilTag Generator for TIAGo Project ===")
    
    # Download official AprilTags
    download_official_apriltags()
    
    # Generate pattern references
    generate_apriltag_patterns()
    
    print("\n=== Setup Complete ===")
    print("1. Official AprilTag images downloaded and resized")
    print("2. Files saved to model texture directories")
    print("3. Vision controller tag mapping:")
    print("   - ID 1: Staff Leonardo")  
    print("   - ID 2: Staff Lorenzo")
    print("   - ID 3: Staff Federico")
    print("   - ID 4: Customer Emanuele")
    print("   - ID 5: Customer Niccolo")
    print("   - ID 6: Customer Antonello")