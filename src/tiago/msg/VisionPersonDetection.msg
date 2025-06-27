# Person detection message from vision system
string person_id          # Unique identifier for the detected person
string cls               # Person classification (e.g., "customer", "staff", "unknown")
geometry_msgs/Point position    # 3D position of the person in robot frame
