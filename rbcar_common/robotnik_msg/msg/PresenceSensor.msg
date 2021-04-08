string CAMERA=camera
string PHOTOCELL=photocell

Header header
geometry_msgs/Pose pose
string sensor_id # Which sensor is detecting the object (which camera or which photocell)
string sensor_type # camera or photocell
string detected_id # if QR is being detected, in photocell would be void
string zone 
bool enabled # Determines if the sensor is on or off 
bool value # Determines whether the sensor is detecting the object
