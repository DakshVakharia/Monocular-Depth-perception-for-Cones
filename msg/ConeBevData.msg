# Cone data with Bird's Eye View (BEV) projection information
# Contains both image coordinates and ground plane coordinates

string cone_id
uint8 color

# Image coordinates (from detector)
float32 x
float32 y

# Ground plane coordinates (from projector)
float32 ground_x
float32 ground_y
float32 distance

# Color enumeration
uint8 UNKNOWN = 0
uint8 ORANGE = 1
uint8 BLUE = 2
uint8 YELLOW = 3
uint8 GREEN = 4
