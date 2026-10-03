# Individual cone data (no header - timestamp comes from frame)
string cone_id
uint8 color

# Image coordinates (filled by detector)
float32 x
float32 y

# Color enumeration
uint8 UNKNOWN = 0
uint8 ORANGE = 1
uint8 BLUE = 2
uint8 YELLOW = 3
uint8 GREEN = 4
