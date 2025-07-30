# Detection.msg
# Bounding box coordinates
flyscan_interfaces/BoundingBox bbox
# Detection confidence score (0.0 to 1.0)
float32 confidence
# Class ID number
int32 class_id
# Human-readable class name
string class_name
# QR/Barcode decoded data (empty if not decoded or not QR/Barcode)
string decoded_data
# Processing parameters used for successful decode (JSON string)
string decode_params