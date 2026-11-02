# Same-frame thermal observation (header copied from thermal Image).
std_msgs/Header header

bool raw_hot_blob
# True iff THIS image has a valid hot blob in ROI.

bool fire_debounced
# Debounced state after this frame's hysteresis update.

bool bearing_valid
# True only when raw_hot_blob and a bearing were computed for THIS frame.
# When false, bearing_rad and blob_half_width_rad are unused (written 0).

float64 bearing_rad
float64 blob_half_width_rad
float32 max_roi_temperature_k
