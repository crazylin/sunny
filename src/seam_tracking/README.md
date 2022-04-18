# seam_tracking (python)

A plugin based system supports self written python code to extract seam.

## Package, node name

seam_tracking::seam_tracking_node

## Subscription

/seam_tracking_node/pnts sensor_msgs::msg::PointCloud2

## Publisher

/seam_tracking_node/seam sensor_msgs::msg::PointCloud2

## Parameters

- /seam_tracking_node/enable bool
- /seam_tracking_node/window_size int
- /seam_tracking_node/gap int
- /seam_tracking_node/step: float
- /seam_tracking_node/length: int

## Usage information

Briefly, this algorithm check connectivity to a sequence of points in time space.  
The number of points to check is given by `window_size`.  
Two consecutive points are recognized as break point by `gap`.  
After that, a front cluster algorithm is applied to group points into segment of lines.  
Two segments are connected if the missing number points is smaller than `step`.  
Finally, lines with shorter `length` are filtered altogether.
