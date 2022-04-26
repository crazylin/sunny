# laser_line_center (c++)

High performance line center ridge extraction algorithm with sub-pixel accuracy.

## Package, node name

laser_line_center::laser_line_center_node

## Subscription

/laser_line_center_node/image sensor_msgs::msg::Image

## Publisher

/laser_line_center_node/line sensor_msgs::msg::PointCloud2

## Parameters

- /laser_line_center_node/ksize int
- /laser_line_center_node/threshold int
- /laser_line_center_node/width_min int
- /laser_line_center_node/width_max int

## Usage information

Briefly, this algorithm finds laser line center row by row.  
For each row, the first order derivative is calculated via sobel,  
The `ksize` is the kernal size for sobel convolution.  
And the maximum and minimum values are interpolated by parabola fitting.  
The absolute value is filtered by `threshold`.  
The distance between maximum and minimum position is recorded as width.  
The width is filtered further by `width_min` and `width_max` in between.  
The results are sequence of row coordinates, and -1 is filled in between valid center.

