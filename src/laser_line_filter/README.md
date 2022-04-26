# laser_line_filter (c++)

Moving average algorithm to filter out noise points.

## Package, node name

laser_line_filter::laser_line_filter_node

## Subscription

/laser_line_filter_node/line sensor_msgs::msg::PointCloud2

## Publisher

/laser_line_filter_node/line_filtered sensor_msgs::msg::PointCloud2

## Parameters

- /laser_line_filter_node/enable bool
- /laser_line_filter_node/window_size int
- /laser_line_filter_node/gap int
- /laser_line_filter_node/deviate: float
- /laser_line_filter_node/step: float
- /laser_line_filter_node/length: int

## Usage information

Briefly, this algorithm applies moving average algorithm to a sequence of points.  
The average is calculated from a centered window by `window_size`,  
The total averaged points are `window_size` * 2 + 1.  
The difference between averaged and original value is recorded as `deviate`,  
It is filtered out immediately if it is larger than given value.  
Then two consecutive points are recognized as break point by `gap`.  
After that, a front cluster algorithm is applied to group points into segment of lines.  
Two segments are connected if the missing number points is smaller than `step`.  
Finally, lines with shorter `length` are filtered altogether.  
The `enable` controls whether this filter should apply or not.
