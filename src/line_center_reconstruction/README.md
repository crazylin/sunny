# line_center_reconstruction (c++)

Homography transformation between two plane: image plane and laser plane

## Package, node name

line_center_reconstruction::line_center_reconstruction_node

## Subscription

/line_center_reconstruction_node/line sensor_msgs::msg::PointCloud2

## Publisher

/line_center_reconstruction_node/pnts sensor_msgs::msg::PointCloud2

## Parameters

- /line_center_reconstruction_node/camera_matrix std::vector<float>
- /line_center_reconstruction_node/distort_coeffs std::vector<float>
- /line_center_reconstruction_node/homography_matrix std::vector<float>

## Usage information

The transformation applies to two planes in arbitrary 3D space.
A perspective transformation `homography_matrix` between the source and the destination planes is found via calibration step.
A possible camera len undistortion is utilized using opencv's undistortPoints given `camera_matrix` and `distort_coeffs`.
For more details, see [calib3d](https://docs.opencv.org/4.5.5/d9/d0c/group__calib3d.html).