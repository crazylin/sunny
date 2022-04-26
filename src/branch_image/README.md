# branch_image (c++)

Branch input images based on image's frame ID, so images may be processed simultaneously.

## Package, node name

branch_image::branch_image_node

## Subscription

/branch_image_node/image sensor_msgs::msg::Image

## Publisher

- /branch_image_node/image_l sensor_msgs::msg::Image
- /branch_image_node/image_r sensor_msgs::msg::Image