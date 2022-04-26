# pipeline_tis (python launch)

The main pipeline run in embedded system.

## Package, node name

pipeline_tis::pipeline_tis_node

## Usage information

This file launchs a bunch of Nodes in certain topology.  
Briefly, the list of nodes are:
- camera_tis
- rotate_image
- laser_line_center
- laser_line_filter
- line_center_reconstruction
- seam_tracking
- modbus
- gpio_raspberry
