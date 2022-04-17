# config_tis (python)

The one config node for almost all parameters in the pipeline.

## Package, node name

config_tis.config_tis_node

## Subscription

/config_tis_node/config

## Usage information

The config node use two layers of paramter file:  
The underlay `.params.yaml` exists as read only or backup or default file.  
It provides default values and meaningful value if the overlay failed.

The overlay `params.yaml` is to save most recently user modifications.  
It is read and updated to the underlay.

If the overlay is empty, it also means use everything in the underlay,  
which is everything in its defaults.

This node also accept a special message: `'restart'` which causes the system to restart.