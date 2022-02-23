# sunny

## Name convention

| Context      | Syntax | Example |
| :---------- | :---------- | :---------- |
| ROS2 package name      | abc_def       | intra_process_demo |
| ROS2 node name      | abc_def_node       | camera_mvs_node |
| ROS2 node header(c++)      | abc_def.hpp      | camera_mvs.hpp |
| ROS2 node source(c++)      | abc_def.cpp      | camera_mvs.cpp |
| ROS2 node namespace(c++)      | abc_def      | namespace camera_mvs |
| ROS2 node class(c++)      | AbcDef      | class CameraMvs |
| ROS2 node header include guard(c++)      | ABC_DEF__ABC_DEF_HPP_      | PACKAGE_NAME__CLASS_NAME_HPP_ |
| ROS2 service name      | abc_def       | set_parameters, get_parameters |
| ROS2 service type   | AbcDef        | SetParameters, GetParameters |
| ROS2 srv file name   | AbcDef.srv        | SetParameters.srv, GetParameters.srv |
| ROS2 topic name      | abc_def       | parameter_events |
| ROS2 topic type   | AbcDef        | ParameterEvent |
| ROS2 msg file name   | AbcDef.msg        | ParameterEvent.msg |
| ROS2 launch file name      | main.launch.py       |  |
| var in srv or msg | abc_def | new_parameters |
