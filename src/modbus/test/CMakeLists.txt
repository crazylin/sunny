add_executable(bandwidth-client bandwidth-client.c)
target_link_libraries(bandwidth-client PkgConfig::MODBUS)

add_executable(bandwidth-server bandwidth-server.c)
target_link_libraries(bandwidth-server PkgConfig::MODBUS)

add_executable(connect connect.cpp)
target_link_libraries(connect PkgConfig::MODBUS)

add_test(
  NAME bandwidth
  COMMAND sh -c "$<TARGET_FILE:bandwidth-server>& sleep 5; $<TARGET_FILE:bandwidth-client>& wait")
