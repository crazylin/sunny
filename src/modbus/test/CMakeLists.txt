add_executable(bandwidth-client bandwidth-client.c)
target_link_libraries(bandwidth-client PkgConfig::MODBUS)

add_executable(bandwidth-server bandwidth-server.c)
target_link_libraries(bandwidth-server PkgConfig::MODBUS)

add_executable(client client.cpp)
target_link_libraries(client PkgConfig::MODBUS)

add_executable(server server.cpp)
target_link_libraries(server PkgConfig::MODBUS)

add_test(
  NAME server_client
  COMMAND sh -c "sleep 5; $<TARGET_FILE:client>& $<TARGET_FILE:server>")

add_test(
  NAME bandwidth
  COMMAND sh -c "sleep 5; $<TARGET_FILE:bandwidth-client>& $<TARGET_FILE:bandwidth-server>")
