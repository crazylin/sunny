macro(test filename)
    add_executable(${filename} ${filename}.cpp)
    target_link_libraries(${filename} ${OpenCV_LIBS})
    target_include_directories(${filename} PUBLIC ${CMAKE_SOURCE_DIR}/include)
    add_test(NAME ${filename} COMMAND ${filename})
endmacro()

foreach(X IN ITEMS empty small black white threshold ksize width perf)
    test(${X})
endforeach()

add_executable(producer_consumer_manager producer_consumer_manager.cpp)
target_link_libraries(producer_consumer_manager pthread)
add_test(NAME producer_consumer_manager COMMAND producer_consumer_manager)
