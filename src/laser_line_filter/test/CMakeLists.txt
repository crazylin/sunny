macro(test filename)
    add_executable(${filename} ${filename}.cpp)
    target_include_directories(${filename} PUBLIC ${CMAKE_SOURCE_DIR}/include)
    add_test(NAME ${filename} COMMAND ${filename})
endmacro()

foreach(X IN ITEMS average length)
    test(${X})
endforeach()
