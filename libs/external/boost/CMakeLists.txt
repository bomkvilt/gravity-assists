GN_vcpkg_install(boost)
find_package(Boost REQUIRED 
    filesystem 
    date_time 
    thread
    system
    regex
    )

add_library(boost_interface INTERFACE)
target_link_libraries(boost_interface INTERFACE ${Boost_LIBRARIES})
target_include_directories(boost_interface INTERFACE ${Boost_INCLUDE_DIR})

GN_unit(boost bFlat
    libs boost_interface
    )
