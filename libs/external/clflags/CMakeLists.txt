GN_vcpkg_install(gflags)

find_package(gflags CONFIG REQUIRED)

GN_unit(clflags
    libs gflags_static
    )
