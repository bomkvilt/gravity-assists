cmake_minimum_required(VERSION 3.15)
include("build/cmake/gremlin/gremlin.cmake")

# ---------------------------| enviroment
# use a gloab vcpkg instance
GN_option(GN_vcpkg_vcpkgRoot "")

# tune directory names
GN_option(GN_dir_private "private")
GN_option(GN_dir_public "public")
GN_option(GN_dir_tests "tests")

GN_option(GN_staticRuntime off)
GN_option(GN_staticLinkage on)

# use double precission
set(bDouble on)

# ---------------------------|

GN_init()
project("gravityAssists")
GN_subprojects(
    libs/common
    3rd-party/spice
    3rd-party/boost
    3rd-party/gsl
    3rd-party/clflags
    libs/reflect
    libs/math
    project/pathfinder
    project/main
    )
