cmake_minimum_required(VERSION 3.15)
include("cmake/gremlin/gremlin.cmake")

# ---------------------------| enviroment
# use a gloab vcpkg instance
GN_option(GN_vcpkg_vcpkgRoot "")

# tune directory names
GN_option(GN_dir_private "private")
GN_option(GN_dir_public "public")
GN_option(GN_dir_tests "tests")

GN_option(GN_staticRuntime off)
GN_option(GN_staticLinkage on)

# ---------------------------|

GN_init()
project("gravityAssists")
GN_subprojects(
    3rd-party/spice
    libs/boost
    libs/reflect
    libs/math
    libs/gsl
    project/pathfinder
    )
