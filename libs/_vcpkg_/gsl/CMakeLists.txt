GN_vcpkg_install(gsl)

find_package(GSL REQUIRED)

GN_unit(gsl
    libs GSL::gsl GSL::gslcblas
    units common
    )
