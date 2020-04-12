GN_vcpkg_install(gsl)

find_package(GSL REQUIRED)

GN_unit(gsl bFlat
    libs GSL::gsl GSL::gslcblas
    )
