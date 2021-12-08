include(FindPackageHandleStandardArgs)
find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(CCD ccd>=2.0)
    if(CCD_LIBRARIES AND NOT CCD_INCLUDE_DIRS)
        set(CCD_INCLUDE_DIRS "/usr/include")
    endif()
endif()
find_package_handle_standard_args(ccd DEFAULT_MSG CCD_LIBRARIES CCD_INCLUDE_DIRS)
