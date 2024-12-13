find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
set(Eigen3_TARGETS Eigen3::Eigen)
if(TARGET adore_map)
    target_link_libraries(adore_map PRIVATE ${PROJ_LIBRARIES} ${OpenCV_LIBS})
endif()

