add_executable(pick_place_tutorial
    src/pick_place_tutorial.cpp)
target_include_directories(pick_place_tutorial
    PUBLIC include)
ament_target_dependencies(pick_place_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS pick_place_tutorial
  DESTINATION lib/${PROJECT_NAME}
)