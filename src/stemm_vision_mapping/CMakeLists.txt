cmake_minimum_required(VERSION 3.8)
project(stemm_vision_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/save_grid.py
        scripts/make_highlight.py
        scripts/grid_confidence.py
        scripts/cloud_to_grid.py
        scripts/merge_maps.py
        scripts/merge_nav_params.py
        scripts/depth_obstacle_grid.py
        scripts/vision_preflight.sh
        scripts/save_vision_map.sh
        DESTINATION lib/${PROJECT_NAME})

ament_package()
