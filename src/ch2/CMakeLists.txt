add_executable(motion motion.cc)
target_link_libraries(motion
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )
add_executable(motion_g motion_g.cc)
target_link_libraries(motion_g
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )
