project(ROBOT_APP C)
add_cfe_app(robot_app fsw/src/robot_app.c fsw/src/robot_control.c)
target_include_directories(robot_app PUBLIC fsw/inc)
