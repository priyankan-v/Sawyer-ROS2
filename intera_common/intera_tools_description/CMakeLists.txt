cmake_minimum_required(VERSION 3.8)
project(intera_tools_description)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

foreach(dir meshes urdf)
   install(DIRECTORY ${dir}/
   DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()
