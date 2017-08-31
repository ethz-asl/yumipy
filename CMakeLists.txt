cmake_minimum_required(VERSION 2.8.3)
project(yumipy)

find_package(catkin_simple REQUIRED)


catkin_python_setup()



catkin_simple(ALL_DEPS_REQUIRED)
cs_install()
cs_export()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
