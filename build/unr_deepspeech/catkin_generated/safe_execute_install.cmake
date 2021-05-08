execute_process(COMMAND "/home/nathan/barista_ws/build/unr_deepspeech/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nathan/barista_ws/build/unr_deepspeech/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
