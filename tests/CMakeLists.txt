if(TARGET Iisy11r1300Module)
  set(ROBOT_MODULE_PARAMS "\"Iisy11r1300Module\"")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/loader.in.cpp"
                 "${CMAKE_CURRENT_BINARY_DIR}/Iisy11r1300Module/loader.cpp")
  add_executable(loader_Iisy11r1300Module
                 "${CMAKE_CURRENT_BINARY_DIR}/Iisy11r1300Module/loader.cpp")
  target_link_libraries(loader_Iisy11r1300Module PUBLIC mc_rtc::mc_rbdyn)
  add_test(NAME loader_Iisy11r1300Module
           COMMAND loader_Iisy11r1300Module "$<TARGET_FILE_DIR:Iisy11r1300Module>")
endif()


