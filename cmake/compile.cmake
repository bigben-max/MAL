macro(cmake_add_library Name)
  add_library(${Name} ${${Name}_SRCS})
  target_link_libraries(${Name} ${${Name}_LIBS})
endmacro(cmake_add_library)

macro(cmake_add_static_library Name)
  add_library(${Name} STATIC ${${Name}_SRCS})
  target_link_libraries(${Name} ${${Name}_LIBS})
endmacro(cmake_add_static_library)

macro(cmake_add_executable Name)
  add_executable(${Name} ${${Name}_SRCS})
  target_link_libraries(${Name} ${${Name}_LIBS})
endmacro(cmake_add_executable)

macro(my_add_node Name)
  add_executable(${Name}_node ${${Name}_SRCS})
  target_link_libraries(${Name}_node ${${Name}_LIBS})
endmacro(my_add_node)

macro(my_add_test Name)
  add_executable(${Name}_test ${${Name}_SRCS})
  target_link_libraries(${Name}_test ${${Name}_LIBS})
endmacro(my_add_test)

macro(install_node)
  # Mark executable scripts (Python etc.) for installation 添加python程序．in
  # contrast to setup.py, you can choose the destination
  install(PROGRAMS scripts/*.py scripts/*sh
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  # 添加可执行文件或者库文件，Mark executables and/or libraries for installation
  install(
    TARGETS *_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  # 添加头文件．Mark cpp header files for installation
  install(
    DIRECTORY inc/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN ".hpp" EXCLUDE)
  # 添加资源文件的目录，例如文件夹：urdf mesh rviz，其下的所有子目录的文件也会安装到相应的目录下．
  # install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  # install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  # install(DIRECTORY mesh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  # install(DIRECTORY rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  # 添加资源文件．Mark other files for installation (e.g. launch and bag files, etc.)
  install(FILES launch/*.launch
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(FILES **.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

endmacro(install_node)

function(generate_cpp_pb_file pb_path proto_gen_cpp_files)
  set(pb_file_path ${pb_path}) # 设置pb file实际所在的目录
  file(GLOB proto_files ${pb_file_path}/*.proto) # 找出所有的pb file
  set(pb_out_path ${pb_file_path}) # 设置pb file生成文件的输出目录

  # message("PROTOC ${PROTOBUF_PROTOC_EXECUTABLE}") message("pb_path
  # ${pb_path}") message("pb_out_path ${pb_out_path}")
  foreach(pb_file ${proto_files})
    get_filename_component(pb_file_name ${pb_file} NAME_WE) # 获取pb file文件名
    set(cur_out_file # 设置输出文件名
        ${pb_out_path}/${pb_file_name}.pb.h
        ${pb_out_path}/${pb_file_name}.pb.cc)
    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${pb_file}
                            --cpp_out=${pb_out_path} -I=${pb_out_path})
  endforeach(pb_file ${proto_files})

  # set_source_files_properties(${pb_out_files} PROPERTIES GENERATED TRUE)
  # Create lists of files to be generated
  set(proto_gen_cpp_files_list "")
  foreach(proto_file ${proto_files})
    get_filename_component(proto_name ${proto_file} NAME_WE)
    list(APPEND proto_gen_cpp_files_list ${pb_out_path}/${proto_name}.pb.h
         ${pb_out_path}/${proto_name}.pb.cc)
  endforeach(proto_file ${proto_files})
  set(proto_gen_cpp_files
      ${proto_gen_cpp_files_list}
      PARENT_SCOPE)
  # message("proto_gen_cpp_files = ${proto_gen_cpp_files_list}")
endfunction()
