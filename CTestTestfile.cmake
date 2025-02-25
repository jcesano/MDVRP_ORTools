# CMake generated Testfile for 
# Source directory: C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp
# Build directory: C:/OrTools/build_test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(test_vrp "C:/OrTools/build_test/Debug/bin/vrp.exe")
  set_tests_properties(test_vrp PROPERTIES  _BACKTRACE_TRIPLES "C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;81;add_test;C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(test_vrp "C:/OrTools/build_test/Release/bin/vrp.exe")
  set_tests_properties(test_vrp PROPERTIES  _BACKTRACE_TRIPLES "C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;81;add_test;C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(test_vrp "C:/OrTools/build_test/MinSizeRel/bin/vrp.exe")
  set_tests_properties(test_vrp PROPERTIES  _BACKTRACE_TRIPLES "C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;81;add_test;C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(test_vrp "C:/OrTools/build_test/RelWithDebInfo/bin/vrp.exe")
  set_tests_properties(test_vrp PROPERTIES  _BACKTRACE_TRIPLES "C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;81;add_test;C:/OrTools/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/examples/vrp/CMakeLists.txt;0;")
else()
  add_test(test_vrp NOT_AVAILABLE)
endif()
