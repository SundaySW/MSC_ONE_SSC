add_library(gtest STATIC
  "${CMAKE_CURRENT_LIST_DIR}/googletest/googletest/src/gtest-all.cc"
  "${CMAKE_CURRENT_LIST_DIR}/googletest/googletest/src/gtest_main.cc")

target_include_directories(gtest
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/googletest/googletest"
  PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/googletest/googletest/include")

target_compile_definitions(gtest
  PUBLIC
    -DGTEST_HAS_PTHREAD=0
    -DGTEST_LANG_CXX11=1)

target_compile_features(gtest
  PUBLIC
    cxx_alias_templates
    cxx_auto_type
    cxx_decltype
    cxx_final
    cxx_lambdas
    cxx_nullptr
    cxx_override)

add_executable(function2_tests
        ../include/function2/function2.hpp
        assign-and-constructible-test.cpp
        build-test.cpp
        empty-function-call-test.cpp
        function2-test.hpp
        functionality-test.cpp
        noexcept-test.cpp
        self-containing-test.cpp
        standard-compliant-test.cpp
        type-test.cpp
        multi-signature-test.cpp
        regressions.cpp
        view-test.cpp
        overload-test.cpp)

target_link_libraries(function2_tests
  PRIVATE
    function2
    gtest)

add_test(NAME function2-unit-tests COMMAND function2_tests)

add_executable(function2_playground
        ../include/function2/function2.hpp
        playground.cpp)

target_link_libraries(function2_playground
  PRIVATE
    function2)
