cmake_minimum_required(VERSION 3.16)
project(epistemic_planner VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(nlohmann_json 3.10 REQUIRED)
find_package(Threads REQUIRED)

set(CORE_SOURCES
    src/formula.cpp
    src/state.cpp
    src/action.cpp
    src/parser.cpp
    src/json.cpp
    src/product_update.cpp
    src/bisimulation.cpp
    src/hdelta.cpp
    src/heuristic.cpp
    src/knowledge_relaxation.cpp
    src/selection_policy.cpp
    src/search.cpp
    src/replan.cpp
    src/portfolio.cpp
    src/signature.cpp
    src/strategy.cpp
    src/symmetry.cpp
    src/parallel.cpp
    src/validator.cpp
)

add_executable(epistemic_planner src/main.cpp ${CORE_SOURCES})

# Enables BMI2 PEXT where available. Turn off for binaries run on another CPU.
option(ALETHEIA_NATIVE "Compile with -march=native" ON)
if(ALETHEIA_NATIVE)
  target_compile_options(epistemic_planner PRIVATE -march=native)
endif()

target_include_directories(epistemic_planner PRIVATE include)
target_link_libraries(epistemic_planner PRIVATE nlohmann_json::nlohmann_json Threads::Threads)

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
if(ipo_ok AND CMAKE_BUILD_TYPE STREQUAL "Release")
  set_property(TARGET epistemic_planner PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message(STATUS "IPO/LTO not enabled: ${ipo_msg}")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_custom_command(TARGET epistemic_planner POST_BUILD
    COMMAND ${CMAKE_STRIP} $<TARGET_FILE:epistemic_planner>
    COMMENT "Stripping binary"
  )
endif()

include(GNUInstallDirs)
install(TARGETS epistemic_planner RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# The planning core as a library, for programs that search in process rather
# than run the binary. It is compiled separately from the executable, so the
# binary keeps its LTO and -march=native build. The library gets neither:
# bitset.hpp inlines a different pext_word under BMI2, so a library built with
# -march=native and a consumer built without would disagree about an inline
# function, and slim LTO objects in an archive fail to link into a consumer
# that is not built with LTO.
option(ALETHEIA_LIBRARY "Build and install the planning core as a library" ON)
if(ALETHEIA_LIBRARY)
  include(CMakePackageConfigHelpers)

  add_library(aletheia STATIC ${CORE_SOURCES})
  add_library(aletheia::aletheia ALIAS aletheia)
  set_target_properties(aletheia PROPERTIES POSITION_INDEPENDENT_CODE ON)

  # Headers include each other by bare name, and an installed header is found
  # next to the one including it. Consumers name them as <aletheia/task.hpp>.
  target_include_directories(aletheia
    PRIVATE include
    INTERFACE "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
  target_link_libraries(aletheia
    PRIVATE nlohmann_json::nlohmann_json
    PUBLIC Threads::Threads)

  install(TARGETS aletheia EXPORT aletheiaTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aletheia)

  set(config_dir ${CMAKE_INSTALL_DATADIR}/aletheia/cmake)
  install(EXPORT aletheiaTargets NAMESPACE aletheia:: DESTINATION ${config_dir})
  configure_package_config_file(cmake/aletheiaConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/aletheiaConfig.cmake
    INSTALL_DESTINATION ${config_dir})
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/aletheiaConfigVersion.cmake
    COMPATIBILITY SameMinorVersion)
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/aletheiaConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/aletheiaConfigVersion.cmake
    DESTINATION ${config_dir})
  install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/aletheia)
endif()
