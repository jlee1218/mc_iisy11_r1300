cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME mc_iisy11_r1300)
set(PROJECT_DESCRIPTION "mc iisy11 r1300 robot module")
set(PROJECT_URL "")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

if(MSVC AND _VCPKG_INSTALLED_DIR)
  set(VCPKG_INSTALL_PREFIX "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_PREFIX}")
  # Only debug actually exists but we make sure it's first in that case
  set(CMAKE_MSVCIDE_RUN_PATH
      "${VCPKG_INSTALL_PREFIX}/\$(Configuration)/bin;${VCPKG_INSTALL_PREFIX}/bin;${CMAKE_MSVCIDE_RUN_PATH}"
  )
endif()

find_package(mc_rtc REQUIRED)

if(NOT DEFINED MC_ISSY11_R1300_DESCRIPTION_PATH)
	find_description_package(mc_iisy11_r1300_description)
endif()

add_subdirectory(src)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
