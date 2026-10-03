# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Builds the board devicetree, when the board has one, and generates C from
# it with dtmap: devicetree/hw.h (generated include dir), the hw.c the board
# library compiles, and Kconfig.dt, sourced by Kconfig. Runs before Kconfig,
# and again whenever a devicetree source or dtmap file changes. The blob is
# checked against the Linux bindings plus dts/bindings at build time.
# See docs/development/devicetree.md.

set(PBL_DT_DIR ${PROJECT_BINARY_DIR}/devicetree)

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/devicetree.py generate
          --srcdir ${PBL_BASE}
          --builddir ${PROJECT_BINARY_DIR}
          --board ${BOARD}
          --cc ${CMAKE_C_COMPILER}
  WORKING_DIRECTORY ${PBL_BASE}
  RESULT_VARIABLE ret
  OUTPUT_VARIABLE summary
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT ret EQUAL 0)
  message(FATAL_ERROR "Devicetree failed")
endif()
message(STATUS "Devicetree: ${summary}")

include(${PBL_DT_DIR}/devicetree.cmake)

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${PBL_DT_DEPENDS} ${PBL_BASE}/tools/cmake/devicetree.py)

function(pbl_devicetree_validate)
  if(NOT PBL_DT_ENABLED)
    return()
  endif()

  get_filename_component(python_bin ${PYTHON_EXECUTABLE} DIRECTORY)
  find_program(DT_MK_SCHEMA dt-mk-schema HINTS ${python_bin})
  find_program(DT_VALIDATE dt-validate HINTS ${python_bin})
  if(NOT DT_MK_SCHEMA OR NOT DT_VALIDATE)
    message(WARNING "dt-validate not found (pip install dtschema): "
                    "the devicetree is not checked against its bindings")
    return()
  endif()

  set(linux_bindings ${PBL_BASE}/third_party/devicetree/devicetree-rebasing/Bindings)
  set(our_bindings ${PBL_BASE}/dts/bindings)
  file(GLOB_RECURSE bindings CONFIGURE_DEPENDS
    ${linux_bindings}/*.yaml ${our_bindings}/*.yaml)

  set(schema ${PBL_DT_DIR}/processed-schema.json)
  set(stamp ${PBL_DT_DIR}/validate.stamp)
  set(script ${PBL_BASE}/tools/cmake/devicetree.py)
  add_custom_command(
    OUTPUT ${schema}
    COMMAND ${PYTHON_EXECUTABLE} ${script} schema
            --dt-mk-schema ${DT_MK_SCHEMA} --output ${schema}
            --vendor-prefixes ${our_bindings}/vendor-prefixes.txt
            ${linux_bindings} ${our_bindings}
    DEPENDS ${bindings} ${our_bindings}/vendor-prefixes.txt ${script}
    COMMENT "Processing devicetree bindings"
    VERBATIM
  )
  add_custom_command(
    OUTPUT ${stamp}
    COMMAND ${PYTHON_EXECUTABLE} ${script} validate
            --dt-validate ${DT_VALIDATE} --schema ${schema}
            --dtb ${PBL_DT_DTB} --stamp ${stamp}
    DEPENDS ${schema} ${PBL_DT_DTB} ${script}
    COMMENT "Validating the devicetree"
    VERBATIM
  )
  add_custom_target(pbl_devicetree_validate ALL DEPENDS ${stamp})
endfunction()
