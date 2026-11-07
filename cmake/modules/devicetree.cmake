# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Generates the instance file of each driver with devices, which the driver
# includes, and devicetree/devices.h. The tree is compiled by
# tools/cmake/kconfig.py before Kconfig runs. Boards without a devicetree are
# described in C alone.

function(pbl_devicetree)
  if(NOT PBL_DTB)
    return()
  endif()

  set(tool ${PBL_BASE}/tools/devicetree)
  set(outdir ${CMAKE_BINARY_DIR}/generated/include)
  set(outputs ${outdir}/devicetree/devices.h)
  foreach(output ${PBL_DT_OUTPUTS})
    list(APPEND outputs ${outdir}/${output})
  endforeach()
  add_custom_command(
    OUTPUT ${outputs}
    COMMAND ${PYTHON_EXECUTABLE} ${tool}/dtmap.py
            --srcdir ${PBL_BASE}
            --dtb ${PBL_DTB}
            --config ${PBL_DOTCONFIG}
            --outdir ${outdir}
    DEPENDS ${PBL_DTB} ${PBL_DOTCONFIG} ${PBL_KCONFIG_DEPENDS} ${tool}/dtmap.py ${tool}/fdt.py
    COMMENT "Generating the board devices from devicetree"
    VERBATIM
  )
  add_custom_target(pbl_devicetree DEPENDS ${outputs})
  add_dependencies(pbl_generated_headers pbl_devicetree)
endfunction()
