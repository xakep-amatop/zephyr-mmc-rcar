# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources(pfc_r8a7795.c)
zephyr_library_sources_ifdef(CONFIG_ARM_MMU mmu_regions.c)


dt_comp_path(chosen_pfc_list COMPATIBLE "renesas,rcar-pfc")
foreach(path IN LISTS chosen_pfc_list)
  dt_node_has_status(status_result PATH ${path} STATUS okay)
  if(${status_result})
    zephyr_library_sources(pfc_r8a7795.c)
    break()
  endif()
endforeach()
