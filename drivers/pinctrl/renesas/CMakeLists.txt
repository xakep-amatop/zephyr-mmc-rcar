# Copyright (c) 2024 EPAM Systems
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_SOC_R8A77951 OR CONFIG_SOC_ARM64_R8A77951)
zephyr_library_sources(pfc_r8a77951.c)
endif()
zephyr_library_sources_ifdef(CONFIG_SOC_R8A77961 pfc_r8a77961.c)
zephyr_library_sources_ifdef(CONFIG_SOC_R8A779F0 pfc_r8a779f0.c)
zephyr_library_sources_ifdef(CONFIG_PINCTRL_RCAR_PFC pfc_rcar.c)
zephyr_library_sources_ifdef(CONFIG_PINCTRL_SMARTBOND pinctrl_smartbond.c)
zephyr_library_sources_ifdef(CONFIG_PINCTRL_RA pinctrl_ra.c)
zephyr_library_sources_ifdef(CONFIG_PINCTRL_RZT2M pinctrl_rzt2m.c)
