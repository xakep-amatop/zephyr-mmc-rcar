.. _rcar_h3ulcb:

R-CAR H3 ARM CA57 (ARMv8)
#################################

Overview
********

Hardware
********

Connections and IOs
===================

H3ULCB Board
------------

Here are official IOs figures from eLinux for H3ULCB board:

.. figure:: img/rcar_h3ulcb_top.jpg
   :align: center

.. figure:: img/rcar_h3ulcb_bottom.jpg
   :align: center

Supported Features
==================
The Renesas rcar_h3ulcb_ca57 board configuration supports the following
hardware features:

+-----------+------------+--------------------------------------+
| Interface | Controller | Driver/Component                     |
+===========+============+======================================+
| UART      | on-chip    |                                      |
+-----------+------------+--------------------------------------+

Other hardware features have not been enabled yet for this board.

The default configuration can be found in the defconfig file:

        ``boards/arm64/rcar_h3ulcb/rcar_h3ulcb_ca57_defconfig``

Programming and Debugging
*************************

Flashing
========

The flash on board is not supported by Zephyr at this time.

Debugging
=========
