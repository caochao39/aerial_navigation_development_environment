^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joy
^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2022-01-28)
------------------
* Install includes to include/ and misc CMake fixes (`#225 <https://github.com/ros-drivers/joystick_drivers/issues/225>`_)
* Style fixes for newer cpplint.
* Contributors: Chris Lalancette, Shane Loretz

3.0.1 (2022-01-28)
------------------
* Override device_id when device_name is provided. (`#199 <https://github.com/ros-drivers/joystick_drivers/issues/199>`_) (`#211 <https://github.com/ros-drivers/joystick_drivers/issues/211>`_)
* Contributors: Tiger Sachse

3.0.0 (2021-03-12)
------------------
* Fix SDL2 include path (`#196 <https://github.com/ros-drivers/joystick_drivers/issues/196>`_)
* Contributors: Scott K Logan

2.4.1 (2020-05-13)
------------------
* Small fixes for uncrustify on Foxy. (`#171 <https://github.com/ros-drivers/joystick_drivers/issues/171>`_)
* Contributors: Chris Lalancette

2.4.0 (2020-05-12)
------------------
* Cross platform joystick support for ROS 2 (`#157 <https://github.com/ros-drivers/joystick_drivers/issues/157>`_)
* roslint and Generic Clean-Up (`#161 <https://github.com/ros-drivers/joystick_drivers/issues/161>`_)
* Merge pull request `#158 <https://github.com/ros-drivers/joystick_drivers/issues/158>`_ from clalancette/ros1-cleanups
* Greatly simplify the sticky_buttons support.
* Small fixes to rumble support.
* Use C++ style casts.
* Use empty instead of length.
* joy_def_ff -> joy_dev_ff
* Cleanup header includes.
* Use size_t appropriately.
* NULL -> nullptr everywhere.
* Style cleanup in joy_node.cpp.
* Merge pull request `#154 <https://github.com/ros-drivers/joystick_drivers/issues/154>`_ from zchen24/master
* Minor: moved default to right indent level
* Contributors: Chris Lalancette, Joshua Whitley, Zihan Chen
