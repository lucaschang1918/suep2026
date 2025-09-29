cmake /home/rm_autoaim/suep2026/dev_ws/src/rm_auto_aim/armor_detector \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -G Ninja \
      -DCMAKE_INSTALL_PREFIX=/home/rm_autoaim/suep2026/dev_ws/install


cmake --build /home/rm_autoaim/suep2026/dev_ws/build/armor_detector -- -j8


cmake --install /home/rm_autoaim/suep2026/dev_ws/build/armor_detector
