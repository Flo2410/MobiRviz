{ pkgs ? import <nixpkgs> { } }:

let
  ros = import
    (fetchTarball {
      url = "https://github.com/lopsided98/nix-ros-overlay/archive/master.tar.gz";
      sha256 = "1cimxrlba5x1b2xl3y3n9v95py882yx2xkyybvdy47vsfar9ch2a";
    })
    { };

  ros-dist = "humble";

  venvDir = "./env";

  python = pkgs.python311.withPackages (p: with p; [
    venvShellHook
    pyqt5
    sip
  ]);

in
with pkgs;
with ros.pkgs;
with ros.rosPackages.${ros-dist};
pkgs.mkShell {
  buildInputs = [
    cmake
    gcc
    curl
    vcstool
    python

    flex
    bison
    # libncurses-dev
    usbutils

    (buildEnv {
      paths = [
        ros-base
        colcon

        geometry-msgs

        teleop-twist-keyboard
        rviz2
        rviz-common
        rviz-default-plugins
        rviz-visual-tools
        rviz-rendering
        # nav2-rviz-plugins
        # navigation2
        # nav2-bringup

        slam-toolbox

        rmw-cyclonedds-cpp
        rmw-fastrtps-cpp

        urdf-launch

        rqt
        rqt-action
        rqt-bag-plugins
        rqt-bag
        rqt-common-plugins
        rqt-console
        rqt-graph
        rqt-gui-cpp
        rqt-gui-py
        rqt-gui
        rqt-image-view
        rqt-msg
        rqt-plot
        rqt-publisher
        rqt-py-common
        rqt-py-console
        rqt-reconfigure
        rqt-service-caller
        rqt-shell
        rqt-srv
        rqt-topic
      ];
    })


  ];

  # RMW_IMPLEMENTATION = "rmw_fastrtps_cpp";

  shellHook = ''
    PYTHONPATH=\$PWD/\${venvDir}/\${python.sitePackages}/:\$PYTHONPATH
    # pip install -r requirements.txt
  '';
}
