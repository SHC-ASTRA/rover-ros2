{
  description = "Development environment for ASTRA Anchor";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!

    astra-msgs = {
      url = "github:SHC-ASTRA/astra_msgs?ref=a310e967fedf9a2b7eb340839b3edf8a52ba32f8";
      inputs.nix-ros-overlay.follows = "nix-ros-overlay";
    };

    astra-descriptions = {
      url = "github:SHC-ASTRA/astra_descriptions?ref=e9dd878727d9cf0f93dafabc4ecaae352b2d1f81";
      inputs.nix-ros-overlay.follows = "nix-ros-overlay";
    };

    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      ...
    }@inputs:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        astra-msgs-pkgs = inputs.astra-msgs.packages.${system};
        astra-descriptions-pkgs = inputs.astra-descriptions.packages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          name = "ASTRA Anchor";
          packages = with pkgs; [
            colcon
            socat
            can-utils
            (python313.withPackages (
              p: with p; [
                pyserial
                python-can
                pygame
                scipy
                crccheck
                black
              ]
            ))
            (
              with rosPackages.humble;
              buildEnv {
                paths = [
                  # Custom ROS2 packages
                  astra-msgs-pkgs.astra-msgs
                  astra-descriptions-pkgs.arm-description
                  astra-descriptions-pkgs.arm-moveit-config
                  astra-descriptions-pkgs.core-description

                  ros-core
                  ros2cli
                  ros2run
                  ros2bag
                  rviz2
                  xacro
                  ament-cmake-core
                  python-cmake-module
                  diff-drive-controller
                  parameter-traits
                  generate-parameter-library
                  joint-state-publisher-gui
                  robot-state-publisher
                  ros2-control
                  controller-manager
                  control-msgs
                  control-toolbox
                  moveit-core
                  moveit-planners
                  moveit-common
                  moveit-msgs
                  moveit-ros-planning
                  moveit-ros-planning-interface
                  moveit-ros-visualization
                  moveit-configs-utils
                  moveit-ros-move-group
                  moveit-servo
                  moveit-simple-controller-manager
                  topic-based-ros2-control
                  pilz-industrial-motion-planner
                  pick-ik
                  ompl
                  joy
                  ros2-controllers
                  chomp-motion-planner
                ];
              }
            )
          ];
          shellHook = ''
            # Display stuff
            export DISPLAY=''${DISPLAY:-:0}
            export QT_X11_NO_MITSHM=1

            # Enable ros2 command autocomplete
            eval "$(register-python-argcomplete ros2)"
            eval "$(register-python-argcomplete colcon)"
          '';
        };

        formatter = (inputs.treefmt-nix.lib.evalModule pkgs ./treefmt.nix).config.build.wrapper;
      }
    );

  nixConfig = {
    # Cache to pull ros packages from
    extra-substituters = [
      "https://ros.cachix.org"
      "https://attic.iid.ciirc.cvut.cz/ros"
    ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q="
    ];
  };
}
