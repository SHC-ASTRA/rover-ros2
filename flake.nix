{
  description = "Development environment for ASTRA Anchor";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!

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
      in
      {
        devShells.default = pkgs.mkShell {
          name = "ASTRA Anchor";
          packages = with pkgs; [
            colcon
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
          '';
        };

        formatter = (inputs.treefmt-nix.lib.evalModule pkgs ./treefmt.nix).config.build.wrapper;
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
