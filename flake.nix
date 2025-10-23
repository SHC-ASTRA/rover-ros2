{
  description = "Development environment for ASTRA Anchor";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
  };

  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
    }:
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
            (python312.withPackages (
              p: with p; [
                pyserial
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
                  # ros2-controllers nixpkg does not build :(
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
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
