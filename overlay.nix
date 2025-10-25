final: prev:
{
  anchor-pkg = final.callPackage ././install/anchor_pkg/share/anchor_pkg/package.nix {};
  arm-description = final.callPackage ././install/arm_description/share/arm_description/package.nix {};
  arm-moveit-config = final.callPackage ././install/arm_moveit_config/share/arm_moveit_config/package.nix {};
  arm-pkg = final.callPackage ././install/arm_pkg/share/arm_pkg/package.nix {};
  bio-pkg = final.callPackage ././install/bio_pkg/share/bio_pkg/package.nix {};
  core-description = final.callPackage ././install/core_description/share/core_description/package.nix {};
  core-gazebo = final.callPackage ././install/core_gazebo/share/core_gazebo/package.nix {};
  core-pkg = final.callPackage ././install/core_pkg/share/core_pkg/package.nix {};
  headless-pkg = final.callPackage ././install/headless_pkg/share/headless_pkg/package.nix {};
  latency-tester = final.callPackage ././install/latency_tester/share/latency_tester/package.nix {};
  ros2-interfaces-pkg = final.callPackage ././install/ros2_interfaces_pkg/share/ros2_interfaces_pkg/package.nix {};
  servo-arm-twist-pkg = final.callPackage ././install/servo_arm_twist_pkg/share/servo_arm_twist_pkg/package.nix {};
}
