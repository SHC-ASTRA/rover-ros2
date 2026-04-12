{ ... }:
{
  projectRootFile = "flake.nix";
  programs = {
    nixfmt.enable = true;
    black.enable = true;
    shfmt.enable = true;
  };
}
