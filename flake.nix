{
  description = "Python Dev Flake";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs = {
    nixpkgs,
    flake-utils,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
      };
    in {
      devShells.default = with pkgs;
        mkShell {
          buildInputs = [
            python3
            uv
          ];
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
            pkgs.stdenv.cc.cc
            pkgs.zlib
            pkgs.glib
            pkgs.libGL
            pkgs.libGLU
            pkgs.libxcb
          ];
          shellHook = ''
            if [ ! -d ".venv" ]; then
              uv venv
            fi
            source .venv/bin/activate
          '';
        };
    });
}
