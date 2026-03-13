{
  description = "Truck-N-Trailer Dev Flake";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    esp-idf.url = "github:mirrexagon/nixpkgs-esp-dev";
  };
  outputs = {
    nixpkgs,
    flake-utils,
    esp-idf,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
      };
    in {
      devShells = {
        default = with pkgs;
          mkShell {
            buildInputs = [
              python3
              uv
              ipopt
              python3Packages.pyqt6
              qt6.qtwayland
            ];
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
              pkgs.stdenv.cc.cc
              pkgs.zlib
              pkgs.glib
              pkgs.libGL
              pkgs.libGLU
              pkgs.libxcb
              pkgs.libxext
              pkgs.libx11
              pkgs.libsm
              pkgs.libice
            ];
            shellHook = ''
              if [ ! -d ".venv" ]; then
                uv venv
              fi
              source .venv/bin/activate
              export PYTHONPATH="${python3Packages.pyqt6}/${python3.sitePackages}:$PYTHONPATH"
            '';
          };
        esp = esp-idf.devShells.${system}.esp-idf-full;
      };
    });
}
