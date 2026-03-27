{
  description = "Truck-N-Trailer Dev Flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    esp-idf.url = "github:mirrexagon/nixpkgs-esp-dev";
  };

  outputs = {
    nixpkgs,
    esp-idf,
    ...
  }: let
    systems = [
      "x86_64-linux"
      "aarch64-linux"
      "x86_64-darwin"
      "aarch64-darwin"
    ];
    forAllSystems = nixpkgs.lib.genAttrs systems;
  in {
    devShells = forAllSystems (system: let
      pkgs = import nixpkgs {inherit system;};
    in {
      core = pkgs.mkShell {
        buildInputs = with pkgs; [
          python3
          uv
          ipopt
          python3Packages.pyqt6
        ];

        LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
          stdenv.cc.cc
          zlib
          glib
          libGL
          libGLU
          libxcb
          libxext
          libx11
          libsm
          libice
        ]);

        shellHook = ''
          if [ ! -d ".venv" ]; then
            uv venv
          fi
          source .venv/bin/activate
        '';
      };

      firmware = esp-idf.devShells.${system}.esp-idf-full;
    });
  };
}
