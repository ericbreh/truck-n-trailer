{...}: {
  perSystem = {pkgs, ...}: {
    devShells.core = pkgs.mkShell {
      buildInputs = with pkgs; [
        python3
        uv
        ipopt
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
  };
}
