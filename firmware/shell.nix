{ ... }: {
  perSystem = { inputs', ... }: {
    devShells.firmware = inputs'.esp-idf.devShells.esp-idf-full;
  };
}