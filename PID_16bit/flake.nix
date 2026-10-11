{
  description = "Hardware PID Controller Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
  let
    system = "x86_64-linux";
    pkgs = import nixpkgs {
      inherit system;
    };
  in
  {
    devShells.${system}.default =
      pkgs.mkShell {
        packages = with pkgs; [
          python3
          python3Packages.pip
          python3Packages.virtualenv
          gnumake
          gcc
          verilator
          yosys
          gtkwave
          z3
          sby
          git
        ];

        shellHook = ''
          echo "=== Hardware PID Flight Controller Environment ==="
          verilator --version
          yosys -V
          python3 --version
        '';
      };
  };
}
