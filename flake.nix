{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    zephyr-nix.url = "github:nix-community/zephyr-nix";
  };

  outputs = { nixpkgs, zephyr-nix, ... }: 
  let
    pkgs = nixpkgs.legacyPackages.x86_64-linux;
    zephyr = zephyr-nix.packages.x86_64-linux;

    # We define our universal flash command right here in Nix!
    # We define our universal flash command right here in Nix!
    # We define our universal flash command right here in Nix!
    flashNode = pkgs.writeShellScriptBin "flash" ''
      PI_IP="192.168.57.7"
      ELF_FILE="build/zephyr/zephyr.elf"
      
      # Dynamically grab the GDB path using the Zephyr SDK environment variable!
      GDB="$ZEPHYR_SDK_INSTALL_DIR/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb"

      if [ ! -f "$ELF_FILE" ]; then
          echo "Error: $ELF_FILE not found."
          echo "Are you in the right project folder? Did you run 'west build'?"
          exit 1
      fi

      if [ ! -x "$GDB" ]; then
          echo "Error: GDB not found at $GDB"
          exit 1
      fi

      echo "0. Wiping edge node state (Restarting OpenOCD)..."
      echo "shutdown" | nc -w 1 $PI_IP 4444 > /dev/null 2>&1 || true
      sleep 2 

      echo "1. Waking up OpenOCD and forcing hardware reset..."
      cat <<EOF | nc -w 2 $PI_IP 4444
      source [find target/stm32f1x.cfg]
      reset_config none
      init
      reset halt
      exit
      EOF
      
      sleep 1

      echo "2. Beaming Zephyr OS via GDB..."
      $GDB $ELF_FILE -batch \
          -ex "target extended-remote $PI_IP:3333" \
          -ex "load" \
          -ex "set {int}0xE000ED0C = 0x05FA0004" \
          -ex "detach" \
          -ex "quit"

      echo -e "\nDone! Firmware loaded and chip rebooted."
    '';

  in {
    devShells.x86_64-linux.default = pkgs.mkShell {
      packages = [
        zephyr.sdkFull
        zephyr.pythonEnv
        zephyr.hosttools
        pkgs.cmake
        pkgs.ninja
        pkgs.netcat-gnu
        flashNode
      ];
    };
  };
}