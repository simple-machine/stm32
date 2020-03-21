with import <nixpkgs> {};
{ unstable ? import <nixos-unstable> {} }:
let
 python-packages = python-packages: with python-packages; [
  pythonPackages.pyserial
 ];
 python-with-packages = python3.withPackages python-packages;
in
stdenv.mkDerivation {
  name = "env";
  buildInputs = [
    openocd gcc-arm-embedded dfu-util usbutils stlink brltty busybox screen usbtop
    evemu input-utils python-with-packages arduino
  ];
}

