#!/usr/bin/env bash
# System packages for ArduPilot SITL (subset of install-prereqs-ubuntu.sh for noble). Run as root.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
APT="apt-get -y -o DPkg::Lock::Timeout=7200"
$APT update
$APT install build-essential ccache g++ gawk git make wget valgrind screen python3-pexpect \
  python3-dev python3-venv xterm libcsfml-dev libsfml-dev \
  libcsfml-audio2.6 libcsfml-graphics2.6 libcsfml-network2.6 libcsfml-system2.6 libcsfml-window2.6 \
  libsfml-audio2.6 libsfml-graphics2.6 libsfml-network2.6 libsfml-system2.6 libsfml-window2.6 \
  libxml2-dev libxslt1-dev python3-wxgtk4.0 libgtk-3-dev
echo "ARDUPILOT ROOT PREREQS DONE"
