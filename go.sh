#!/bin/bash
cd `dirname $0`

rm -rf build

# We're done!
echo Cleaned up the project!

mkdir -p build && cd build
cmake .. 
make -j `nproc` $*

echo Running the project!
./mpc
