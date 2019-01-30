#!/bin/sh

MAKEOPTS="-j2"

echo "Changing into simulator directory"
cd ./pyrosim/simulator/external

echo "Unpacking ode-0.12.tar.bz2..." &&
tar -xjf ode-0.12.tar.bz2 &&
echo "done" &&

echo "Building ode-0.12..." &&
cd ode-0.12 &&
./configure --enable-double-precision &&
make $MAKEOPTS
cd ../.. &&
echo "done" &&

echo  "Building simulator..." &&
make $MAKEOPTS
echo "done"
