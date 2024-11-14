sudo apt install libeigen3-dev libopenblas-dev libgmp-dev libmpfr-dev

cd ./3rd/SuiteSparse-7.8.3/

rm -rf build

mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=./install ..
make -j
make install
