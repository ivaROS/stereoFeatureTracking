echo "Building stvo/3rdparty/line_descriptor ... "
cd stvo/3rdparty/line_descriptor
rm -rf build/ lib/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../../../

echo "Building stvo/3rdparty/DBoW2 ... "
cd stvo/3rdparty/DBoW2
rm -rf build/ lib/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../../../

echo "Building stvo ... "
cd stvo
rm -rf build/ lib/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../
