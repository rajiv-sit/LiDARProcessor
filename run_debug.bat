@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

RMDIR /Q /S build

conan install . -c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=True --output-folder=build --build=missing --settings=build_type=Debug
cmake -S . -B build -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW
cmake --build build --config Debug
mkdir build\Debug\shaders 1>NUL 2>NUL
robocopy shaders build\Debug\shaders /E /z
robocopy visualization build\Debug imgui.ini /z
mkdir build\Debug\data 1>NUL 2>NUL
robocopy data build\Debug\data /E /z
PUSHD build\Debug
LiDARProcessor.exe
POPD

#conan create . -s build_type=Debug

popd
