@echo off

rmdir /s /q .\build\

mkdir build

cd build

cmake -G "MinGW Makefiles" -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ ..

mingw32-make -j4

move .\raytracing.exe ..

cd ..

set "filename=instructions.txt"

(
echo. 
echo +--------------+
echo ^| Instructions ^|
echo +--------------+
echo.
echo Usage: ./raytracing "(x0,y0,z0) (x1,y1,z1) (x2,y2,z2)" "(r,g,b)"
echo Example: ./raytracing "(-1.0,0.0,0.0) (1.0,0.0,0.0) (-1.0,1.0,0.0)" "(1.0,0.7,0.0)"
echo Every component of the ^(r,g,b^)^ triplet must be normalized [0.0,1.0].
echo.
) > "%filename%"

type "%filename%"

del "%filename%"

pause