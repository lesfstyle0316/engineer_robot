param(
    [ValidateSet('Debug','Release')][string]$Configuration='Debug',
    [string]$CubeCltRoot='D:\xx\dk\cubeclt\STM32CubeCLT_1.22.0'
)
$ErrorActionPreference='Stop'
$cmake=Join-Path $CubeCltRoot 'CMake\bin\cmake.exe'
$ninja=Join-Path $CubeCltRoot 'Ninja\bin\ninja.exe'
$gccDirectory=Join-Path $CubeCltRoot 'GNU-tools-for-STM32\bin'
foreach($required in @($cmake,$ninja,(Join-Path $gccDirectory 'arm-none-eabi-gcc.exe'))) {
    if(-not (Test-Path -LiteralPath $required)) { throw "Missing tool: $required" }
}
$env:PATH="$gccDirectory;$env:PATH"
$buildDirectory=Join-Path $PSScriptRoot "build\$Configuration"
$toolchain=Join-Path $PSScriptRoot 'cmake\gcc-arm-none-eabi.cmake'
& $cmake -S $PSScriptRoot -B $buildDirectory -G Ninja "-DCMAKE_TOOLCHAIN_FILE=$toolchain" "-DCMAKE_MAKE_PROGRAM=$ninja" "-DCMAKE_BUILD_TYPE=$Configuration"
if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $cmake --build $buildDirectory -j 6
exit $LASTEXITCODE
