param([string]$Gcc='D:\xx\dk\CLion 2026.2.1\bin\mingw\bin\gcc.exe')
$ErrorActionPreference='Stop'
if(-not (Test-Path -LiteralPath $Gcc)) { throw "Native Windows GCC not found: $Gcc" }
$testDirectory=Join-Path $PSScriptRoot 'build\HostTests'
New-Item -ItemType Directory -Force -Path $testDirectory | Out-Null
$sourceFiles=@('user_driver/src/robot_config.c','user_driver/src/drv_rc.c','user_driver/src/drv_motor.c','user_driver/src/drv_pid.c','user_driver/src/drv_power.c','task/src/robot_core.c','task/src/robot_selftest.c')
$sourceFiles=@($sourceFiles | ForEach-Object { Join-Path $PSScriptRoot $_ })
$testExecutable=Join-Path $testDirectory 'robot_selftest.exe'
$compilerArguments=@('-std=c11','-O2','-Wall','-Wextra','-Werror',"-I$PSScriptRoot/user_driver/inc","-I$PSScriptRoot/task/inc")+$sourceFiles+@('-o',$testExecutable,'-lm')
& $Gcc @compilerArguments
if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $testExecutable
if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$gpp=Join-Path (Split-Path -Parent $Gcc) 'g++.exe'
$cObjectSources=@('user_driver/src/robot_config.c','user_driver/src/drv_motor.c','user_driver/src/drv_power.c','user_driver/src/drv_pid.c','user_driver/src/drv_rc.c','task/src/robot_core.c')
$cObjects=@()
foreach($source in $cObjectSources) {
    $object=Join-Path $testDirectory (([IO.Path]::GetFileNameWithoutExtension($source))+'.obj')
    & $Gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$PSScriptRoot/user_driver/inc" "-I$PSScriptRoot/task/inc" -c (Join-Path $PSScriptRoot $source) -o $object
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    $cObjects+=$object
}
$cppSources=@('bsp/src/can_device.cpp','user_driver/src/DJI_motor_driver.cpp','user_driver/src/DM_motor_driver.cpp','user_driver/src/motor_system.cpp','task/src/can_motor_selftest.cpp')
$cppSources=@($cppSources | ForEach-Object { Join-Path $PSScriptRoot $_ })
$cppExecutable=Join-Path $testDirectory 'can_motor_selftest.exe'
$cppArguments=@('-std=c++17','-O2','-Wall','-Wextra','-Werror',"-I$PSScriptRoot/bsp/inc","-I$PSScriptRoot/user_driver/inc","-I$PSScriptRoot/task/inc")+$cppSources+$cObjects+@('-o',$cppExecutable,'-lm')
& $gpp @cppArguments
if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $cppExecutable
exit $LASTEXITCODE
