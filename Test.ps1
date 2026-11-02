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
exit $LASTEXITCODE
