@echo off

REM //---------- set up variable ----------
setlocal
call :setESC
set ROOT_DIR=%~dp0


REM // ------------  check VS version ------------ 
CALL :printHeader, "Check Visual Studio Version"
echo Visual studio version: %VisualStudioVersion%
if "%VisualStudioVersion%" == "" (
    call :printError, "uh oh... You need to run this command from x64 Native Tools Command Prompt for VS 2022"
    goto :buildfailed_nomsg
)
if "%VisualStudioVersion%" lss "17.0" (
    call :printError, "Hello there! We just upgraded AirSim to Unreal Engine 4.27 and Visual Studio 2022. Here are few easy steps for upgrade so everything is new and shiny:  https://github.com/Microsoft/AirSim/blob/main/docs/unreal_upgrade.md"
    goto :buildfailed_nomsg
)
ECHO(

REM // ------------  Parse command line arguments ------------ 
CALL :printHeader, "Parse command line arguments"

set noFullPolyCar=""
set buildMode=""

:loop
IF NOT "%1"=="" (
    IF "%1"=="--Debug" (
        set "buildMode=Debug"
        SHIFT
    ) ELSE IF "%1"=="--Release" (
        set "buildMode=Release"
        SHIFT
    ) ELSE IF "%1"=="--no-full-poly-car" (
        set "noFullPolyCar=y"
        SHIFT
    ) ELSE IF "%1"=="--RelWithDebInfo" (
        set "buildMode=RelWithDebInfo"
        SHIFT
    ) ELSE (
        echo Unknown command line parameter: %1
        SHIFT
    )
    GOTO :loop
)
echo buildMode = %buildMode%
echo noFullPolyCar = %noFullPolyCar%
ECHO(

REM // ------------ Check for powershell ------------ 
CALL :printHeader, "Check for powershell"

set powershell=powershell
where powershell > nul 2>&1
if ERRORLEVEL 1 goto :pwsh
echo found Powershell && goto start
:pwsh
set powershell=pwsh
where pwsh > nul 2>&1
if ERRORLEVEL 1 goto :nopwsh
set PWSHV7=1
echo found pwsh && goto start

:nopwsh
echo Powershell or pwsh not found, please install it.
goto :eof

:start
ECHO( 

chdir /d %ROOT_DIR% 

REM //---------- Check cmake version ----------
CALL :printHeader, "Check cmake version"
CALL check_cmake.bat
if ERRORLEVEL 1 (
  CALL check_cmake.bat
  if ERRORLEVEL 1 (
    echo ERROR: cmake was not installed correctly, we tried.
    goto :buildfailed
  )
)
ECHO(

REM //---------- get rpclib ----------

IF NOT EXIST external\rpclib mkdir external\rpclib

set RPC_VERSION_FOLDER=rpclib-2.3.1
IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER% (
    REM //leave some blank lines because %powershell% shows download banner at top of console
    ECHO(
    ECHO(   
    ECHO(   
    CALL :printHeader, "Downloading rpclib"
    @echo on
    if "%PWSHV7%" == "" (
        %powershell% -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://github.com/WouterJansen/rpclib/archive/refs/tags/v2.3.1.zip -OutFile external\rpclib.zip }"
    ) else (
        %powershell% -command "iwr https://github.com/WouterJansen/rpclib/archive/refs/tags/v2.3.1.zip -OutFile external\rpclib.zip"
    )
    @echo off
    
    REM //remove any previous versions
    rmdir external\rpclib /q /s

    %powershell% -command "Expand-Archive -Path external\rpclib.zip -DestinationPath external\rpclib"
    del external\rpclib.zip /q
    
    REM //Fail the build if unable to download rpclib
    IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER% (
        ECHO Unable to download rpclib, stopping build
        goto :buildfailed
    )

    ECHO(
)

REM //---------- Build rpclib ------------
IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER%\build mkdir external\rpclib\%RPC_VERSION_FOLDER%\build
cd external\rpclib\%RPC_VERSION_FOLDER%\build
CALL :printHeader, "Configuring CMake rpclib"
cmake -G"Visual Studio 17 2022" ..
ECHO(

if %buildMode% == "" (
    CALL :printHeader, "Building rpclib - Configuration = Release"
    cmake --build . --config Release
    ECHO(

    CALL :printHeader, "Building rpclib - Configuration = Debug"
    cmake --build . --config Debug
    ECHO(

    CALL :printHeader, "Building rpclib - Configuration = RelWithDebInfo"
    cmake --build . --config RelWithDebInfo
    ECHO(
) else (
    CALL :printHeader, "Building rpclib - Configuration = %buildMode%"
    cmake --build . --config %buildMode%
    ECHO(
)

if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 
ECHO(   

REM //---------- copy rpclib binaries and include folder inside AirLib folder ----------
CALL :printHeader, "Copy rpclib lib and include files to AirLib folder structure"
set RPCLIB_TARGET_LIB=AirLib\deps\rpclib\lib\x64
if NOT exist %RPCLIB_TARGET_LIB% mkdir %RPCLIB_TARGET_LIB%
set RPCLIB_TARGET_INCLUDE=AirLib\deps\rpclib\include
if NOT exist %RPCLIB_TARGET_INCLUDE% mkdir %RPCLIB_TARGET_INCLUDE%
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\include %RPCLIB_TARGET_INCLUDE%

if %buildMode% == "" (
    @echo on
    robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\Debug %RPCLIB_TARGET_LIB%\Debug
    robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\Release %RPCLIB_TARGET_LIB%\Release
    robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\RelWithDebInfo %RPCLIB_TARGET_LIB%\RelWithDebInfo
    @echo off
) else (
    @echo on
    robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\%buildMode% %RPCLIB_TARGET_LIB%\%buildMode%
    @echo off
)
ECHO(   


REM //---------- get High PolyCount SUV Car Model ------------
CALL :printHeader, "Configure High Polycount SUV car model"

IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv mkdir Unreal\Plugins\AirSim\Content\VehicleAdv
IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv\SUV\v1.2.0 (
    IF NOT DEFINED noFullPolyCar (
        REM //leave some blank lines because %powershell% shows download banner at top of console
        ECHO(   
        ECHO(   
        ECHO(   
        ECHO "Downloading high-poly car assets. To install without this assets, re-run build.cmd with the argument --no-full-poly-car"

        IF EXIST suv_download_tmp rmdir suv_download_tmp /q /s
        mkdir suv_download_tmp
        @echo on
        REM %powershell% -command "& { Start-BitsTransfer -Source https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip -Destination suv_download_tmp\car_assets.zip }"
        REM %powershell% -command "& { (New-Object System.Net.WebClient).DownloadFile('https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip', 'suv_download_tmp\car_assets.zip') }"
        if "%PWSHV7%" == "" (
            %powershell% -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://github.com/CodexLabsLLC/Colosseum/releases/download/v2.0.0-beta.0/car_assets.zip -OutFile suv_download_tmp\car_assets.zip }"
        ) else (
            %powershell% -command "iwr https://github.com/CodexLabsLLC/Colosseum/releases/download/v2.0.0-beta.0/car_assets.zip -OutFile suv_download_tmp\car_assets.zip"
        )
        @echo off
        rmdir /S /Q Unreal\Plugins\AirSim\Content\VehicleAdv\SUV
        %powershell% -command "Expand-Archive -Path suv_download_tmp\car_assets.zip -DestinationPath Unreal\Plugins\AirSim\Content\VehicleAdv"
        rmdir suv_download_tmp /q /s
        
        REM //Don't fail the build if the high-poly car is unable to be downloaded
        REM //Instead, just notify users that the gokart will be used.
        IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv\SUV ECHO Unable to download high-polycount SUV. Your AirSim build will use the default vehicle.
    ) else (
        ECHO Not downloading high-poly car asset. The default unreal vehicle will be used.
    )
)
ECHO(

REM //---------- setup Eigen dependency for AirLib ----------
CALL :printHeader, "Setup Eigen dependency for AirLib "
IF NOT EXIST AirLib\deps mkdir AirLib\deps
IF NOT EXIST ExternalRepositories\Colosseum_Eigen\Eigen (
    ECHO Submodule ExternalRepositories\Colosseum_Eigen not properly initialized. Try running 'git submodule update --init'
    goto :buildfailed 
)
IF NOT EXIST AirLib\deps\eigen3 (
    mkdir AirLib\deps\eigen3
    robocopy /MIR ExternalRepositories\Colosseum_Eigen\Eigen AirLib\deps\eigen3\Eigen
)

IF NOT EXIST AirLib\deps\eigen3 goto :buildfailed
ECHO(

REM //---------- now we have all dependencies to compile AirSim.sln which will also compile MavLinkCom ----------
if %buildMode% == "" (
    CALL :printHeader, "Building AirSim.sln Configuration = Debug"
    msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Debug AirSim.sln
    ECHO(   
    if ERRORLEVEL 1 goto :buildfailed

    CALL :printHeader, "Building AirSim.sln Configuration = Release"
    msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Release AirSim.sln 
    ECHO(   
    if ERRORLEVEL 1 goto :buildfailed

    CALL :printHeader, "Building AirSim.sln Configuration = RelWithDebInfo"
    msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=RelWithDebInfo AirSim.sln 
    ECHO(   
    if ERRORLEVEL 1 goto :buildfailed
) else (
    CALL :printHeader, "Building AirSim.sln Configuration = %buildMode%"
    msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=%buildMode% AirSim.sln
    ECHO(   
    if ERRORLEVEL 1 goto :buildfailed
)

REM //---------- copy binaries and include for MavLinkCom in deps ----------
CALL :printHeader, "Copy MavLinkCom lib and include files into AirLib folder structure"
set MAVLINK_TARGET_LIB=AirLib\deps\MavLinkCom\lib
if NOT exist %MAVLINK_TARGET_LIB% mkdir %MAVLINK_TARGET_LIB%
set MAVLINK_TARGET_INCLUDE=AirLib\deps\MavLinkCom\include
if NOT exist %MAVLINK_TARGET_INCLUDE% mkdir %MAVLINK_TARGET_INCLUDE%
robocopy /MIR MavLinkCom\include %MAVLINK_TARGET_INCLUDE%
robocopy /MIR MavLinkCom\lib %MAVLINK_TARGET_LIB%
ECHO(

REM //---------- all our output goes to Unreal/Plugin folder ----------
CALL :printHeader, "Copy Airlib files into Unreal environment and plugin folder structure"
if NOT exist Unreal\Plugins\AirSim\Source\AirLib mkdir Unreal\Plugins\AirSim\Source\AirLib
robocopy /MIR AirLib Unreal\Plugins\AirSim\Source\AirLib  /XD temp *. /njh /njs /ndl /np
copy /y AirSim.props Unreal\Plugins\AirSim\Source\AirLib
ECHO( 

REM //---------- update all environments ----------
CALL :printHeader, "Update all Unreal environments"
FOR /D %%E IN (Unreal\Environments\*) DO (
    cd %%E
    call .\update_from_git.bat ..\..\..
    cd ..\..\..
)

REM //---------- done building ----------
exit /b 0

:buildfailed
CALL :printHeader, "Build failed!"
echo Build failed - see messages above. 1>&2

:buildfailed_nomsg
chdir /d %ROOT_DIR% 
exit /b 1

:setESC
for /F "tokens=1,2 delims=#" %%a in ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') do (
  set ESC=%%b
  exit /B 0
)
exit /B 0

:printHeader
REM //---------- PrintHeader function ----------
if not "%~1"=="" (
    ECHO %ESC%[33m*****************************************************************************************%ESC%[0m
    ECHO %ESC%[33m%~1%ESC%%[0
    ECHO %ESC%[33m*****************************************************************************************%ESC%%[0m
)
exit /b 0

:printError
REM //---------- PrintError function ----------
if not "%~1"=="" (
REM    ECHO %ESC%[31m*****************************************************************************************%ESC%[0m
    ECHO %ESC%[31m%~1%ESC%%[0
REM  ECHO %ESC%[31m*****************************************************************************************%ESC%%[0m
)
exit /b 0
