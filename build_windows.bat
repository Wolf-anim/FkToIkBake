@echo off
rem ---------------------------------------------------------------------
rem  FkToIkBake - build the Maya 2027 plugin (.mll) on Windows.
rem  Double-click this file, or:  build_windows.bat "C:\path\to\devkitBase"
rem
rem  Needs: Visual Studio 2022 or newer with "Desktop development with C++"
rem         (its bundled CMake is used automatically), and the Maya 2027
rem         devkit unzipped either next to this file as "devkitBase",
rem         at C:\devkitBase, or passed as the first argument.
rem  Result: package\FkToIkBake\plug-ins\win64\2027\fkToIkBake.mll
rem ---------------------------------------------------------------------
setlocal EnableDelayedExpansion
set "ROOT=%~dp0"
set "LOG=%ROOT%build_log.txt"

rem ---- locate the Maya devkit
set "DEVKIT=%~1"
if "%DEVKIT%"=="" for %%D in ("%ROOT%devkitBase" "%ROOT%devkit\devkitBase" "C:\devkitBase" "C:\Autodesk\devkitBase") do (
    if "!DEVKIT!"=="" if exist "%%~D\include\maya\MFnPlugin.h" set "DEVKIT=%%~D"
)
if "%DEVKIT%"=="" if exist "C:\Program Files\Autodesk\Maya2027\include\maya\MFnPlugin.h" set "DEVKIT=C:\Program Files\Autodesk\Maya2027"
if "%DEVKIT%"=="" goto :nodevkit
echo NO_DEVKIT > "%LOG%"
if not exist "%DEVKIT%\include\maya\MFnPlugin.h" goto :nodevkit
echo NO_DEVKIT > "%LOG%"
echo [FkToIkBake] Maya devkit : %DEVKIT%

rem ---- locate Visual Studio + its CMake
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :novs
echo NO_VS > "%LOG%"
set "VSDIR="
set "VSVER="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion`) do set "VSVER=%%i"
if "%VSDIR%"=="" goto :novs
echo NO_VS > "%LOG%"
set "VSMAJOR=%VSVER:~0,2%"
set "GEN=Visual Studio 17 2022"
if "%VSMAJOR%"=="18" set "GEN=Visual Studio 18 2026"
echo [FkToIkBake] Visual Studio: %VSDIR% (%GEN%)

rem Prefer the CMake bundled with this Visual Studio (knows its generator).
set "CMAKE=%VSDIR%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if not exist "%CMAKE%" set "CMAKE=cmake"
if "%CMAKE%"=="cmake" where cmake >nul 2>nul || goto :nocmake
echo NO_CMAKE > "%LOG%"

set "MAYALOC=C:\Program Files\Autodesk\Maya2027"

echo [FkToIkBake] Configuring... (log: %LOG%)
"%CMAKE%" -S "%ROOT%." -B "%ROOT%build" -G "%GEN%" -A x64 -DMAYA_DEVKIT="%DEVKIT%" -DMAYA_LOCATION="%MAYALOC%" > "%LOG%" 2>&1 || goto :failed
echo [FkToIkBake] Building...
"%CMAKE%" --build "%ROOT%build" --config Release >> "%LOG%" 2>&1 || goto :failed

if not exist "%ROOT%package\FkToIkBake\plug-ins\win64\2027\fkToIkBake.mll" goto :failed
echo.
echo [FkToIkBake] Running engine tests...
"%ROOT%build\Release\fk2ik_tests.exe" >> "%LOG%" 2>&1 && (echo [FkToIkBake] Engine tests PASSED) || (echo [FkToIkBake] Engine tests FAILED - see build_log.txt)
echo.
echo [FkToIkBake] SUCCESS: %ROOT%package\FkToIkBake\plug-ins\win64\2027\fkToIkBake.mll
echo [FkToIkBake] Now drag package\FkToIkBake\install_fkToIkBake.mel into the Maya viewport.
goto :end

:nodevkit
echo NO_DEVKIT > "%LOG%"
echo [FkToIkBake] Maya 2027 devkit not found.
echo   Download "Maya 2027 devkit" (Windows) from the Autodesk developer site,
echo   unzip it so that  "%ROOT%devkitBase\include\maya\MFnPlugin.h"  exists,
echo   then run this file again.
goto :end
:novs
echo NO_VS > "%LOG%"
echo [FkToIkBake] Visual Studio with C++ tools not found.
echo   Install Visual Studio 2022 Community with "Desktop development with C++".
goto :end
:nocmake
echo NO_CMAKE > "%LOG%"
echo [FkToIkBake] CMake not found. In the Visual Studio Installer add "C++ CMake tools for Windows".
goto :end
:failed
echo [FkToIkBake] BUILD FAILED. Send build_log.txt to Claude:
echo   %LOG%
:end
echo.
if "%FK2IK_NOPAUSE%"=="" timeout /t 15
endlocal
