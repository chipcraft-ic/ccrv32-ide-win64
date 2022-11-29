@echo off

rem Defaults
set "CHIPCRAFT_SDK_BOARD=sim"
set "CHIPCRAFT_SDK_DBG_PORT="
set "CHIPCRAFT_SDK_UART_PORT="

rem Override defaults using last used values
if exist "%LOCALAPPDATA%/ccenv.cmd" (
  "%LOCALAPPDATA%/ccenv.cmd"
)

set "CHIPCRAFT_SDK_HOME=%CD%"

echo Available boards:
dir /b "%CHIPCRAFT_SDK_HOME%\boards"

set /p CHIPCRAFT_SDK_BOARD="Board [%CHIPCRAFT_SDK_BOARD%]: "

echo Available ports:
chgport

set /p CHIPCRAFT_SDK_DBG_PORT="Debug Port (e.g. COMn) [%CHIPCRAFT_SDK_DBG_PORT%]: "
set /p CHIPCRAFT_SDK_UART_PORT="UART Port (e.g. COMn) [%CHIPCRAFT_SDK_UART_PORT%]: "

rem Save settings
echo "set CHIPCRAFT_SDK_BOARD=%CHIPCRAFT_SDK_BOARD%" >"%LOCALAPPDATA%/ccenv.cmd"
echo "set CHIPCRAFT_SDK_DBG_PORT=%CHIPCRAFT_SDK_DBG_PORT%" >>"%LOCALAPPDATA%/ccenv.cmd"
echo "set CHIPCRAFT_SDK_UART_PORT=%CHIPCRAFT_SDK_UART_PORT%" >>"%LOCALAPPDATA%/ccenv.cmd"

set PATH=%PATH%;%CHIPCRAFT_SDK_TOOLCHAIN_PATH%;%CHIPCRAFT_SDK_HOME%/tools/srecord;%CHIPCRAFT_SDK_HOME%/tools/resources/windows/srecord
cmd
