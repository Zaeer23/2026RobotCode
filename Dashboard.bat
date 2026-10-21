@echo off
REM ===========================================================================
REM  Driver station dashboard
REM
REM  Runs on THIS computer. The roboRIO is headless, so the GUI lives here and
REM  talks to the robot over NetworkTables, the same way Shuffleboard does.
REM
REM  Launches detached with javaw, so it does NOT hold this window open and
REM  does NOT look like a stuck gradle build.
REM
REM  Optional team number:   Dashboard.bat 7064
REM  Simulated robot:        Dashboard.bat localhost
REM ===========================================================================
setlocal
cd /d "%~dp0"

REM ---- Find Java -------------------------------------------------------------
REM WPILib ships its own JDK. Prefer this season's, then any other season's,
REM then whatever Java this computer already has (JAVA_HOME, then PATH).
set "JDK="
if exist "C:\Users\Public\wpilib\2026\jdk\bin\javaw.exe" set "JDK=C:\Users\Public\wpilib\2026\jdk"
if not defined JDK for /d %%Y in ("C:\Users\Public\wpilib\*") do if exist "%%~Y\jdk\bin\javaw.exe" set "JDK=%%~Y\jdk"
if not defined JDK if defined JAVA_HOME if exist "%JAVA_HOME%\bin\javaw.exe" set "JDK=%JAVA_HOME%"

set "JAVAW="
if defined JDK (
  set "JAVA_HOME=%JDK%"
  set "JAVAW=%JDK%\bin\javaw.exe"
) else (
  for /f "delims=" %%J in ('where javaw 2^>nul') do if not defined JAVAW set "JAVAW=%%J"
)

if not defined JAVAW (
  echo.
  echo   The dashboard could not find Java on this computer.
  echo.
  echo   It needs Java 17 or newer. The easiest way to get it is the WPILib 2026
  echo   installer, which also puts everything needed to deploy robot code here:
  echo.
  echo       https://github.com/wpilibsuite/allwpilib/releases
  echo.
  echo   Any other Java 17+ install works too, such as Eclipse Temurin.
  echo   Then run Dashboard.bat again.
  echo.
  pause
  exit /b 1
)

REM ---- Build once, then launch ------------------------------------------------
if not exist "build\dashboard\dashboard.jar" (
  echo Building the dashboard, one moment...
  call gradlew.bat dashboardBundle
  if errorlevel 1 (
    echo.
    echo   The dashboard did not build. The error is above.
    pause
    exit /b 1
  )
)

start "" "%JAVAW%" -Djava.library.path=build\dashboard\natives -jar build\dashboard\dashboard.jar %1
exit
