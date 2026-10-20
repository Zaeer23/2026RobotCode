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
REM ===========================================================================
cd /d "%~dp0"
set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk
set JAVAW=%JAVA_HOME%\bin\javaw.exe

if not exist "build\dashboard\dashboard.jar" (
  echo Building the dashboard, one moment...
  call gradlew.bat dashboardBundle
)

start "" "%JAVAW%" -Djava.library.path=build\dashboard\natives -jar build\dashboard\dashboard.jar %1
exit
