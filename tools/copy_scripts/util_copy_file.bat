@REM Copy file ($3) from a source (%1) to a destination folder (%2).
@REM Log robocopy output in a temp folder ($4) so as not to spam
@REM the VisualStudio console.
@REM 
@REM Usage notes:
@REM - We create destination and temp folders if these do not exist.
@REM - We expect all folders relative to our batch script.
@REM     Relative folders must not contain any whitespace.
@REM - We overwrite but never delete destination files (i.e. no /mir).
@REM     This avoids tragic accidents with corrupt input paths.
@REM     To purge output, one should manually delete build folders,
@REM     then rerun either a VisualStudio build or all copy batches.

@cd /d "%~dp0"
@if not exist "%4" mkdir "%4"
@robocopy "%1" "%2" "%3" /e > "%4\%3_copy_log.txt"


@REM On success and error, robocopy sets unusual error codes
@REM that represent a bitmask of various result states.
@REM This breaks VisualStudio builds, so instead we consume
@REM error codes here, and emit specially-formatted warning
@REM and error messages that the IDE will catch on execution.

@if %errorlevel% geq 4 goto fail
@if %errorlevel% equ 3 goto copy
@if %errorlevel% equ 1 goto copy
@exit /b 0


:fail
@echo ERROR: Failed to copy assets from: "%1" to: "%2"
@echo     Check log for details: "%4\%3_copy_log.txt"
@echo     Solve, then restart VisualStudio build or asset copy batch.
@exit /b 0

:copy
@echo Successfully copied new files to: "%2"
@exit /b 0
