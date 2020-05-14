@REM Copy opencv DLLs to build folders.

@echo Copying dlls needed by libfranka.

@cd /d "%~dp0"

@call util_copy_file.bat ^
	"..\..\..\libfranka\visual_studio_2019\externals\poco\bin\Debug" ^
	"..\..\build\x64_Debug" ^
	"PocoFoundationd.dll" ^
	"..\..\build\temp\x64_Debug"
@call util_copy_file.bat ^
	"..\..\..\libfranka\visual_studio_2019\externals\poco\bin\Debug" ^
	"..\..\build\x64_Debug" ^
	"PocoNetd.dll" ^
	"..\..\build\temp\x64_Debug"
	
@call util_copy_file.bat ^
	"..\..\..\libfranka\visual_studio_2019\externals\poco\bin\Release" ^
	"..\..\build\x64_Release" ^
	"PocoFoundation.dll" ^
	"..\..\build\temp\x64_Release"
@call util_copy_file.bat ^
	"..\..\..\libfranka\visual_studio_2019\externals\poco\bin\Release" ^
	"..\..\build\x64_Release" ^
	"PocoNet.dll" ^
	"..\..\build\temp\x64_Release"