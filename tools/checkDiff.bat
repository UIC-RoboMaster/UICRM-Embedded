@echo off
REM 检查输入参数
if "%~2"=="" (
    echo Usage: %~nx0 ^<branch_name^> ^<directory^>
    exit /b 1
)

set "BRANCH_NAME=%~1"
set "DIRECTORY=%~2"

REM 检查是否在 Git 仓库中
git rev-parse --is-inside-work-tree >nul 2>&1
if not "%ERRORLEVEL%"=="0" (
    echo Error: This script must be run inside a Git repository.
    exit /b 1
)

REM 检查目录是否存在
if not exist "%DIRECTORY%" (
    echo Error: Directory "%DIRECTORY%" does not exist.
    exit /b 1
)

REM 检查分支是否存在
git rev-parse --verify "%BRANCH_NAME%" >nul 2>&1
if not "%ERRORLEVEL%"=="0" (
    echo Error: Branch "%BRANCH_NAME%" does not exist.
    exit /b 1
)

REM 查找目录下的所有 .cpp 和 .h 文件，并与指定分支对比
echo Comparing files in directory "%DIRECTORY%" with branch "%BRANCH_NAME%":
for /r "%DIRECTORY%" %%F in (*.cpp *.h) do (
    git diff --quiet "%BRANCH_NAME%" -- "%%F"
    if not "%ERRORLEVEL%"=="0" (
        echo Modified: %%F
    ) else (
        echo No changes: %%F
    )
)
