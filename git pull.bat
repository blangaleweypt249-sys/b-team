@echo off
chcp 65001 >nul
title Git 强制同步远程代码


set REPO_PATH=D:\Project\my_project
set BRANCH=main


echo ==========================
echo 强制同步远程 %BRANCH%
echo ==========================


cd /d %REPO_PATH%


if errorlevel 1 (
    echo 路径不存在
    pause
    exit /b
)


echo.
echo [1/4] 获取远程更新

git fetch origin


if errorlevel 1 (
    echo fetch失败
    pause
    exit /b
)


echo.
echo [2/4] 切换分支

git checkout %BRANCH%


if errorlevel 1 (
    echo 分支不存在
    pause
    exit /b
)


echo.
echo [3/4] 强制覆盖本地

git reset --hard origin/%BRANCH%


if errorlevel 1 (
    echo reset失败
    pause
    exit /b
)




echo.
echo ==========================
echo 同步完成
echo 当前版本:
git log -1 --oneline

echo ==========================

pause