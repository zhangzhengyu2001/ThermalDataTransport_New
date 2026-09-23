@echo off
chcp 65001 >nul
title ThermalDataTransport 后端服务 - 开发模式 (自动重载)
cd /d "%~dp0"

if not exist "C:\Users\Zhang\anaconda3\envs\airctrl\python.exe" (
    echo [错误] 找不到 airctrl 虚拟环境: C:\Users\Zhang\anaconda3\envs\airctrl
    echo 请确认已用 conda 创建 airctrl 环境后重试。
    pause
    exit /b 1
)

echo ================================================================
echo   质谱气压控制后端 - 开发模式（代码改动自动重载）
echo   网页界面: http://127.0.0.1:8000
echo   关闭本窗口即停止服务
echo ================================================================
echo.

rem 延迟 3 秒后自动打开浏览器
start "" /b powershell -NoProfile -WindowStyle Hidden -Command "Start-Sleep -Seconds 3; Start-Process 'http://127.0.0.1:8000'"

"C:\Users\Zhang\anaconda3\envs\airctrl\python.exe" -m uvicorn tools.backend_server:app --reload --host 0.0.0.0 --port 8000

echo.
echo 服务已停止。
pause
