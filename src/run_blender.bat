@echo off
echo "--- Running Blender Script ---"

:: Укажи свой путь к blender.exe
set BLENDER_PATH="C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"

:: Команда для запуска. -b - в фоновом режиме, -P - выполнить Python-скрипт
%BLENDER_PATH% -P trash_anim.py

echo "--- Script finished ---"
pause