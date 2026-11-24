@echo off
cd /d "%~dp0"
echo El robot sigue tu mano. Teclas: q salir, p pausa, c calibrar, h home
python seguir_mano_robot.py %*
pause
