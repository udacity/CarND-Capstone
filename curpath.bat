@echo off

setlocal
set "CurPath=%~dp0"
set "CurPath=%CurPath:~0,-1%"
set "CurPath=%CurPath:\=/%"
endlocal & set "CurPath=%CurPath%"