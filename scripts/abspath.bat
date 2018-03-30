@echo off

rem ==============================================
rem Convert given path to absolute path with Unix token separators
rem ==============================================

setlocal
pushd "%~1"
set "AbsPath=%cd%"
set "AbsPath=%AbsPath:\=/%"
popd
endlocal & set "AbsPath=%AbsPath%"