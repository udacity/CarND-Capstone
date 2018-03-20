@echo off

setlocal
call "%~dp0curpath.bat"
call "%~dp0docker-id.bat"
docker build --tag=%DockerUserName%/%DockerImageName%:%DockerImageTag% --file=%curpath%/Dockerfile %curpath%
endlocal