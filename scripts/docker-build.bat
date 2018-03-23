@echo off

setlocal
call "%~dp0abspath.bat" "%~dp0.."
call "%~dp0docker-id.bat"
docker build --tag=%DockerUserName%/%DockerImageName%:%DockerImageTag% --file="%AbsPath%/Dockerfile" "%AbsPath%"
endlocal