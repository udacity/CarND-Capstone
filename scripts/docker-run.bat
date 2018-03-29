@echo off

setlocal
call "%~dp0abspath.bat" "%~dp0.."
call "%~dp0docker-id.bat"

docker run --rm -it -p 4567:4567 --name="%DockerContainerName%" -v %AbsPath%:/capstone -v %AbsPath%/.ros:/root/.ros/ %DockerUserName%/%DockerImageName%:%DockerImageTag%

endlocal
