@echo off

setlocal
call "%~dp0abspath.bat" "%~dp0.."
call "%~dp0docker-id.bat"
rem ===========
rem Options explained: 
rem detached (-d), remove (-rm) after finished, allocate terminal (-it), expose ports (-p), map volumes from host (-v)
rem ===========
echo docker run --rm -it -p 4567:4567 -p 22:22 --name="%DockerContainerName%" -v "%AbsPath%":/capstone -v "%AbsPath%/.ros":/root/.ros/ -v "%AbsPath%/.ros/log":/root/.ros/log/ %DockerUserName%/%DockerImageName%:%DockerImageTag%
popd

rem Running "bash" command causes the entry point "user/sbin/sshd -D" not run or not run correctly

endlocal
