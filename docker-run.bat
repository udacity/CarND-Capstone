@echo off

setlocal
call "%~dp0curpath.bat"
call "%~dp0docker-id.bat"
rem ===========
rem Options explained: 
rem detached (-d), remove (-rm) after finished, allocate terminal (-it), expose ports (-p), map volumes from host (-v)
rem ===========
docker run -d --rm -it -p 4567:4567 -p 22:22 --name="%DockerContainerName%" -v "%CurPath%":/capstone -v "%CurPath%/.ros":/root/.ros/ -v "%CurPath%/.ros/log":/root/.ros/log/ %DockerUserName%/%DockerImageName%:%DockerImageTag%

rem Running "bash" command causes the entry point "user/sbin/sshd -D" not run or not run correctly

endlocal
