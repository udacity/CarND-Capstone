# Steering Controller Performance

## Running Performance Test

Run the following command to prepare the test data:

```
make test-init
```

Once done build and run the docker container

```
make build run
```

In the running container start the test node:

```
roslaunch src/twist_controller/launch/dbw_test.launch
```

You may see errors like this:

```
[ERROR] [1550117114.932990800]: Client [/dbw_test] wants topic /actual/brake_cmd to have datatype/md5s│Removing: /Users/nyukhalov/Library/Logs/Homebrew/zsh... (64B)
um [dbw_mkz_msgs/BrakeCmd/899b0f3ef31bf0a48497d65b424a1975], but our version has [dbw_mkz_msgs/BrakeCm│Removing: /Users/nyukhalov/Library/Logs/Homebrew/theora... (64B)
d/c0d20e1056976680942e85ab0959826c]. Dropping connection.
```

The reason of the errors is unknown yet, but looks like we can simply ignore them.

Wait until test node is finished. The following log message indicates that the node finished:

```
[dbw_test-3] process has finished cleanly
```

Now you can exit from the container, we don't need it anymore.

Run the `steering_perf.py` script from the util directory to calculate overall error:

```
python util/steering_perf.py
```

The output will look like this:

```
Steering performance based on 1000 samples = 0.0782663364056498
```

## Perf Tune

### 2019-02-14

Streeting controller from the walkthrough lesson.
The `following_flag` in the `waypoint_follower` is always set to `False` to force the follower to always update twist commands.

Performance: 
- `Steering performance based on 1000 samples = 0.0845631461869925`
- `Steering performance based on 1000 samples = 0.0782663364056498`
- `Steering performance based on 1000 samples = 0.1017662478219718`