#Waypoints
- No Carla specific instructions

#DBW 
- For DBW, it is suggested that we keep the control commands frequency at 50hz
- Carla has an Automatic transmission, so a force of 700Nm needs to be applied to keep it still 

#Detection
- Carla uses Tensorflow 1.3.0 (need to make sure it works on that)
- The trafic lights used on the Carla test track are different looking than the ones in used in the training/simulation, so we might need a separate model.


#Some other clarifications

When we start the simulator, there are 2 options: the “highway” and the real test track. The real test track is basically a copy of Udacity’s test track, which in reality is really a small parking lot (nothing fancy).

It seems like the parking lot model is actually built from LIDAR data, that’s why it looks so weird.

The acual car seems like it will just go in circles around the test track and there is a traffic light somewhere in the middle.

The important thing to remember is that the traffic lights used in the real test track is different than on the simulator, so we might need to test a different model.

#General flow:

- We program and use the simulator to make sure the car follows waypoints and obeys speed limit and responds to traffic lights
- Once we submit, our code will get tested by Udacity, first on the simulator, and, if that passes, it will get tested with the real car.
- Following the test on the real car, we will get data in the form of BAG file/archive
- Using the BAG file, we can use the simulator to “replay” our performance. This way we will get excellent feedback loop and can fix potential bugs until it works.

#Example of Test Track traffic light
![Test Track Traffic Light](test_track_trafficlight.png)
