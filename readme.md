# Xsens Tutorial

### 1. Installing Software in Windows

Xsens has two versions of software: 
- Xsens MVN Analyze
- Xsens Animate Pro.

In 2018, the company seems merge the two versions into one called MVN 2018. But now the company seems to separate the versions again, now the suggest software under the offical website is MVN Analyze 2019. 

Download MVN Analyze 2019: https://www.xsens.com/download/MVN/mvn2019/mvn_2019_0_setup.exe 


### 2.Running the Xsens MVN in Windows

- First a valid license (provided by Xsense canpany) is needed before opening the software, the license looks like a USB stick and can be found in the backpack. Insert the license just like a normal USB device.

- Open the software, `click Open -> New Session`, then A window will ask you to fill user's body measurement data. Follow these two videos to create new session: 
- Setup record session: https://tutorial.xsens.com/video/set-up-a-recording-session 
- Measure the body data: https://tutorial.xsens.com/video/body-measurements. 

After filling the data, click OK to create this session. 

### 3.Setting the Xsens MVN Awinda Suit

- To wear Xsens, you need to start with putting the sensors on the shirt (Three different size of cloth, choose one that suit you best). Then place straps and stick the sensor to the right place. There are words on the side of the sensors that tell you where they should be placed. (i.e.  L(Left)/R(Right) Hand, Shoulder, Lower/Upper Leg......)

- Three bases in the backpack. when using Xsens, you only need to use the base with antenna to receive data. The other two bases are for charging. Plugging in the base, and find a cable with USB Type-B port to connect the base with the computer.

These two videos demonstrate the procedures above:
- Putting on the shirt and placing straps: https://tutorial.xsens.com/video/placing-straps 
- Putting the sensors: https://tutorial.xsens.com/video/preparing-hardware-mvn-awinda

After finishing the steps above, A full human skeleton model appears on the software which means you have successfully set Xsens.

- The last step is for calibration: https://tutorial.xsens.com/video/calibrating-awinda 

### 4.Transferring Data to Ubuntu Linux

- The communication between Window and Linux through UDP protocol. After receiving data from Windows, every IMU sensor will be published as tf tree in the Ubuntu side. Two computers are needed, one is running Xsens MVN 2018, and the other is running Ubuntu Linux. Two computers must connect to the same WIFI or it is better to use a cable to connect to the same router which is more stable.

- git clone this repository to your src folder: 
```
$ git clone https://github.com/shawn8583/xsens_mvn_deploy.git
```
go to the top of your ros workspace and compile the code

- To receive data successfully, you need to check the IP address using:
```
$ ifconfig
```
- Go back to the computer running Windows, 
`click Options -> Preferences -> Miscellaneous -> Network Streamer ` 

- And do the following steps(Very Important):
```
1) Click “add”

2) Set Protocol to UDP (Only have two options UDP and TCP)

3) Set Port as 9763

4) Set the “Host” as the one we found in Ubuntu using “$ ifconfig”

5) Only select “Position + Orientation (Quaternion)”, DON’T select other types and options.

6) Make sure it’s enabled (select enable).

7) Other stuff can leave as default, it doesn’t matter.
```
- Run the ros package to receive data:
```
$ rosrun xsens_data_tf xsens_data_tf_broadcaster
```
If there’s still a live skeleton figure displaying on the Xsens Software in the computer running Windows, it should be able to receive data in Ubuntu, the terminal also will print out the stream. 


### 5.Visualization in Rviz

- To visualize data received published by rostopic
```
$ rosrun rviz rviz
```
Then,
```
1) add a TF

2) set “Fixed Frame” to “body_sensor”
```

The rviz should be able to visualize a live skeleton now.


End of this tutorial, thanks!

Wish this will be helpful : )


If you have any question, please contact:
```
Shawn: Shawn8583@163.com
Tony: 470963660@qq.com

Contributor: Shawn and Tony
```
