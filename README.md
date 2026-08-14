# andROSid

Turns an Android phone into a ROS 2 sensor source: camera, IMU and GPS, all on one
clock, streamed over a localhost socket into a ROS 2 node running in Termux/proot.

The app exists because a proot container cannot reach Android's camera, sensor or
location HALs. Going through an app also allows hardware timestamps, taken at the
sensor rather than at the moment a pipe happened to be scheduled.

## Building the app

Open the `andROSid/android` folder in Android Studio and hit `Run` with the phone connected
over USB debugging. Ensure your Android device has Developer Options unlocked and
USB debugging enabled there.

Then launch the app and grant camera + location + notifications. There is no UI:
opening the app starts streaming, closing it stops. The notification is your
indicator that it's alive.

## Setting up a ROS 2 Environment on your Device

Install [Termux](https://f-droid.org/packages/com.termux/) app on your android device, and inside it, set up [`proot-distro`](https://github.com/termux/proot-distro):

```bash
pkg install proot-distro
```

Download the ROS 2 image of your preference (this was tested on Jazzy) and access the
container:

```bash
proot-distro install ros:jazzy-ros-base --override-alias ros-jazzy
proot-distro login ros-jazzy
```

On the container, clone this repo and build the package:

```bash
mkdir src && cd src && git clone https://github.com/loolirer/andROSid.git && cd ..
```

Then build the package and setup the environment:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select android_bridge
source install/setup.bash
```

And run the bridge.

```bash
ros2 run android_bridge mobile_sensors
```

## Usage

If everything is working correctly, the bridge node should be publishing on the
topics below:

- `/imu/data_raw`
- `/imu/mag`
- `/gps/fix`
- `/camera/image_raw/compressed`