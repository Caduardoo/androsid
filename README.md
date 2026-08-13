# andROSid

Turns an Android phone into a ROS 2 sensor source: camera, IMU and GPS, all on one
clock, streamed over a localhost socket into a ROS 2 node running in Termux/proot.

```
┌─────────────────────────────┐        ┌──────────────────────────┐
│ Android app (Kotlin)        │        │ proot-distro / ros-jazzy │
│                             │  TCP   │                          │
│ SensorManager  ─┐           │ :9870  │  mobile_sensors node     │
│ LocationManager ├─ socket ──┼───────►│    /imu/data_raw         │
│ CameraX         ┘           │        │    /imu/mag              │
│                             │        │    /gps/fix              │
│ foreground service          │        │    /camera/image_raw/... │
└─────────────────────────────┘        └──────────────────────────┘
```

The app exists because a proot container cannot reach Android's camera, sensor or
location HALs — there is no `/dev/video0`, and the sensors are behind Binder, not
sysfs. Going through an app also buys the thing that actually matters for state
estimation: **hardware timestamps**, taken at the sensor rather than at the moment
a pipe happened to be scheduled.

## Layout

```
andROSid/
├── app/                                  Android app
│   └── src/main/java/com/androsid/
│       ├── MainActivity.kt               permission gate + start/stop
│       ├── SensorService.kt              foreground service, IMU + GPS, clock
│       ├── CameraSource.kt               CameraX -> NV21 -> JPEG
│       └── StreamServer.kt               framed TCP broadcast
└── android_bridge/                       ROS 2 package
    └── android_bridge/mobile_sensors.py  socket -> sensor_msgs
```

## Building the app

Open the `andROSid/` folder in Android Studio and hit Run with the phone connected
over USB debugging. Android Studio will offer to upgrade the Gradle wrapper and AGP
version — accept it; the versions pinned here are just a starting point.

There is no `gradle-wrapper.jar` in this tree (it's a binary). Android Studio
generates it on first open. From the CLI instead: `gradle wrapper && ./gradlew installDebug`.

Then launch the app, grant camera + location + notifications, and tap **Start
streaming**. You can close the activity — the foreground service keeps running, and
the notification is your indicator that it's alive.

## Building the ROS 2 side

Inside your proot distro:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws                     # any colcon workspace
cp -r /path/to/andROSid/android_bridge src/
colcon build --packages-select android_bridge
source install/setup.bash
ros2 run android_bridge mobile_sensors
```

Verify:

```bash
ros2 topic hz /imu/data_raw
ros2 topic echo /gps/fix --once
ros2 topic hz /camera/image_raw/compressed
```

Android loopback is not sandboxed between apps, so `127.0.0.1:9870` reaches the app
from inside proot with no special setup. The node reconnects on its own if the app
restarts.

## Wire format

One socket carries everything so that all streams share a single clock. Big-endian,
repeated:

```
[1 byte type][4 bytes payload length][payload]

type 0x01  JSON sensor sample, UTF-8
type 0x02  camera frame: [8 bytes timestamp nanos][JPEG bytes]
```

JSON samples look like `{"s":"gyro","t":<epoch nanos>,"v":[x,y,z]}`, with GPS using
named fields instead of `v`.

## Things you will need to fix for your robot

**Axis convention.** `android_to_flu()` in `mobile_sensors.py` is written for this mount:
phone landscape, rear camera facing forward, buttons up — i.e. rotated
counter-clockwise from portrait as seen facing the screen.

| ROS FLU | Android | why |
|---|---|---|
| forward `x` | `-Z` | rear camera points opposite the screen normal |
| left `y` | `+Y` | top edge of the phone points left |
| up `z` | `+X` | right edge in portrait, where the buttons are |

Android's sensor frame is fixed to the phone's *natural* portrait orientation and
does **not** follow screen rotation, so this holds regardless of what the UI does.

Verify before trusting anything downstream: sitting still, `/imu/data_raw` should
show `linear_acceleration.z ≈ +9.81` and the other two near zero. Pitch the nose
down and `x` should go positive. If you remount the phone, this function is the one
line to change.

**No orientation.** `orientation_covariance[0]` is set to `-1`, the REP-145 way of
saying "there is no orientation estimate in this message". Get one by running:

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args -r imu/data_raw:=/imu/data_raw -r imu/mag:=/imu/mag
```

**No CameraInfo.** Nothing here publishes intrinsics, because publishing a
zero-filled `CameraInfo` is worse than publishing none — downstream nodes will
silently use garbage. Run `camera_calibration` against a checkerboard and add a
`camera_info_manager` publisher once you have a real YAML.

**IMU covariances are guesses.** The values in `_on_imu()` are placeholders. If you
put this into an EKF, measure them properly with a stationary Allan variance run.

**Camera timestamp base.** `ImageProxy.imageInfo.timestamp` shares the boot-relative
base with `SensorEvent.timestamp` only when the device reports
`SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME`. Some devices report `UNKNOWN`, in which case
the camera stamps sit on a different timeline and IMU-camera fusion will be subtly
wrong. Check with a camera2 probe app before trusting VIO.

## Frames

The bridge converts IMU samples into FLU before publishing, so `imu_link` is already
rotationally aligned with `base_link` — only a translation separates them.

The camera needs the standard optical-frame rotation (Z out of the lens, X right,
Y down) from a forward-facing FLU parent:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll -1.5708 --pitch 0 --yaw -1.5708 \
  --frame-id base_link --child-frame-id camera_optical_frame
```

Because the phone is mounted rotated counter-clockwise into landscape, frames may
arrive 180° rotated depending on your sensor's `SENSOR_ORIENTATION`. `CameraSource`
logs `rotationDegrees` once at startup so you can see what the HAL is giving you.

If the image is upside down, prefer fixing it in the transform rather than rotating
pixels — a 180° roll about the optical axis is free in TF and costs a full decode,
rotate and re-encode in the node:

```bash
  --roll 1.5708 --pitch 0 --yaw 1.5708
```

(That is `R_optical · Rz(π)`, not simply `roll + π` — those are different rotations.)
The tradeoff is that images look upside down in rqt while being geometrically correct.

## Tuning

| What | Where | Note |
|---|---|---|
| IMU rate | `SENSOR_PERIOD_US` in `SensorService.kt` | 5000 µs = 200 Hz ceiling; the HAL may give less |
| GPS rate | `LOCATION_PERIOD_MS` | 1 Hz is the realistic maximum |
| JPEG quality | `jpegQuality` in `CameraSource` | lower it before lowering resolution |
| Resolution | not set — CameraX defaults to 640×480 | raising it costs a *software* JPEG encode |

Thermals are the constraint most likely to bite you first. `CameraSource` already
skips the whole encode when no ROS node is connected; if the phone still throttles,
drop JPEG quality before anything else.

## Talking to a laptop

Android's Wi-Fi stack filters multicast, so default Fast DDS discovery usually finds
nothing from another machine. Either configure an `initialPeersList` in a Fast DDS
XML profile, or switch to `rmw_zenoh_cpp`, which is unicast. For phone-local testing:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```
