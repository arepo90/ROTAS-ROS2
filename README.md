# ROTAS + ROS2

Full implementation of ROTAS and ROS2 for real-time video streaming and information sharing from ROS2 nodes to ROTAS server applications. 

## Prerequisites

- CMake >=3.10
- OpenCV 4.10.0 + opencv_contrib extra modules
- ROS2
- colcon

> WIP

## How to use

### Windows

1. Clone repo
```
git clone https://github.com/arepo90/ROTAS-ROS2.git
cd ROTAS-ROS2
```

> WIP

2. Run

Talker (sender for testing purposes):
```
ros2 run ROTAS_pkg talker
```

Relay (ROS2 receiver and ROTAS sender):
```
ros2 run ROTAS_pkg relay
```

### Linux

1. Clone repo
```
git clone https://github.com/arepo90/ROTAS-ROS2.git
cd ROTAS-ROS2
```

> WIP

2. Run

Talker (sender for testing purposes):
```
ros2 run ROTAS_pkg talker
```

Relay (ROS2 receiver and ROTAS sender):
```
ros2 run ROTAS_pkg relay_linux
```

### Flags

To change application settings without the need to modify the source code and rebuild the executables, you can include any of the following flags with its corresponding arguments:

> Any changes made to the client will be communicated to the server automatically during the initial handshake, with the notable exception of the port and camera amount, since communication cannot be established without a common port.

| Flag             | Argument     | Setting                     | Default               |
|------------------|--------------|-----------------------------|-----------------------|
| `-H` `--help`    |              | Displays  options           |                       |
| `-v` `--verbose` |              | Verbose console logs        | false                 |
| `-p` `--port`    | port         | TCP port                    | 8080                  |
| `-i` `--ip`      | address      | Server IP address           | 127.0.0.1 (localhost) |
| `-c` `--cams`    | number, list | List of camera sources      | 1 0                   |
| `-w` `--width`   | pixels       | Horizontal video resolution | 1280                  |
| `-h` `--height`  | pixels       | Vertical video resolution   | 720                   |
| `-q` `--quality` | number       | Video image quality (1-100) | 50                    |

> Camera USB ports tend to be in order (starting from 0), but this is not always the case. To verify, run the `cam_detection.py` script and use the `--cams` flag on the client to specify them.

> When streaming multiple webcams, one socket is used for every source (in ascending order). For example, for 3 sources on default settings, ports 8080, 8081 and 8082 must be open and available.

## Transmission

> WIP

## Logs
All applications regularly print console messages of different types (to receive __all__ available logs, make sure to include the `--verbose` flag on the executable):

- `[i]` __Information__: Provides updates as to what the program is currently doing. When followed by `...`, it means the program is awaiting a response.
- `[w]` __Warning__: Alerts about a non fatal error, most likely caused by the remote peer or a dependency.
- `[e]` __Error__: Gives out an error number and message relating to what went wrong, followed by ending the program. Usually caused by an internal problem or a fatal cononection loss.
- `[recv]` __Received__: Shows relevant information about the latest packet received (size, loss, frame rate, etc.).
- `[ros2]` __ROS2 Topic__: Prints the latest information received from the subscribed ROS2 topic.
- __Dependency warnings and info__: OpenCV, Winsock, ROS2 and colcon tend to print out compilation warnings and information throughout the execution of the applications. They can be safely ingnored as long as they don't include any errors.

## Author
Miguel Esteban Martinez Villegas - arepo90
