# `vda5050_msgs`

This package provides ROS 2 message definitions and JSON serialization/deserialization
utlities for the [VDA 5050](https://www.vda.de/en/topics/automotive-industry/vda-5050)
specification.

> [!NOTE]
> The message definitions in this package follow the **VDA 5050 version 2.0.0** specification.

## Features

The features of this package include:

- ROS 2 message definitions for the VDA 5050 message schema.
- Utility functions for converting between ROS 2 messages and JSON.

> [!WARNING]
> This package is under acive development. APIs and mesage definitions may change without notice.
Use with caution in production environments.

## Requirements

- ROS 2
- `nlohmann_json`


## Supported Messages

- `vda5050_msgs::msg::Connection`

`vda5050_msgs::msg::Header` is a shared field across all VDA 5050 messages.

## Usage

### Build the ROS 2 messages and utility library

1. Create a `colcon` workspace and download the source code

```bash
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/ros-industrial/vda5050_msgs.git
```

2. Download dependencies and build the workspace

```bash
cd colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Example CMake

Link the JSON utility library in your `CMakeLists.txt`:

```
target_link_libraries(example
  PUBLIC
    ${vda5050_msgs_LIBRARIES}
    vda5050_msgs::json_utils
)

target_include_directories(example
  PUBLIC
    ${vda5050_msgs_INCLUDE_DIRS}
)
```

### Example C++

```cpp
#include <chrono>

#include <nlohmann/json.hpp>

#include <vda5050_msgs/json_utils/header.hpp>
#include <vda5050_msgs/msg/header.hpp>

vda5050_msgs::msg::Header header;
header.header_id = 0;

// Timestamp in milliseconds in UTC indicating a specific point relative to
// a clock's 0 point.
//
// Note: When using the serialization/deserialization utilities, this will be
// converted to/from ISO 8601 format YYYY-MM-DDTHH:mm:ss.ssZ
auto duration = std::chrono::system_clock::now().time_since_epoch();
auto millisecs =
    std::chrono::duration_cast<std::chrono::milliseconds>(duration);
header.timestamp = static_cast<int64_t>(millisecs.count());

header.version = "2.0.0";
header.manufacturer = "Manufacturer";
header.serial_number = "S0001";

// Serialize to JSON
nlohmann::json j;
vda5050_msgs::json_utils::to_json(j, header);

// Deserialize from JSON
vda5050_msgs::msg::Header header_d;
vda5050_msgs::json_utils::from_json(j, header_d);
```

## Support

This package is developed and maintained by ROS-Industrial Consortium Asia Pacific.

## License

The package is released under the [Apache 2](./LICENSE) license.
