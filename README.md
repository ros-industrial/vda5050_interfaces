# vda5050_interfaces

This package provides ROS 2 message definitions for the
[VDA5050](https://www.vda.de/en/topics/automotive-industry/vda-5050)
specification.

> [!NOTE]
> The message definitions in this package follow the **VDA5050 version 2.0.0**.

## Supported Messages

### Core Messages

- `Connection.msg`
- `Order.msg`
- `State.msg`
- `InstantActions.msg`
- `Factsheet.msg`
- `Visualization.msg`

### Common Header

All core messages include a shared header

- `Header.msg`

This message contains all the common VDA5050 header fields (e.g., `header_id`,
`timestamp`, `version`, `manufacturer`, `serial_number`) and is embedded in
each top-level message instead of duplicating the fields.

Example:
```cpp
vda5050_interfaces::msg::State msg;
msg.header.header_id = 1;
msg.header.version = "2.0.0";
```

## Build

```bash
colcon build --packages-select vda5050_interfaces
source install/setup.bash
```

## Usage

### Example C++

```cpp
#include <vda5050_interfaces/msg/connection.hpp>

vda5050_interfaces::msg::Connection msg;
msg.connection_state = vda5050_interfaces::msg::Connection::ONLINE;
```

### Optional Fields

ROS 2 does not natively support optional fields. So this package uses **bounded
arrays of size <= 1** to represent optional values.

### Definition

```
string[<=1] zone_set_id
```

### Usage

```cpp
vda5050_interfaces::msg::State msg;

// Assign a value
msg.zone_set_id.push_back("test_zone");
```

### Semantics

- `msg.zone_set_id.empty()`: Field is not set
- `msg.zone_set_id.front()`: Field is set
- `msg.zone_set_id.front().empty()`: Field is set to `""`

For all types, the same pattern applies with the default values changing depending on the type.

## Support

This package is developed and maintained by ROS-Industrial Consortium Asia Pacific.

## License

The package is released under the [Apache 2](./LICENSE) license.
