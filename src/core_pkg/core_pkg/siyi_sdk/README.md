# SIYI Gimbal Camera Python Library

This library provides a Python interface for controlling SIYI gimbal cameras over TCP. It allows you to:

- Connect to the camera and maintain a stable connection.
- Control the camera's movement (pan, tilt).
- Request a continuous stream of data (attitude, etc.).

> [!NOTE]
> The code for this library (including this README) was written almost entirely by an AI assistant, with
> the assistance and direction of a human developer. It probably sucks, but I need sleep, so I'm not going
> to redo it. Sorry.

## Core Features

- **Asynchronous communication:** Utilizes `asyncio` for non-blocking operation.
- **Command handling:**  Easy-to-use functions for sending control commands.
- **Data streaming:**  Ability to receive continuous data streams for real-time monitoring.

## Key Components

- `SiyiGimbalCamera` class: Main class for interfacing with the camera.
- Enums: `DataStreamType`, `DataStreamFrequency`, `SingleAxis`, `CommandID` to manage protocol values.
- Logging: Uses the Python `logging` library.

Refer to the code comments and docstrings for specific function details.
