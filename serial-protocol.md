# Simple machine - OpenCV protocol description

## Design choices
 - To increase ease of deployment, the protocol is simple (can be easily implemented by hobbyist on a variety of platform) and relies on the USB CDC serial protcol. While this probably increases a bit latency compared to a custom protocol, almost all micro-controllers have serial communication embedded in them, so for example this is natively supported by arduino.
 - To increase ease of deployment, the protocol does not rely on custom Vendor and Product ID. Instead, a magic number is sent at the start of communication for pairing. On failure, the device stops communication to avoid errors. A device reset can be used to restart communication.
 - To increase portability, a version number is sent during pairing. That way, reflashing of the device is not required (even if strongly suggested) even if a new version is released. Because it is the client that sends its version, the mobile application can support more than one version of the protocol.
 - To detect disconnects as fast as possible and to warn users, a heartbeat is included in the protocol.
 - For each requests, the device returns a 0x00 for success, or a code for errors. The list of errors can be found at the end of this document

# Protocol

- Big-endian (biggest byte of a number first)
- Sent over 115200 bpm CDC-ACM serial communication (USB serial)
- The devices make one handshake at the beginning of the communication, than can do an arbitrary number of requests
- Since packets can always hold at least 8 bytes, commands are expected to be under that length. Requests should be sent in a single transmission from the mobile to the embedded device.

## Handshake
Initialize communication between the connected devices. In case of failure, the devices puts itself in failure mode and stops all communications until reset.
 - Host: 0x73 0x6D 0x6F 0x76 (magic bytes, "smov" for "simple machine - open ventilator")
 - Client: 0x73 0x6D 0x6F 0x76 (magic bytes)
 - Client: <version number> (unsigned 16-bit int)
 - Host: 0x00 or 0x01 (Accept or reject connection — unsupported protocol version)

## Heartbeat
Make sure communication is kept between the host and the client. If tha clients takes more than 50 ms to answer, a disconnect is assumed. Reconnect procedure should be initiated and a warning should be brought up for the operator.
 - Host: 0x00
 - Client: 0x00

## Set power/speed:
Set the power or speed of the motor to a certain value. To accomodate for a variety of hardware and a potential lack of encoder for the motor, this value does not need to map to any concrete unit nor value. In case a motor with feedback is controlled, this should be a linear mapping of speed to ease the operation of the mobile device. If a simple motor is used, the mobile will make the control loop based on visual results.
 - Host: 0x01 <speed (16-bit unsigned integer)>
 - Client: 0x00

# Error codes
 - **0x01**: Missing argument for *power/speed* command
 - **0xFE**: Communication error
 - **0xFF**: Unknown command
