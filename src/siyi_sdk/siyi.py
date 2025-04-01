import asyncio
import socket
import struct
from crccheck.crc import Crc16

# used in TCP keep-alive
HEARTBEAT_PACKET = bytes.fromhex("55 66 01 01 00 00 00 00 00 59 8B")


class SiyiGimbalCamera:
    """
    Class to interface with the SIYI Gimbal Camera over TCP.

    This class manages the connection, heartbeat, and command construction
    (e.g., 0x07 rotation control) for controlling the gimbal's orientation.
    """

    def __init__(self, ip: str, port: int = 37260, *, heartbeat_interval: int = 2):
        """
        Initialize a new instance of the SiyiGimbalCamera.

        :param ip: The IP address of the camera.
        :param port: The port for TCP connection (default is 37260).
        :param heartbeat_interval: Interval (in seconds) for heartbeat packets.
        """
        self.ip = ip
        self.port = port
        self.heartbeat_interval = heartbeat_interval
        # These will be assigned once a connection has been established.
        self.reader: asyncio.StreamReader
        self.writer: asyncio.StreamWriter
        self.is_connected = False
        # Maintain a sequence counter (0 - 65535) for outgoing packets.
        self.seq: int = 0

    async def connect(self) -> None:
        """
        Establish a TCP connection with the gimbal camera.

        Initiates the connection, stores the reader and writer streams,
        flags the connection as open, and starts the heartbeat loop.
        """
        try:
            self.reader, self.writer = await asyncio.open_connection(
                self.ip, self.port
            )
            self.is_connected = True
            print(f"Connected to {self.ip}:{self.port}")
            # Schedule the heartbeat loop to run concurrently.
            asyncio.create_task(self.heartbeat_loop())
        except (socket.gaierror, ConnectionRefusedError) as e:
            self.is_connected = False
            print(f"Could not connect: {e}")
            raise

    async def disconnect(self) -> None:
        """
        Gracefully disconnect the TCP connection.

        Closes the writer stream and marks the connection as closed.
        """
        if self.is_connected:
            self.writer.close()
            # Wait for the writer to close to avoid resource warnings.
            await self.writer.wait_closed()
            self.is_connected = False
            print("Disconnected")
        else:
            print("Not connected, cannot disconnect.")

    async def heartbeat_loop(self) -> None:
        """
        Periodically send heartbeat packets to maintain the connection.

        This loop runs as long as the connection is active. If an error
        occurs, it breaks the loop and disconnects.
        """
        if not self.is_connected:
            print("Heartbeat loop started before connection was established.")
            return

        try:
            while self.is_connected:
                try:
                    # Write the heartbeat packet to the socket
                    self.writer.write(HEARTBEAT_PACKET)
                    await self.writer.drain()
                    print("Sent heartbeat packet")
                    # Sleep for the configured heartbeat interval
                    await asyncio.sleep(self.heartbeat_interval)
                except (socket.error, BrokenPipeError) as e:
                    print(f"Connection error in heartbeat loop: {e}")
                    break
        finally:
            print("Heartbeat loop stopped.")
            if self.is_connected:
                await self.disconnect()

    def build_rotation_packet(self, turn_yaw: int, turn_pitch: int) -> bytes:
        """
        Build a complete packet for the 0x07 Gimbal Rotation Control command.

        Packet Structure:
         - STX (2 bytes): Fixed header bytes: 0x55 0x66
         - CTRL (1 byte): Control flag (e.g., 0x01 to request an ACK)
         - Data Len (2 bytes, little-endian): Length of command-specific data (2 bytes)
         - SEQ (2 bytes, little-endian): Sequence number for the packet
         - CMD_ID (1 byte): Command identifier (0x07 for rotation control)
         - Data (2 bytes): Two signed 8-bit integers for turn_yaw and turn_pitch
         - CRC16 (2 bytes, little-endian): CRC16 checksum for error detection

        :param turn_yaw: int8_t value (-100 ~ 100) indicating the yaw rotation speed/direction.
        :param turn_pitch: int8_t value (-100 ~ 100) indicating the pitch rotation speed/direction.
        :return: The complete packet as bytes ready for sending.
        """
        packet = bytearray()

        # STX: Fixed header bytes.
        packet.extend(b"\x55\x66")
        # CTRL: Using 0x01 as the control flag for requesting an ACK.
        packet.append(0x01)
        # Data length: Only two bytes of data (turn_yaw and turn_pitch).
        packet.extend(struct.pack("<H", 2))
        # SEQ: 2-byte sequence number (little-endian).
        packet.extend(struct.pack("<H", self.seq))
        # CMD_ID: 0x07 for rotation control.
        packet.append(0x07)
        # Data: Pack turn_yaw and turn_pitch as signed 8-bit integers.
        packet.extend(struct.pack("bb", turn_yaw, turn_pitch))
        # Compute the CRC16 checksum over the packet so far.
        crc_value = Crc16.calc(packet)
        # Append the CRC16 checksum as 2 bytes (little-endian).
        packet.extend(struct.pack("<H", crc_value))
        # Increment the sequence counter for the next command, wrapping if needed.
        self.seq = (self.seq + 1) % 0x10000
        return bytes(packet)

    async def send_rotation_command(self, turn_yaw: int, turn_pitch: int) -> None:
        """
        Send the 0x07 rotation control command with specific yaw and pitch values.

        :param turn_yaw: int8_t value (-100 ~ 100) for yaw rotation.
        :param turn_pitch: int8_t value (-100 ~ 100) for pitch rotation.
        :raises RuntimeError: If the socket is not connected.
        """
        if not self.is_connected:
            raise RuntimeError(
                "Socket is not connected, cannot send rotation command."
            )
        # Build the full command packet.
        packet = self.build_rotation_packet(turn_yaw, turn_pitch)
        # Write the packet to the socket.
        self.writer.write(packet)
        await self.writer.drain()
        print(
            f"Sent rotation command with yaw {turn_yaw} and pitch {turn_pitch}"
        )
