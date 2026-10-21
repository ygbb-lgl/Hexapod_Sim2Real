#!/usr/bin/env python3
"""Offline CRC and stream-resynchronisation tests for the DETA40 SDK.

No serial device is opened.  Run directly with::

    python test_crc.py

The fixed IMU/AHRS vectors below were reconstructed byte-for-byte from the
2026-10-20 DETA40 capture.  They therefore verify the CRC algorithms against
independent device output; the remaining synthetic frames exercise parser
behaviour.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import List, Tuple


THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from imu_sdk import (  # noqa: E402
    FRAME_HEAD,
    TYPE_AHRS,
    TYPE_IMU,
    _FDILinkStreamParser,
    calc_crc8,
    calc_crc16,
)


# Independent vectors captured from the physical DETA40.  Do not replace
# these constants with values calculated by build_frame().
REAL_IMU_HEADER = bytes.fromhex("fc403885")
REAL_IMU_HEADER_CRC8 = 0xBA
REAL_IMU_PAYLOAD = bytes.fromhex(
    "3b44aa3ae4de53baafb0a1b7241d20bd2de53e3e43e51cc1"
    "00c0bdc300e017c40000314405562b4280e6c54705562b42"
    "f97f816101000000"
)
REAL_IMU_PAYLOAD_CRC16 = 0xE2C0

REAL_AHRS_HEADER = bytes.fromhex("fc413084")
REAL_AHRS_HEADER_CRC8 = 0x39
REAL_AHRS_PAYLOAD = bytes.fromhex(
    "92338f3a78547aba024dc0b8b0ab97bca26f72bb088da43ccd"
    "f97f3f355b17bc6499febaa642243c1c6c816101000000"
)
REAL_AHRS_PAYLOAD_CRC16 = 0xCF76


def build_frame(packet_type: int, sequence: int, payload: bytes) -> bytes:
    """Build a valid wire frame using the DETA40 byte order."""

    header = bytes((FRAME_HEAD, packet_type, len(payload), sequence & 0xFF))
    payload_crc = calc_crc16(payload)
    return (
        header
        + bytes(
            (
                calc_crc8(header),
                (payload_crc >> 8) & 0xFF,
                payload_crc & 0xFF,
            )
        )
        + payload
    )


def parsed_signature(
    frames: List[Tuple[dict, bytes]],
) -> List[Tuple[int, int, bytes]]:
    """Keep only fields relevant to stream-order assertions."""

    return [(header["type"], header["sequence"], payload) for header, payload in frames]


class TestDETA40CRCReferenceVectors(unittest.TestCase):
    """Validate both algorithms against bytes emitted by the real sensor."""

    def test_standard_check_values(self) -> None:
        self.assertEqual(calc_crc8(b"123456789"), 0xA1)
        self.assertEqual(calc_crc16(b"123456789"), 0x31C3)

    def test_real_imu_crc_vector(self) -> None:
        self.assertEqual(calc_crc8(REAL_IMU_HEADER), REAL_IMU_HEADER_CRC8)
        self.assertEqual(calc_crc16(REAL_IMU_PAYLOAD), REAL_IMU_PAYLOAD_CRC16)

    def test_real_ahrs_crc_vector(self) -> None:
        self.assertEqual(calc_crc8(REAL_AHRS_HEADER), REAL_AHRS_HEADER_CRC8)
        self.assertEqual(calc_crc16(REAL_AHRS_PAYLOAD), REAL_AHRS_PAYLOAD_CRC16)


class TestDETA40CRCStreamParser(unittest.TestCase):
    def setUp(self) -> None:
        self.imu_frame = (
            REAL_IMU_HEADER
            + bytes(
                (
                    REAL_IMU_HEADER_CRC8,
                    REAL_IMU_PAYLOAD_CRC16 >> 8,
                    REAL_IMU_PAYLOAD_CRC16 & 0xFF,
                )
            )
            + REAL_IMU_PAYLOAD
        )
        self.ahrs_frame = (
            REAL_AHRS_HEADER
            + bytes(
                (
                    REAL_AHRS_HEADER_CRC8,
                    REAL_AHRS_PAYLOAD_CRC16 >> 8,
                    REAL_AHRS_PAYLOAD_CRC16 & 0xFF,
                )
            )
            + REAL_AHRS_PAYLOAD
        )

    def test_valid_physical_frames_are_accepted(self) -> None:
        parser = _FDILinkStreamParser()
        frames = parser.feed(self.imu_frame + self.ahrs_frame)

        self.assertEqual(
            parsed_signature(frames),
            [
                (TYPE_IMU, 0x85, REAL_IMU_PAYLOAD),
                (TYPE_AHRS, 0x84, REAL_AHRS_PAYLOAD),
            ],
        )
        self.assertEqual(parser.crc_error_count, 0)

    def test_header_crc_error_is_rejected_then_resynchronises(self) -> None:
        parser = _FDILinkStreamParser()
        damaged = bytearray(self.imu_frame)
        damaged[4] ^= 0x01

        frames = parser.feed(bytes(damaged) + self.ahrs_frame)

        self.assertEqual(
            parsed_signature(frames),
            [(TYPE_AHRS, 0x84, REAL_AHRS_PAYLOAD)],
        )
        self.assertEqual(parser.crc_error_count, 1)
        self.assertEqual(parser.header_crc_error_count, 1)
        self.assertEqual(parser.payload_crc_error_count, 0)

    def test_payload_crc_error_is_rejected_then_resynchronises(self) -> None:
        parser = _FDILinkStreamParser()
        damaged = bytearray(self.imu_frame)
        damaged[7 + 17] ^= 0x80

        frames = parser.feed(bytes(damaged) + self.ahrs_frame)

        self.assertEqual(
            parsed_signature(frames),
            [(TYPE_AHRS, 0x84, REAL_AHRS_PAYLOAD)],
        )
        self.assertEqual(parser.crc_error_count, 1)
        self.assertEqual(parser.header_crc_error_count, 0)
        self.assertEqual(parser.payload_crc_error_count, 1)

    def test_missing_payload_byte_does_not_consume_next_frame(self) -> None:
        parser = _FDILinkStreamParser()
        shortened = self.imu_frame[: 7 + 17] + self.imu_frame[7 + 18 :]

        frames = parser.feed(shortened + self.ahrs_frame)

        self.assertEqual(
            parsed_signature(frames),
            [(TYPE_AHRS, 0x84, REAL_AHRS_PAYLOAD)],
        )
        self.assertGreaterEqual(parser.crc_error_count, 1)
        self.assertEqual(parser.header_crc_error_count, 0)
        self.assertEqual(parser.payload_crc_error_count, 1)

    def test_frame_split_at_every_byte_is_reassembled_once(self) -> None:
        # Feeding one byte at a time covers every possible serial read split,
        # including splits inside both CRC fields.
        parser = _FDILinkStreamParser()
        frames = []  # type: List[Tuple[dict, bytes]]
        for byte in self.imu_frame:
            frames.extend(parser.feed(bytes((byte,))))

        self.assertEqual(
            parsed_signature(frames),
            [(TYPE_IMU, 0x85, REAL_IMU_PAYLOAD)],
        )
        self.assertEqual(parser.crc_error_count, 0)
        self.assertEqual(bytes(parser.buffer), b"")

    def test_sticky_frames_keep_order(self) -> None:
        parser = _FDILinkStreamParser()
        frames = parser.feed(self.imu_frame + self.ahrs_frame + self.imu_frame)

        self.assertEqual(
            [(header["type"], header["sequence"]) for header, _ in frames],
            [(TYPE_IMU, 0x85), (TYPE_AHRS, 0x84), (TYPE_IMU, 0x85)],
        )
        self.assertEqual(parser.crc_error_count, 0)

    def test_noise_bad_length_and_bad_crc_do_not_hide_next_frame(self) -> None:
        parser = _FDILinkStreamParser()
        bad_header_crc = bytearray(self.imu_frame)
        bad_header_crc[4] ^= 0x40
        malformed_length = bytes((FRAME_HEAD, TYPE_IMU, 0x37, 0x22))
        stream = (
            b"\x00\x7e\xfdgarbage"
            + malformed_length
            + bytes(bad_header_crc)
            + b"\xfd\x55"
            + self.ahrs_frame
        )

        # Exercise resynchronisation across arbitrary read boundaries as well
        # as multiple failure modes in one stream.
        frames = []  # type: List[Tuple[dict, bytes]]
        cuts = (1, 5, 13, 19, 37, len(stream))
        start = 0
        for stop in cuts:
            frames.extend(parser.feed(stream[start:stop]))
            start = stop

        self.assertEqual(
            parsed_signature(frames),
            [(TYPE_AHRS, 0x84, REAL_AHRS_PAYLOAD)],
        )
        self.assertEqual(parser.crc_error_count, 1)
        self.assertGreaterEqual(parser.frame_error_count, 1)
        self.assertGreater(parser.discarded_byte_count, 0)

    def test_crc_failure_cannot_update_sdk_packet_state(self) -> None:
        # This parser-level assertion protects _reader_loop: rejected frames
        # never reach IMUSDK._handle_packet, so cached policy data cannot be
        # overwritten with a corrupted sample.
        parser = _FDILinkStreamParser()
        damaged = bytearray(self.imu_frame)
        damaged[-1] ^= 0x01

        self.assertEqual(parser.feed(bytes(damaged)), [])
        self.assertEqual(parser.crc_error_count, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
