import pytest
from src.serial_data import SerialData, ChecksumError
import struct


def test_checksum_invalid():
    header = [66, 82, 4, 0, 5, 0, 0, 0, 1, 2, 3,
              0]  # Basic Protocol Version response. Real checksum = 163
    with pytest.raises(ChecksumError):
        s = SerialData(header)
        s.checksum = struct.pack('<H', 2)  # Inject invalid checksum
        assert not s._validate_checksum()


def test_checksum_valid():
    header = [66, 82, 4, 0, 5, 0, 0, 0, 1, 2, 3,
              0]  # Basic Protocol Version response. Real checksum = 163
    s = SerialData(header)
    s.checksum = struct.pack('<H', 163)  # Inject valid checksum
    assert s._validate_checksum()
