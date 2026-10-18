"""Passive MSCL filter reader. No configuration, sampling or actuator commands.

API references and deployment prerequisites: docs/IMU_TELEMETRY.md.
Only decoded MSCL fields are used; this is not a binary MIP parser.
"""
import hashlib
import importlib
import math
from pathlib import Path
import sys
import tempfile


class MsclReader:
    """Pi-only reader; read(timeout_ms) yields decoded packet dictionaries.

    The service owns the sole reader thread. No automatic device reconnect or
    timestamp-epoch stitching: restart the service after a physical reconnect.
    """
    def __init__(self, device, baud=115200):
        if sys.platform != 'linux':
            raise RuntimeError('MSCL acquisition runs on the Pi/Linux, not this laptop')
        try:
            mscl = importlib.import_module('mscl')
        except (ImportError, OSError) as exc:
            raise RuntimeError('Official HBK pymscl is unavailable. Check Pi Python/ARM64/glibc compatibility; '
                               'see docs/IMU_TELEMETRY.md. Do not install an unrelated mip package.') from exc
        import fcntl
        resolved = str(Path(device).resolve(strict=True))
        digest = hashlib.sha256(resolved.encode()).hexdigest()[:24]
        self._lock = open(Path(tempfile.gettempdir()) / ('osl-imu-' + digest + '.lock'), 'a')
        self._connection = None
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._lock.close()
            raise RuntimeError('Another OSL IMU reader already owns this USB device') from exc
        try:
            self._connection = mscl.Connection.Serial(resolved, baud)
            self._node = mscl.InertialNode(self._connection)
        except Exception as exc:
            self.close()
            raise RuntimeError('Cannot open IMU: verify USB path, serial permissions, MSCL version, '
                               'and that no other application owns the port') from exc

    @staticmethod
    def decode_packet(packet):
        """Decode ONLY filter attitude fields; never mix 0x80 complementary data.

        SDK MipTypes: 0x8205 roll/pitch/yaw qualifiers 6/7/8;
        0x8203 quaternion vector qualifier 5. Values are copied without rotation,
        normalization or interpolation. Finite invalid values remain invalid.
        """
        values = {}
        for point in packet.data():
            field, qualifier = int(point.field()), int(point.qualifier())
            if field == 0x8205 and qualifier in (6, 7, 8):
                raw = [('imu_' + ('roll', 'pitch', 'yaw')[qualifier - 6], point.as_float())]
            elif field == 0x8203 and qualifier == 5:
                vector = point.as_Vector()
                if vector.size() != 4:
                    raise ValueError('MSCL quaternion does not contain four components')
                raw = [('imu_q' + str(i), vector.as_floatAt(i)) for i in range(4)]
            else:
                continue
            for name, value in raw:
                if name in values:
                    raise ValueError('Duplicate orientation channel in SDK packet')
                finite = math.isfinite(value)
                valid = bool(point.hasValidFlag() and point.valid() and finite)
                reason = ('' if valid else 'nonfinite_device_value' if not finite
                          else 'device_invalid_or_validity_flag_absent')
                values[name] = dict(value=value if finite else None, valid=valid, reason=reason)
        if not values:
            return None
        shared = packet.shared()
        clocks = {}
        if shared.hasReferenceTime():
            clocks['reference_ns'] = dict(value=int(shared.referenceTime()), valid=True)
        for stem, name in (('Gps', 'gps'), ('External', 'external')):
            if getattr(shared, 'has' + stem + 'Timestamp')():
                valid = bool(getattr(shared, name + 'TimestampValid')())
                # Invalid SDK timestamps may be represented as zero by MSCL;
                # retain invalidity, not a fabricated timestamp of zero.
                value = int(getattr(shared, name + 'Timestamp')().nanoseconds()) if valid else None
                key = 'gps_unix_ns' if name == 'gps' else 'external_ns'
                clocks[key] = dict(value=value, valid=valid)
        if packet.hasDeviceTime():
            valid = bool(packet.deviceTimeValid())
            clocks['legacy_unix_ns'] = dict(
                value=int(packet.deviceTimestamp().nanoseconds()) if valid else None, valid=valid)
        return dict(values=values, device_timestamps=clocks)

    def read(self, timeout_ms=100):
        return [decoded for packet in self._node.getDataPackets(timeout_ms)
                if (decoded := self.decode_packet(packet)) is not None]

    def close(self):
        try:
            if self._connection is not None:
                self._connection.disconnect()
                self._connection = None
        finally:
            if not self._lock.closed:
                self._lock.close()
