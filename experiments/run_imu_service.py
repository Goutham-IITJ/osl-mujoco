"""Pi-only passive MicroStrain reader and read-only HTTP endpoint."""
from pathlib import Path
import argparse
import os
import signal
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from oslbench.microstrain_reader import MsclReader
from oslbench.imu_service import Acquisition, make_server, check_bind


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True, help='Prefer /dev/serial/by-id/...')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8793)
    parser.add_argument('--mounting', required=True, help='Describe actual mounting or explicitly state unknown')
    parser.add_argument('--stale-after', type=float, default=.5, help='Pi receipt silence threshold; not a safety limit')
    args = parser.parse_args()
    token = os.environ.get('OSL_IMU_TOKEN')
    reader = acquisition = server = None
    try:
        check_bind(args.bind, token)
        reader = MsclReader(args.device, args.baud)
        acquisition = Acquisition(reader, args.mounting, stale_after=args.stale_after)
        server = make_server(acquisition, args.bind, args.port, token)
        acquisition.start()
        def stop(signum, frame):
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        print(f'Read-only IMU endpoint: http://{args.bind}:{server.server_port}/v1/latest', flush=True)
        print('Existing filter stream only; no device configuration changes. No fresh data until observed.', flush=True)
        server.serve_forever(poll_interval=.1)
    except (RuntimeError, ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if server is not None:
            server.server_close()
        if acquisition is not None:
            acquisition.close()
        elif reader is not None:
            reader.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
