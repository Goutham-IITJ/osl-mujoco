"""Laptop client: read-only IMU observations in the existing browser dashboard."""
from pathlib import Path
import argparse
import os
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from oslbench.http_telemetry_source import HttpTelemetrySource, HttpTelemetryPresentation
from oslbench.replay_presentation import ReplayWebDashboard


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='Pi IPv4 address or SSH tunnel loopback')
    parser.add_argument('--pi-port', type=int, default=8793)
    parser.add_argument('--interval', type=float, default=.05)
    parser.add_argument('--timeout', type=float, default=1.)
    parser.add_argument('--connect-wait', type=float, default=10.)
    parser.add_argument('--port', type=int, default=8794, help='Existing local browser dashboard port')
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    source = dashboard = None
    try:
        source = HttpTelemetrySource(args.host, args.pi_port, interval=args.interval,
                                     timeout=args.timeout, token=os.environ.get('OSL_IMU_TOKEN'))
        deadline = time.monotonic() + args.connect_wait
        while not source.ready:
            if time.monotonic() >= deadline:
                raise RuntimeError('No verified Pi telemetry metadata: ' + source.diagnostics.get('error', ''))
            time.sleep(.02)
        presentation = HttpTelemetryPresentation(source)
        dashboard = ReplayWebDashboard(presentation, args.port, not args.no_browser)
        print(dashboard.url, flush=True)
        start = time.monotonic()
        while source.status != 'CLOSED':
            presentation.update(time.monotonic() - start)
            dashboard.pump()
            dashboard.update(presentation.snapshot())
            time.sleep(.02)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if dashboard is not None:
            dashboard.close()
        if source is not None:
            source.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
