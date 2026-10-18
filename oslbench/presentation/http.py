"""Transport diagnostics added to the shared observation display."""
from .telemetry import TelemetryPresentation


class HttpTelemetryPresentation(TelemetryPresentation):
    """Existing presentation plus transport diagnostics; no second dashboard."""
    def snapshot(self):
        result = super().snapshot()
        details = self.source.diagnostics
        result['identity']['transport'] = details
        # Never call old producer data fresh merely because HTTP succeeded.
        for name, item in details.get('channels', {}).items():
            if item['pi_receipt_age_s'] > details['stale_after_s']:
                result['channels'][name]['freshness'] = 'STALE'
        return result
