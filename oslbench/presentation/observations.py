"""Shared observation formatting, independent of scheduling and browser lifecycle."""

MAX_PLOT_PTS = 320


def observation_channels(manifest, latest, history, time_s, received=None, stale_after_s=None):
    """Shared display formatting; local receipt age never assumes source-clock sync."""
    channels = {}
    for name, meta in manifest['channels'].items():
        record = latest.get(name)
        sample = record.sample if record else None
        quality = ('UNAVAILABLE' if record is None else 'MISSING' if sample is None
                   else 'VALID' if sample.valid else 'INVALID')
        age = None if record is None else time_s-record.time_s
        rows = history[name]
        # Select original observations only; never create interpolated points.
        if len(rows) > MAX_PLOT_PTS:
            rows = [rows[i*(len(rows)-1)//(MAX_PLOT_PTS-1)] for i in range(MAX_PLOT_PTS)]
        channels[name] = dict(unit=meta['unit'], frame=meta['frame'], stage=meta['stage'],
            sign=meta['sign'], source=meta['source'], quality=quality,
            value=sample.value if sample is not None else None,
            time_s=record.time_s if record else None, age_s=age,
            held=age is not None and age > 0,
            sequence=record.sequence if record else None,
            snapshot_id=record.snapshot_id if record else None,
            points=[[r.time_s, r.sample.value if r.sample is not None and r.sample.valid else None] for r in rows])
    if received is not None:
        for name, channel in channels.items():
            receipt = received.get(name)
            age = None if receipt is None else time_s - receipt
            channel.update(received_time_s=receipt, age_s=age,
                           held=age is not None and age > 0,
                           freshness=('UNAVAILABLE' if age is None else
                                      'STALE' if age > stale_after_s else 'RECENT'))
    return channels
