"""Receipt-aware presentation for a read-only TelemetrySource."""
from oslbench.telemetry.source import TelemetrySource, local_time, validate_observation
from oslbench.presentation.observations import observation_channels


class TelemetryPresentation:
    def __init__(self, source: TelemetrySource, stale_after_s=.5):
        self.source=source
        self.metadata=source.metadata
        self.stale_after_s=local_time(stale_after_s)
        if not self.stale_after_s:raise ValueError('Stale threshold must be positive')
        self.time_s=0.
        self.latest={}
        self.received={}
        self.history={name:[] for name in self.metadata['channels']}
        self.delivered=0
        self.last_sequence=None

    def update(self, now_s):
        now=local_time(now_s)
        if now<self.time_s:raise ValueError('Local receipt clock cannot reverse')
        batch=self.source.poll(now)
        previous=self.last_sequence
        timestamps={name:r.time_s for name,r in self.latest.items()}
        # Reject the whole batch before mutating presented observations.
        for record in batch:
            validate_observation(record,self.metadata)
            if previous is not None and record.sequence<=previous:
                raise ValueError('Arrival sequence must increase')
            if record.time_s<timestamps.get(record.channel,0.):
                raise ValueError('Per-channel source time reversed; start a new source session')
            previous=record.sequence;timestamps[record.channel]=record.time_s
        for record in batch:
            self.latest[record.channel]=record
            self.received[record.channel]=now
            self.history[record.channel].append(record)
            # Live presentation history is bounded; acquisition is not a recorder.
            self.history[record.channel]=self.history[record.channel][-320:]
            self.delivered+=1;self.last_sequence=record.sequence
        self.time_s=now
        return batch

    def key(self,key):
        if key=='escape':self.source.close()

    def snapshot(self):
        m=self.metadata
        synthetic=m['source']=='synthetic_mock'
        return dict(mode='mock' if synthetic else 'telemetry',dataset_id=m['dataset_id'],clock_id=m['clock']['id'],
            identity=dict(m.get('identity',{}),stale_after_receipt_s=self.stale_after_s,
                          display_clock='caller monotonic elapsed seconds; not synchronized to observation clock'),
            provenance=m['source'],label='SYNTHETIC MOCK TELEMETRY - NO PHYSICAL HARDWARE' if synthetic else 'EXTERNAL OBSERVATIONS - PROVENANCE AS DECLARED',
            status=self.source.status,time_s=self.time_s,speed=None,delivered=self.delivered,total=None,
            last_sequence=self.last_sequence,
            channels=observation_channels(m,self.latest,self.history,self.time_s,
                                          self.received,self.stale_after_s))
