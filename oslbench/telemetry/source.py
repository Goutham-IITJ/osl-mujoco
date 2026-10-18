"""Read-only producer boundary and deterministic synthetic mock. No transport."""
from typing import Protocol
import copy
import math

from oslbench.contracts import Sample
from oslbench.telemetry_schema import ReplayRecord, CHANNELS, OBSERVATION_CHANNELS


class TelemetrySource(Protocol):
    """One source session, one source clock; no reset/seek/commands required.

    poll(local_time_s) is nonblocking and returns newly available ReplayRecord
    envelopes in arrival sequence order. local_time_s is caller-owned monotonic
    elapsed time, NEVER presumed equal to record.time_s. Empty means no new data.
    status distinguishes OPEN, EXHAUSTED, DISCONNECTED and CLOSED. close is
    idempotent, terminal and releases acquisition resources; later polls return ().
    metadata is a detached dict using the existing channel declaration vocabulary.
    """
    @property
    def metadata(self) -> dict: ...
    @property
    def status(self) -> str: ...
    def poll(self, local_time_s: float) -> tuple[ReplayRecord, ...]: ...
    def close(self) -> None: ...


def local_time(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ValueError('Local elapsed time must be finite and nonnegative')
    return float(value)


def validate_observation(record, metadata):
    """Validate the existing envelope against declared channels before display."""
    if not isinstance(record, ReplayRecord):
        raise TypeError('Expected existing ReplayRecord observation envelope')
    if type(record.sequence) is not int or record.sequence < 0 or type(record.snapshot_id) is not int or record.snapshot_id < 0:
        raise ValueError('Observation sequence/group must be nonnegative integers')
    local_time(record.time_s)
    meta = metadata['channels'].get(record.channel)
    if meta is None or record.channel not in OBSERVATION_CHANNELS:
        raise ValueError('Undeclared or unsupported channel')
    if tuple(meta[k] for k in ('quantity','unit','frame')) != OBSERVATION_CHANNELS[record.channel]:
        raise ValueError('Incompatible channel units/frame')
    if (record.clock_id != metadata['clock']['id'] or metadata['clock']['unit'] != 's' or
        (record.unit,record.frame,record.stage,record.source) != tuple(meta[k] for k in ('unit','frame','stage','source'))):
        raise ValueError('Envelope does not match declared clock/channel provenance')
    if record.sample is not None:
        sample=record.sample
        if not isinstance(sample,Sample) or (sample.time_s,sample.unit,sample.frame,sample.stage,sample.source) != (record.time_s,record.unit,record.frame,record.stage,record.source):
            raise ValueError('Sample and envelope disagree')


class MockTelemetrySource:
    """Finite synthetic records with delayed arrival and a deliberate disconnect.

    Source times near 100 s deliberately differ from local receipt times near 0 s.
    No relationship to a real sensor, sampling rate or calibration is implied.
    """
    def __init__(self):
        names=('knee_angle','knee_velocity','actuator_output_angle','belt_joint_torque')
        channels={name:dict(quantity=CHANNELS[name][0],unit=CHANNELS[name][1],frame=CHANNELS[name][2],
            stage='post_step',source='synthetic_mock',timestamp='time_s',
            sign='Synthetic fixture uses existing positive model-coordinate convention; no hardware calibration') for name in names}
        self._metadata=dict(dataset_id='synthetic_mock_session',source='synthetic_mock',
            clock=dict(id='synthetic_observation_clock',unit='s'),channels=channels,
            identity=dict(synthetic=True,description='Hand-authored synthetic contract examples; not physics output or physical measurements',
                          stage_note='post_step is an existing synthetic fixture stage label, not evidence of a simulated or physical step'))
        # arrival time, channel, source time, value (None = missing), validity
        schedule=((0.,'knee_angle',100.,.2,True),(.25,'knee_velocity',99.9,.3,True),
                  (.75,'knee_angle',100.4,None,False),(1.,'knee_angle',100.7,.25,True),
                  (1.25,'belt_joint_torque',100.2,2.,False),(1.5,'knee_angle',101.,.3,True))
        self._schedule=[]
        for sequence,(arrival,name,t,value,valid) in enumerate(schedule):
            m=channels[name]
            sample=None if value is None else Sample(value,m['unit'],m['frame'],t,m['stage'],m['source'],valid)
            record=ReplayRecord(sequence,sequence,name,t,self._metadata['clock']['id'],sample,m['source'],m['unit'],m['frame'],m['stage'])
            validate_observation(record,self._metadata)
            self._schedule.append((arrival,record))
        self._cursor=0
        self._now=0.
        self._status='OPEN'

    @property
    def metadata(self): return copy.deepcopy(self._metadata)

    @property
    def status(self): return self._status

    def poll(self, local_time_s):
        now=local_time(local_time_s)
        if now < self._now: raise ValueError('Local receipt clock cannot reverse')
        self._now=now
        if self._status in ('CLOSED','DISCONNECTED'):return ()
        records=[]
        while self._cursor < len(self._schedule) and self._schedule[self._cursor][0] <= now:
            records.append(self._schedule[self._cursor][1]);self._cursor+=1
        if self._cursor==len(self._schedule):self._status='EXHAUSTED'
        if now>=3.:self._status='DISCONNECTED'
        return tuple(records)

    def disconnect(self):
        if self._status!='CLOSED':self._status='DISCONNECTED'

    def close(self): self._status='CLOSED'
