"""Stable joint-selectable comparison and simulated-variable browser dashboard."""
from collections import deque
from pathlib import Path
import math
from oslbench.presentation.replay import ReplayWebDashboard

class TrackingPresentation:
    def __init__(self,source,output):
        self.source=source;self.output=str(output);self.stop_requested=False;self.status='WAITING'
        self.rows={j:deque(maxlen=4000) for j in ('knee','ankle')}
        self.steps={j:deque(maxlen=4000) for j in ('knee','ankle')}
        self.count={j:0 for j in self.rows};self.squares={j:0. for j in self.rows};self.peaks={j:0. for j in self.rows}
        self.assumptions=''
    def key(self,key):
        if key=='stop-recording':self.stop_requested=True
    def add(self,row):
        j=row['joint'];self.rows[j].append(row)
        if row['difference_deg'] is not None:
            self.count[j]+=1;self.squares[j]+=row['difference_deg']**2;self.peaks[j]=max(self.peaks[j],abs(row['difference_deg']))
    def step(self,row):
        if round(row['source_time_s']/.0005)%20==0:self.steps[row['joint']].append(row)
    def snapshot(self):
        thin=lambda rows:list(rows)[::max(1,len(rows)//300)]
        return dict(source=self.source,output=self.output,status=self.status,assumptions=self.assumptions,
            comparison={j:thin(r) for j,r in self.rows.items()},telemetry={j:thin(r) for j,r in self.steps.items()},
            metrics={j:dict(pairs=self.count[j],rms_deg=math.sqrt(self.squares[j]/self.count[j]) if self.count[j] else None,
                           peak_deg=self.peaks[j]) for j in self.rows})

def dashboard(presentation,port=8798,open_browser=True):
    d=ReplayWebDashboard(presentation,port,open_browser=False)
    d.html=Path(__file__).with_suffix('.html').read_bytes()
    if open_browser:d._open()
    return d
