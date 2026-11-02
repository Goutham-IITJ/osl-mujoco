"""Generate editable/vector and raster figures of the implemented comparison path."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

OUT=Path(__file__).resolve().parent
plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','pdf.fonttype':42})
fig,ax=plt.subplots(figsize=(16,11))
fig.patch.set_facecolor('white')
ax.set(xlim=(0,18),ylim=(0,12));ax.axis('off')
ink='#173047'
def box(x,y,w,h,title,detail,color='#e9f2fa'):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.06,rounding_size=.12',facecolor=color,edgecolor='#56758b',lw=1.2))
    ax.text(x+w/2,y+h*.69,title,ha='center',va='center',fontsize=12,weight='bold',color=ink)
    ax.text(x+w/2,y+h*.30,detail,ha='center',va='center',fontsize=10,color=ink,linespacing=1.3)
def arrow(points,color='#496579',dashed=False):
    for a,b in zip(points[:-2],points[1:-1]):
        ax.plot([a[0],b[0]],[a[1],b[1]],color=color,lw=1.3,ls='--' if dashed else '-')
    ax.annotate('',xy=points[-1],xytext=points[-2],arrowprops=dict(arrowstyle='->',color=color,lw=1.3,linestyle='--' if dashed else '-'))
ax.text(9,11.65,'OSL hardware–simulation comparison architecture',ha='center',fontsize=20,weight='bold',color=ink)
ax.text(4,11.03,'Hardware host / Raspberry Pi',ha='center',fontsize=15,weight='bold',color='#166757')
ax.text(12.5,11.03,'Simulation computer',ha='center',fontsize=15,weight='bold',color='#254c87')
green='#e6f4ef'
box(1,9.2,6,1.2,'Telemetry HTTP server','Bounded in-memory sample buffer\nknee_telemetry.py; no database',green)
box(1,7.4,6,1.15,'Reference generator + physical knee PID','Target and encoder error → current command\nNominal control rate: 200 Hz',green)
box(1,5.6,6,1.15,'Motor driver + physical drivetrain','Current command over CAN\nMotor / transmission / belt',green)
box(1,3.8,6,1.15,'Physical knee + joint encoder','Actual knee angle via AS5048B / I²C\nTested setup: isolated knee fixture',green)
box(10,9.2,6,1.2,'Receiver + source-time buffer','HTTP polling → sequence / validity checks\nSave raw data; default live buffer: 100 ms')
box(10,7.4,6,1.15,'Independent simulation PID','Same target; own simulated feedback\n200 Hz; C or knee-error derivative')
box(10,5.6,6,1.15,'Current conversion + drivetrain model','Current → actuator dynamics / friction\nElastic belt → knee torque')
box(10,3.8,6,1.15,'MuJoCo rigid-body plant','Physics timestep: 0.5 ms (2 kHz)\nFull-leg or separate knee-only adapter')
for x in (4,13):
    for upper,lower in ((7.4,6.75),(5.6,4.95)):
        arrow([(x,upper),(x,lower)])
arrow([(4,8.55),(4,9.2)])
ax.text(4.13,8.87,'Sample publish',fontsize=9,color=ink)
arrow([(7,9.8),(10,9.8)])
ax.text(8.5,10.28,'Wi-Fi / IP',ha='center',fontsize=11,color=ink)
ax.text(8.5,9.33,'HTTP / TCP :8794\nJSON batches; token',ha='center',fontsize=9,color=ink)
arrow([(13,9.2),(13,8.55)])
ax.text(13.12,8.87,'Target + source time',fontsize=9,color=ink)
arrow([(1,4.4),(.35,4.4),(.35,7.97),(1,7.97)])
ax.text(.17,6.1,'Encoder feedback',rotation=90,ha='center',va='center',fontsize=9,color=ink)
arrow([(7,4.4),(7.5,4.4),(7.5,9.48),(7,9.48)])
ax.text(7.65,6.7,'Measured angle / validity',rotation=90,ha='center',va='center',fontsize=9,color=ink)
arrow([(16,4.4),(16.65,4.4),(16.65,7.98),(16,7.98)])
ax.text(16.85,6.25,'Simulated feedback',rotation=90,ha='center',va='center',fontsize=9,color=ink)
box(2.1,1.45,13.8,1.3,'Time-aligned comparison, plots and saved records',
    'Hardware angle + simulated angle + target; tracking errors and RMS / peak differences\nSimulation currents, torques, belt deflection; CSV / JSON records', '#f1f3f5')
arrow([(13,3.8),(13,2.75)])
arrow([(16,9.8),(17.6,9.8),(17.6,2.1),(15.9,2.1)])
ax.text(17.4,8.6,'Recorded hardware samples',rotation=90,ha='center',va='center',fontsize=9,color=ink)
ax.text(9,.95,'Timing: source timestamps drive multiple fixed physics steps per sample (~10 at 200 Hz); no Pi–simulation handshake.',ha='center',fontsize=10,color=ink)
ax.text(9,.56,'Replay: saved samples replace the network input. Model calibration is a separate offline activity.',ha='center',fontsize=10,color=ink)
ax.text(9,.17,'Measured encoder angles initialize the simulation once and provide comparison data; subsequent control uses simulated state.',ha='center',fontsize=10,color=ink)
fig.tight_layout()
for ext in ('png','svg','pdf'):
    fig.savefig(OUT/('04_paper_hardware_simulation.'+ext),dpi=190,bbox_inches='tight',facecolor='white')
plt.close(fig)
