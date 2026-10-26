"""Rebuild report figures from preserved simulation results; no new simulation."""
from pathlib import Path
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})

def read(case):
    with (ROOT/'build/knee_controller_selection'/case/'full_precision.csv').open() as f:
        return {k: [float(r[k]) for r in rows] for rows in [list(csv.DictReader(f))] for k in rows[0]}

old, knee = read('old_matched'), read('knee')
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
axes[0].plot(knee['time_s'], knee['reference_deg'], color='#333333', ls='--', label='AB19 knee reference')
for data, label, color in [(old,'Ankle-derived gains','#bf6d22'),(knee,'Knee gains','#176da0')]:
    axes[0].plot(data['time_s'],data['knee_angle_deg'],label=label,color=color)
    axes[1].plot(data['time_s'],data['tracking_error_deg'],color=color,label=label)
axes[0].set(ylabel='Knee angle (degrees)', title='Matched comparison: C feedback, 200 Hz, assumed 0.5701 Nm/A')
axes[0].legend(loc='upper left',ncol=3,fontsize=9)
axes[1].set(xlabel='Simulation time (s)',ylabel='Tracking error (degrees)')
for ax in axes: ax.grid(alpha=.2)
fig.savefig(OUT/'01_knee_tracking.png',dpi=180);plt.close(fig)

def box(ax,x,y,w,h,label,planned=False):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.015',
        facecolor='#fff1d9' if planned else '#eaf3f8',edgecolor='#9a702e' if planned else '#40647b',
        linestyle='--' if planned else '-',linewidth=1.3))
    ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=11)
def arrow(ax,a,b,planned=False):
    ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',color='#555555',lw=1.4,linestyle='--' if planned else '-'))

fig, ax=plt.subplots(figsize=(11,6));ax.set(xlim=(0,11),ylim=(0,6));ax.axis('off')
box(ax,.1,4.35,2.7,1.05,'Hardware controller\nTarget + encoder + current')
box(ax,3.3,4.35,2.8,1.05,'Telemetry module\nBuffered HTTP server')
box(ax,6.7,4.35,3.9,1.05,'Laptop receiver\nRecord + live hardware plot')
arrow(ax,(2.8,4.87),(3.3,4.87));arrow(ax,(6.1,4.87),(6.7,4.87))
ax.text(6.4,5.55,'Wi-Fi\nport 8794',ha='center',fontsize=10)
box(ax,3.3,1.7,3.1,1.25,'Independent MuJoCo\nShared target input\nOwn simulated feedback',True)
box(ax,7.1,1.7,3.5,1.25,'Compare knee angles\nHardware vs simulation\nError and timing',True)
arrow(ax,(7.4,4.35),(4.85,2.95),True)
ax.text(5.25,3.65,'Target + source time',ha='center',fontsize=10)
arrow(ax,(9.5,4.35),(9.5,2.95),True)
ax.text(9.65,3.65,'Encoder',fontsize=10)
arrow(ax,(6.4,2.32),(7.1,2.32),True)
ax.text(.1,.65,'Blue: implemented and tested locally     Dashed amber: next implementation stage',fontsize=11)
ax.set_title('Live telemetry architecture and planned comparison',loc='left',fontweight='bold')
fig.tight_layout();fig.savefig(OUT/'02_architecture.png',dpi=180);plt.close(fig)

fig, ax=plt.subplots(figsize=(11,4));ax.set(xlim=(0,11),ylim=(0,4));ax.axis('off')
box(ax,.1,2.5,2, .8,'Target angle')
box(ax,3,2.35,3,1.1,'Shared PID algorithm\nKnee configuration')
box(ax,7,2.35,3.7,1.1,'Current to torque\nMuJoCo drivetrain + knee')
arrow(ax,(2.1,2.9),(3,2.9));arrow(ax,(6,2.9),(7,2.9))
box(ax,2.8,.45,3.4,1.05,'P and I: knee angle\nD: actuator angle differences')
arrow(ax,(8.8,2.35),(8.8,.95));arrow(ax,(8.8,.95),(6.2,.95));arrow(ax,(4.5,1.5),(4.5,2.35))
ax.text(7.2,.35,'Simulated feedback',fontsize=10)
ax.set_title('Selected simulation controller with feedback configuration C',loc='left',fontweight='bold')
fig.tight_layout();fig.savefig(OUT/'03_control_loop.png',dpi=180);plt.close(fig)
print('Created three report figures from preserved data.')
