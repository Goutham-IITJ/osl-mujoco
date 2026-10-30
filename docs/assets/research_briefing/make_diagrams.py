"""Rebuild the meeting diagrams; no simulation or baseline files are modified."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

OUT = Path(__file__).resolve().parent
BG, BLUE, TEXT, EDGE = '#17191c', '#052f53', '#acd6ff', '#497b9c'

def canvas(title, subtitle):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG)
    ax.set(xlim=(0, 16), ylim=(0, 10)); ax.axis('off')
    ax.text(8, 9.65, title, ha='center', color=TEXT, fontsize=23, weight='bold')
    ax.text(8, 9.13, subtitle, ha='center', color='#d0d6df', fontsize=12)
    return fig, ax

def box(ax, x, y, w, h, text):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.08,rounding_size=.17',
                              facecolor=BLUE, edgecolor=EDGE, linewidth=1.5))
    ax.text(x+w/2,y+h/2,text,ha='center',va='center',color=TEXT,fontsize=12,linespacing=1.5)

def arrow(ax, a, b, label='', dashed=False):
    ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',color='#b0b9c3',lw=1.7,
                                             linestyle='--' if dashed else '-'))
    if label:
        ax.text((a[0]+b[0])/2+.12,(a[1]+b[1])/2+.14,label,color='#d0d6df',fontsize=11,
                ha='left',bbox=dict(facecolor=BG,edgecolor='none',pad=2))

def save(fig, name):
    fig.savefig(OUT/(name+'.png'),dpi=170,bbox_inches='tight',facecolor=BG)
    fig.savefig(OUT/(name+'.svg'),bbox_inches='tight',facecolor=BG)
    plt.close(fig)

fig, ax = canvas('Knee control: where the drivetrain enters',
    'Current-domain path shown here. Feedback configuration C is a separate choice within this path.')
box(ax,.4,7.1,4.0,1.25,'Target knee angle\nAB19 offline OR streamed target')
box(ax,5.5,7.1,4.3,1.25,'Knee PID at 200 Hz\ncontrol/gait.py + current.py\nP = 1.5, I = 0.15, D = 0.06')
box(ax,11,7.1,4.4,1.25,'Requested / clipped current\nCurrent-loop model + conversion\nA -> actuator-output torque')
arrow(ax,(4.5,7.7),(5.4,7.7)); arrow(ax,(9.9,7.7),(10.9,7.7))
box(ax,10.8,4.25,4.6,1.5,'DrivetrainLayer\nActuator inertia + friction\nNonlinear elastic belt\ncontrol/compliance.py + drivetrain.py')
arrow(ax,(13.2,7),(13.2,5.85))
box(ax,5.25,4.25,4.5,1.5,'DrivetrainBenchSimulation\nBelt joint torque -> qfrc_applied\nMuJoCo step: 0.5 ms\nsim/compliant.py')
arrow(ax,(10.7,5),(9.85,5))
box(ax,.4,4.25,3.8,1.5,'CAD-derived bench MJCF\nRigid-body motion + gravity\nIndependent simulated angle\nmodels/osl_v2_bench.xml')
arrow(ax,(5.15,5),(4.3,5))
box(ax,3.4,1.45,8.8,1.4,'Feedback C: P/I use simulated knee angle\nD uses target differences minus actuator-angle differences / belt ratio\ncontrol/feedback.py: ActuatorDifferenceSource')
arrow(ax,(2.3,4.15),(2.3,2.15)); arrow(ax,(2.3,2.15),(3.3,2.15))
arrow(ax,(13.1,4.15),(13.1,2.15)); arrow(ax,(13.1,2.15),(12.3,2.15))
arrow(ax,(4.8,2.95),(4.8,6.65),dashed=True)
arrow(ax,(4.8,6.65),(7.5,6.65),dashed=True)
arrow(ax,(7.5,6.65),(7.5,7),dashed=True)
ax.text(8,.55,'Feedback returns to the PID above. The original knee position servo is disabled in this path.',
        ha='center',color='#d0d6df',fontsize=12)
save(fig,'01_control_flow')

fig, ax = canvas('OSL software architecture',
    'A responsibility map: what each module owns and where it runs')

def panel(x, y, w, h, title, detail, path, color=BLUE):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.02,rounding_size=.12',
                              facecolor=color,edgecolor=EDGE,linewidth=1.2))
    ax.text(x+.22,y+h-.33,title,color='#e2f1ff',fontsize=15,weight='bold',va='top')
    ax.text(x+.22,y+h-.83,detail,color=TEXT,fontsize=11.5,va='top',linespacing=1.6)
    ax.text(x+.22,y+.25,path,color='#82accc',fontsize=10,va='bottom',family='monospace')

ax.add_patch(FancyBboxPatch((.25,1.05),11.25,7.5,boxstyle='round,pad=.05,rounding_size=.16',
                          facecolor='#10212e',edgecolor='#3d617b',linewidth=1.5))
ax.text(.55,8.22,'LAPTOP  /  oslbench',color=TEXT,fontsize=12,weight='bold')
panel(.55,6.4,10.65,1.48,'01  APPLICATIONS','Select a run: offline gait simulation or live hardware comparison','apps/  +  experiments/')
ax.text(.65,6.05,'02  REUSABLE MODULES',color='#9cb3c5',fontsize=10,weight='bold')
panel(.55,3.65,3.35,2.2,'Control & dynamics','Shared PID\nKnee / ankle settings\nActuator + belt equations','control/')
panel(4.2,3.65,3.35,2.2,'Simulation','Model loading\nPhysics stepping\nIndependent knee state','sim/')
panel(7.85,3.65,3.35,2.2,'Telemetry','Receive + validate\nLive hardware samples\nReplay / mock sources','telemetry/')
panel(.55,1.4,10.65,1.82,'03  PRESENTATION','MuJoCo viewer  |  Angle comparison  |  Simulation diagnostics\nApplication records CSV / JSON alongside the display','presentation/')
panel(12.25,3.65,3.5,4.23,'HARDWARE HOST','Existing knee controller\n        +\nMotor / encoder access\n        +\nTelemetry publisher','hardware/','#153445')
arrow(ax,(12.15,4.65),(11.3,4.65))
ax.text(11.73,5.2,'Wi-Fi',color=TEXT,ha='center',fontsize=10)
ax.text(11.73,4.15,'read-only',color='#b3c3ce',ha='center',fontsize=8)
ax.text(12.35,2.95,'Physical control stays\non the hardware host.',color=TEXT,fontsize=12,linespacing=1.6)
ax.text(.55,.65,'SHARED FOUNDATION',color='#9cb3c5',fontsize=10,weight='bold')
ax.text(3.35,.65,'State / units / commands    |    Model assets    |    Regression tests',color=TEXT,fontsize=12)
ax.text(8,.12,'Layers group responsibilities; they are not a time sequence. Hardware command-backend swapping remains future work.',
        ha='center',color='#b3c3ce',fontsize=10)
save(fig,'02_code_architecture')

fig, ax = canvas('Live comparison: one target, two independent responses',
    'Hardware sends existing controller variables. The laptop sends no motor commands.')
box(ax,.4,7.15,4.5,1.3,'HARDWARE COMPUTER / Pi\nExisting step or gait target generator\nExisting knee PID -> motor driver')
box(ax,6,7.15,4.3,1.3,'Physical knee + encoder\nMeasured angle returns to\nthe physical controller')
arrow(ax,(5,7.8),(5.9,7.8))
box(ax,11.3,7.15,4.3,1.3,'knee_telemetry.py\nTarget + encoder + currents\nSource time + sequence + validity')
arrow(ax,(10.4,7.8),(11.2,7.8))
box(ax,10.6,4.6,5,1.3,'LAPTOP RECEIVER\nRead-only HTTP, port 8794\nValidate + buffer + record stream')
arrow(ax,(13.2,7.05),(13.2,6),'Wi-Fi + token')
box(ax,.4,4.3,7.2,1.85,'Independent simulation\nStreamed TARGET -> knee PID -> drivetrain -> MuJoCo\nController uses its own simulated feedback\nEncoder initializes angle once, then only comparison')
arrow(ax,(10.5,5.25),(7.7,5.25),'target')
box(ax,.4,1.35,4.6,1.5,'Native MuJoCo viewer\n--view\nShows simulated motion\nSame running model')
box(ax,5.6,1.35,4.8,1.5,'Comparison page: 8796\nHardware + simulation angle\nOptional target, separate errors\nRMS / peak difference')
box(ax,11,1.35,4.6,1.5,'Simulation telemetry: 8797\nTorques, belt deformation\nCurrent, PID, motion\nSeparate graphs + CSV logs')
arrow(ax,(2.7,4.2),(2.7,2.95)); arrow(ax,(6.2,4.2),(8,2.95))
arrow(ax,(13.1,4.5),(8.2,2.95),dashed=True)
ax.text(9,3.4,'encoder',color='#d0d6df',fontsize=11,
        bbox=dict(facecolor=BG,edgecolor='none',pad=2))
arrow(ax,(7.7,4.35),(13.2,2.95),dashed=True)
ax.text(11.9,3.65,'simulation logs',color='#d0d6df',fontsize=11,
        bbox=dict(facecolor=BG,edgecolor='none',pad=2))
ax.text(8,.5,'Step test first; gait later. Current C feedback differs from hardware: residual includes controller + plant differences.',
        ha='center',color='#d0d6df',fontsize=12)
save(fig,'03_live_pipeline')
