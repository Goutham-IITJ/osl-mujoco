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

fig, ax = canvas('Code architecture: shared algorithms, distinct responsibilities',
    'Modular implementation exists; a one-line swap to physical motor control is not implemented.')
box(ax,3,7.25,10,1.1,'Entry points / application composition\nexperiments/run_current_controlled_gait.py -> apps/current_gait.py\napps/live_knee_comparison.py -> receiver + simulation + presentation')
for x in (2.3,8,13.7): arrow(ax,(8,7.15),(x,6.35))
box(ax,.3,4.4,4.1,1.85,'CONTROL\ncontrol/current.py: shared PID\ncontrol/gait.py: knee / ankle profiles\ncontrol/feedback.py: signal selection\nDrivetrain equations + compliance')
box(ax,5.6,4.4,4.8,1.85,'SIMULATION\nsim/model.py: load bench model\nsim/current_gait.py: shared build / run\nsim/live_knee.py: source-time stepping\nsim/contracts.py: typed adapter')
box(ax,11.6,4.4,4.1,1.85,'ACQUISITION / TRANSPORT\nhardware/: existing physical PID\n+ knee_telemetry.py publisher\ntelemetry/knee.py: laptop receiver\nSeparate replay / mock / IMU modules')
arrow(ax,(4.5,5.3),(5.5,5.3)); arrow(ax,(11.5,5.3),(10.5,5.3))
box(ax,1,1.6,6.6,1.4,'PRESENTATION\nNative MuJoCo viewer + browser pages\npresentation/current_gait.py or live_knee.py\nDisplay updates do not define physics timestep')
box(ax,8.6,1.6,6.4,1.4,'CONTRACTS AND EVIDENCE\ncontracts.py: state / units / command types\nmodels/ + source URDF + tools/\ntests/ and per-run build/ CSV / JSON records')
arrow(ax,(8,4.3),(4.3,3.1)); arrow(ax,(8,4.3),(11.8,3.1))
ax.text(8,.6,'Simulation uses the shared plant. Hardware keeps its existing controller and driver; Wi-Fi is read-only.',
        ha='center',color='#d0d6df',fontsize=12)
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
