// Execute dashboard controls with a minimal DOM/canvas, without dependencies.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let timer,requests=[],strokes=0;
function element(){return {style:{},children:[],hidden:false,clientWidth:600,textContent:'',append(x){this.children.push(x)},
 querySelector(q){if(!this.parts)this.parts={};return this.parts[q]??=(q==='canvas'?canvas():element())}}}
function canvas(){return {...element(),getContext(){return new Proxy({},{get(o,k){return k==='stroke'?()=>strokes++:()=>{}}})}}}
const ids={};for(const k of ['plots','pause','stop','joint','group','window','status','metrics','output','assumptions'])ids[k]=element();
ids.joint.value='both';ids.group.value='Comparison';ids.window.value='5';
const row={source_time_s:1,hardware_angle_deg:1,sim_angle_deg:1.1,target_deg:2,difference_deg:.1,hardware_tracking_error_deg:1,sim_tracking_error_deg:.9,joint_drive_torque_Nm:3,current_applied_A:2};
const state={source:'synthetic',status:'RUNNING',output:'test',assumptions:'explicit',metrics:{},comparison:{knee:[row],ankle:[row]},telemetry:{knee:[row],ankle:[row]}};
const context={document:{getElementById:k=>ids[k],createElement:()=>element()},devicePixelRatio:1,
 setInterval:fn=>timer=fn,fetch:async url=>{requests.push(url);return {json:async()=>state}},console};
vm.createContext(context);const html=fs.readFileSync(process.argv[2],'utf8');vm.runInContext(html.split('<script>')[1].split('</script>')[0],context);
(async()=>{await timer();assert(strokes>0);const before=strokes;ids.pause.onclick();await timer();assert.equal(strokes,before);
ids.pause.onclick();assert(strokes>before);ids.group.value='Torques';ids.group.onchange();ids.joint.value='ankle';ids.joint.onchange();
assert.equal(ids.plots.children.filter(c=>!c.hidden).length,4);ids.stop.onclick();assert(requests.includes('/key?k=stop-recording'));
assert(html.includes('Source time (s)'));console.log('Dashboard controls, stable cards, joint/group selection, labels and pause passed');})().catch(e=>{console.error(e);process.exitCode=1});
