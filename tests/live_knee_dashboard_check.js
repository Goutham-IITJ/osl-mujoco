// Run the actual UI script against a small DOM to test display-only controls.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const drawn=[];let draws=0,fetches=0,confirmStop=false,stopCalls=0;
class Element{
 constructor(tag){this.style={};this.tag=tag;this.children=[];this.value='';this.hidden=false;this.clientWidth=600;this.width=0;this.height=0;this.textContent='';}
 append(...children){this.children.push(...children);}
 setAttribute(){}
 getContext(){return new Proxy({}, {get:(obj,key)=>key==='fillText'?(text)=>{drawn.push(text);draws++;}:()=>{}});}
}
const ids=new Map();const get=id=>{if(!ids.has(id))ids.set(id,new Element('div'));return ids.get(id);};
get('window').value='5';get('category').value='Torques';
let snapshot={output_dir:'build/test',status:'LIVE',source:'synthetic',session:'test',metrics:{valid_pairs:2,RMS_sim_minus_hardware_deg:1,peak_abs_sim_minus_hardware_deg:2},comparison:[{source_time_s:0,hardware_angle_deg:10,sim_angle_deg:11,target_deg:12,difference_deg:1},{source_time_s:1,hardware_angle_deg:20,sim_angle_deg:21,target_deg:22,difference_deg:1}],telemetry:[{source_time_s:0,knee_drive_torque_Nm:2,actuator_torque_Nm:1,belt_actuator_torque_Nm:.5,friction_torque_Nm:.1},{source_time_s:1,knee_drive_torque_Nm:3,actuator_torque_Nm:2,belt_actuator_torque_Nm:1,friction_torque_Nm:.2}]};
const context=vm.createContext({document:{getElementById:get,createElement:tag=>new Element(tag),createTextNode:text=>({textContent:text})},
 window:{devicePixelRatio:1,addEventListener:()=>{},confirm:()=>confirmStop},setInterval:()=>{},
 fetch:async path=>{fetches++;if(path.includes('/key'))stopCalls++;return {ok:true,json:async()=>JSON.parse(JSON.stringify(snapshot))};}});
vm.runInContext(fs.readFileSync(process.argv[2],'utf8'),context);
(async()=>{
 await new Promise(r=>setImmediate(r));
 assert(drawn.includes('Source time (s)'),'X axis label');
 assert(drawn.includes(process.argv[3]==='comparison'?'Angle / error (degrees)':'Torque (Nm)'),'Y axis units');
 const cards=[...get('plots').children];
 get('pause').onclick();const frozen=draws;
 snapshot.metrics.valid_pairs=3;await vm.runInContext('poll()',context);
 assert.equal(draws,frozen,'Paused display must not redraw');assert(fetches>=2,'Polling continues while paused');
 assert(get('status').textContent.includes('DISPLAY PAUSED'));
 get('pause').onclick();assert(draws>frozen);assert(get('metrics').textContent.includes('3'));
 assert.deepStrictEqual(get('plots').children,cards,'No graph rebuilding');
 if(process.argv[3]==='telemetry'){
  assert.equal(cards.filter(c=>!c.hidden).length,4,'Four separate torque plots');
  get('category').value='PID';get('category').onchange();assert.equal(cards.filter(c=>!c.hidden).length,3);
 }
 await get('stop').onclick();assert.equal(stopCalls,0);
 confirmStop=true;await get('stop').onclick();assert.equal(stopCalls,1);assert(get('stop').disabled);
 console.log('PASS axes, stable nodes, pause/resume, categories and explicit stop');
})().catch(e=>{console.error(e);process.exitCode=1;});

