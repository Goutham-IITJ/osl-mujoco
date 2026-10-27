// Execute the actual dashboard JavaScript against a minimal DOM; no dependencies.
const fs = require('fs'), vm = require('vm'), assert = require('assert');
let created = 0;
class Element {
  constructor(tag) { this.tag=tag; this.children=[]; this.textContent=''; this.clientWidth=400; this.width=0; this.height=0; this.dataset={}; created++; }
  append(...items) { for(const item of items) { item.parent=this; this.children.push(item); } }
  remove() { this.parent.children=this.parent.children.filter(x=>x!==this); }
  getContext() { return new Proxy({}, {get:()=>()=>{}}); }
}
const elements=new Map();
const document={getElementById(id) {if(!elements.has(id))elements.set(id,new Element('div'));return elements.get(id);},
  createElement:tag=>new Element(tag), querySelectorAll:()=>[], addEventListener:()=>{}};
const channel={value:1,unit:'rad',frame:'knee_joint',quality:'VALID',time_s:0,age_s:0,
  received_time_s:0,sequence:0,snapshot_id:0,stage:'post_step',source:'simulation',sign:'positive',points:[[0,1]]};
const state={mode:'telemetry',identity:{},channels:Object.fromEntries(Array.from({length:8},(_,i)=>['channel'+i,{...channel}]))};
const context=vm.createContext({document,devicePixelRatio:1,setInterval:()=>{},fetch:async()=>({ok:true,json:async()=>state})});
vm.runInContext(fs.readFileSync(process.argv[2],'utf8'), context);
(async()=>{
  await new Promise(resolve=>setImmediate(resolve));
  const cards=[...elements.get('plots').children], rows=[...elements.get('rows').children], count=created;
  for(let i=0;i<500;i++) {state.channels.channel0.value=i;await vm.runInContext('poll()',context);}
  assert.equal(created,count,'Updates must not recreate DOM elements');
  assert.deepStrictEqual(elements.get('plots').children,cards);
  assert.deepStrictEqual(elements.get('rows').children,rows);
  assert.equal(rows[0].children[1].textContent,Number(499).toPrecision(8));
  state.channels.channel0.quality='INVALID';await vm.runInContext('poll()',context);assert.equal(rows[0].className,'bad');
  state.channels.channel0.quality='VALID';await vm.runInContext('poll()',context);assert.equal(rows[0].className,'');
  delete state.channels.channel7;await vm.runInContext('poll()',context);assert.equal(elements.get('plots').children.length,7);
  assert.equal(elements.get('error').textContent,'');
  console.log('PASS: 500 updates preserve eight graph/row identities; values, quality and channel removal update.');
})().catch(error=>{console.error(error);process.exitCode=1;});
