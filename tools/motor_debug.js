
(function(){
const st=document.createElement('style');st.textContent=`
#motorView{display:none;flex:1;min-height:0;padding:16px;gap:14px;overflow:hidden}
#motorView.open{display:flex}
#motorView aside{width:350px;min-width:300px;overflow:auto}
#motorView article{flex:1;min-width:0;display:flex;flex-direction:column;border:1px solid #30363d;border-radius:8px;overflow:hidden}
#motorView .panel{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-bottom:12px}
#motorView h3{color:#39d2c0;font-size:13px;margin-bottom:12px}
#motorView .fields{display:grid;grid-template-columns:1fr 1fr;gap:10px}
#motorView label{font-size:10px;color:#8b949e}
#motorView input[type=number],#motorView select{display:block;width:100%;margin-top:5px;padding:8px;background:#0d1117;color:#e6edf3;border:1px solid #30363d;border-radius:5px;font-family:inherit}
#motorView .actions{display:flex;gap:8px;flex-wrap:wrap;margin:14px 0}
#motorView .hint{color:#8b949e;font-size:10px;line-height:1.7}
#motorView .stats{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;padding:10px;border-bottom:1px solid #30363d}
#motorView .stat{background:#21262d;padding:8px;border-radius:5px;min-width:0}
#motorView .stat small{display:block;color:#8b949e;font-size:9px}
#motorView .stat strong{display:block;font-size:12px;color:#39d2c0;overflow-wrap:anywhere}
#motorLog{flex:1;overflow-y:auto;white-space:pre-wrap;overflow-wrap:anywhere;padding:12px;font:11px/1.65 monospace}
@media(max-width:850px){#motorView.open{flex-direction:column;overflow:auto}#motorView aside{width:100%;min-width:0}#motorView article{min-height:350px}}
`;document.head.appendChild(st);
const nav=document.getElementById('motorTabBtn');
const maze=document.createElement('button');maze.className='btn';maze.textContent='Maze Console';maze.onclick=()=>pageSwitch(false);nav.before(maze);
const page=document.createElement('section');page.id='motorView';page.innerHTML=`
<aside><div class="panel"><h3>MOTOR OPEN-LOOP TEST</h3>
<p class="hint">Lift both wheels. Motor rating unknown: use low PWM and brief tests. Full range: -100% to +100% in 0.1% steps. Duration 0 = continuous. Start with low PWM until hardware ratings are confirmed.</p>
<div class="fields">
<label>Wheel<select id="mtWheel"><option value="left">Left</option><option value="right">Right</option><option value="both">Both</option></select></label>
<label>Duration (ms; 0=continuous)<input id="mtDuration" type="number" min="0" max="4294967295" step="100" value="2000"></label>
<label>PWM Left (%)<input id="mtLeft" type="number" min="-100" max="100" step="0.1" value="15"></label>
<label>PWM Right (%)<input id="mtRight" type="number" min="-100" max="100" step="0.1" value="15"></label>
<label>Log period<select id="mtRate"><option value="100">100 ms</option><option selected value="200">200 ms</option><option value="500">500 ms</option><option value="1000">1000 ms</option></select></label>
<label>Counts/wheel rev<input id="mtPpr" type="number" min="1" max="100000" value="1430"></label>
</div>
 <div class="fields" style="margin-top:12px">
 <label>Deadband Left (0..500 ‰)<input id="mtDbL" type="number" min="0" max="500" step="1" value="0"></label>
 <label>Deadband Right (0..500 ‰)<input id="mtDbR" type="number" min="0" max="500" step="1" value="0"></label>
 <label>Rise rate (‰ / ms)<input id="mtRise" type="number" min="1" max="1000" step="1" value="20"></label>
 <label>Fall rate (‰ / ms)<input id="mtFall" type="number" min="1" max="1000" step="1" value="30"></label>
 </div><div class="actions"><button class="btn btn-ble" id="mtConnect">Connect BLE</button><button class="btn" id="mtPing">Verify firmware</button><button class="btn btn-run" id="mtRun" disabled>RUN TEST</button><button class="btn" id="mtUpdate" disabled>UPDATE PWM</button><button class="btn btn-pause" id="mtBrake" disabled>BRAKE 50ms</button><button class="btn btn-reset" id="mtStop">STOP</button></div>
<p class="hint">RUN becomes available when BLE connects. Verify firmware automatically (v3 enables fine PWM, config, brake and live PWM). Control loop remains at 1 kHz. Telemetry is generated in main loop only. Firmware heartbeat watchdog stops if browser updates stop for 1.5 s. Finite duration or STOP also ends the test.</p></div>
<div class="panel"><h3>DIAGNOSTICS</h3><p class="hint">PWM requested/applied, elapsed time, signed encoder counts, pulses/s, and RPM estimate (browser-only). Confirm counts/rev before trusting RPM.</p></div></aside>
<article><div style="display:flex;align-items:center;justify-content:space-between;gap:10px;padding:12px;background:#161b22">
<strong>MOTOR TERMINAL</strong><div style="display:flex;align-items:center;gap:8px"><label><input id="mtAuto" type="checkbox" checked> Auto-scroll</label><button class="btn" id="mtClear">Clear</button><button class="btn" id="mtExport">Export CSV</button></div></div>
<div id="mtStatus" style="padding:10px;border-bottom:1px solid #30363d;color:#d29922;font-size:11px">Disconnected</div>
<div class="stats">${['CMD L/R','APPLIED L/R','Elapsed','Enc L','Enc R','PPS L','PPS R','RPM L','RPM R'].map((name,i)=>'<div class="stat"><small>'+name+'</small><strong id="mtS'+i+'">—</strong></div>').join('')}</div>
<div id="motorLog">Connect BLE and verify firmware.\n</div></article>`;
document.querySelector('.workspace').after(page);
const $=id=>document.getElementById(id);
let ready=false,fwVersion=0,running=false,pending=false,samples=[],writeTail=Promise.resolve(),heartbeatTimer=null,ackTimeout=null;
function isConnected(){return !!(bleChar&&bleDevice&&bleDevice.gatt.connected)}
function print(t){let n=document.createElement('div');n.textContent='['+new Date().toLocaleTimeString('en-GB')+'] '+t;$('motorLog').appendChild(n);while($('motorLog').children.length>600)$('motorLog').firstChild.remove();if($('mtAuto').checked)$('motorLog').scrollTop=$('motorLog').scrollHeight}
function status(t){$('mtStatus').textContent=t}
function refresh(){$('mtRun').disabled=!isConnected()||running||pending;$('mtUpdate').disabled=!running||fwVersion<3;$('mtBrake').disabled=!running||fwVersion<3;$('mtConnect').textContent=isConnected()?'Disconnect BLE':'Connect BLE'}
function write(cmd){let job=writeTail.catch(()=>{}).then(async()=>{if(!isConnected())throw Error('BLE disconnected');let b=new TextEncoder().encode(cmd+'\n');for(let i=0;i<b.length;i+=20)await bleChar.writeValue(b.slice(i,i+20))});writeTail=job;return job}
function clearHeartbeat(){if(heartbeatTimer!==null){clearInterval(heartbeatTimer);heartbeatTimer=null}}
function heartbeat(){if(running&&isConnected())write('MT HB').catch(e=>print('Heartbeat lost: '+e.message));else clearHeartbeat()}
function startHeartbeat(){clearHeartbeat();heartbeat();heartbeatTimer=setInterval(heartbeat,400)}
function ping(){
   if(!isConnected()){print('Connect BLE first');return}
   ready=false;fwVersion=0;refresh();status('BLE connected; checking firmware protocol…');
   write('MT PING').then(()=>print('TX MT PING')).catch(e=>print('ERROR '+e.message));
   if(ackTimeout!==null)clearTimeout(ackTimeout);
   ackTimeout=setTimeout(()=>{if(!ready&&isConnected())status('No MTREADY received. Check uploaded firmware, JDY-33 RX wiring and BLE logs; RUN can still send a command for diagnosis.')},2200)
  }
function permille(id){const val=Number($(id).value);const p=Math.round(val*10);if(!Number.isFinite(val)||p < -1000 || p > 1000 || Math.abs(val*10-p)>0.00001)throw Error(id+' must be within ±100% with 0.1% steps');return p}
function cfg(){return [num('mtDbL',0,500),num('mtDbR',0,500),num('mtRise',1,1000),num('mtFall',1,1000)]}
function num(id,min,max){let n=Number($(id).value);if(!Number.isInteger(n)||n<min||n>max)throw Error(id+' must be an integer between '+min+' and '+max);return n}
async function run(){
 if(!isConnected()||running||pending)return;
 try{
  let l=permille('mtLeft'),r=permille('mtRight');
  const dur=num('mtDuration',0,4294967295),rate=num('mtRate',100,1000);
  num('mtPpr',1,100000);
  const [dl,dr,rise,fall]=cfg();
  if($('mtWheel').value==='left')r=0;
  if($('mtWheel').value==='right')l=0;
  if(!l&&!r)throw Error('PWM must be nonzero');
  pending=true;refresh();status('Sending RUN; waiting for firmware ACK…');
  if(fwVersion>=3){
    await write('MT CFG '+dl+' '+dr+' '+rise+' '+fall);
    await write('MT RUNP '+l+' '+r+' '+dur+' '+rate);
  }else if(fwVersion===2){
    await write('MT RUN '+Math.round(l/10)+' '+Math.round(r/10)+' '+dur+' '+rate);
  }else {
    print('Firmware unverified: sending MT PING and MT RUNP for diagnosis');
    await write('MT PING');
    await write('MT RUNP '+l+' '+r+' '+dur+' '+rate);
  }
  print('TX RUN requested L='+l/10+'% R='+r/10+'% duration='+dur+'ms period='+rate+'ms');
  if(ackTimeout!==null)clearTimeout(ackTimeout);
  ackTimeout=setTimeout(()=>{if(pending){pending=false;refresh();status('RUN not acknowledged. Flash current firmware (MTREADY,3) and inspect BLE/UART RX logs.');print('RUN ACK TIMEOUT — no motor test confirmed')}},2500);
 }catch(e){pending=false;refresh();print('ERROR '+e.message)}
}
async function updatePWM(){
 if(!running||fwVersion<3)return;
 try{let l=permille('mtLeft'),r=permille('mtRight');if($('mtWheel').value==='left')r=0;if($('mtWheel').value==='right')l=0;await write('MT SETP '+l+' '+r);print('TX SETP '+l+' '+r)}catch(e){print('UPDATE ERROR '+e.message)}
}
async function brake(){
 if(!running||fwVersion<3)return;
 try{await write('MT BRAKE');print('TX BRAKE 50ms then coast')}catch(e){print('BRAKE ERROR '+e.message)}
}
function stop(){if(!isConnected()){print('STOP unavailable (disconnected); firmware timeout applies');return}write('MT STOP').then(()=>print('TX STOP')).catch(e=>print('ERROR '+e.message))}
function process(line){
 if(line.startsWith('MTREADY,')){
   fwVersion=Number(line.slice(8))||0;ready=true;
   if(ackTimeout!==null)clearTimeout(ackTimeout);
   status('Firmware v'+fwVersion+' ready'+(fwVersion<3?' (legacy: no fine PWM/ramp config)':''));
   print(line);
 }else if(line.startsWith('MTACK,RUN')){
   running=true;pending=false;status('RUNNING · heartbeat active');print(line);startHeartbeat();
   if(ackTimeout!==null)clearTimeout(ackTimeout);
 }else if(line.startsWith('MTACK,CFG,')||line.startsWith('MTACK,SETP,')||line==='MTACK,BRAKE'||line==='MTACK,STOP'){
   print(line);
 }else if(line.startsWith('MTERR,')){
   if(pending){pending=false;running=false}
   status('Firmware: '+line);print(line);
 }else if(line.startsWith('MTEND,')){
   running=false;pending=false;clearHeartbeat();
   status('FINISHED: '+line.slice(6));print(line);
 }else if(line.startsWith('MTDATA3,')){
   const a=line.slice(8).split(',').map(Number);
   if(a.length!==10||a.some(x=>!Number.isFinite(x))){print('Malformed '+line);return}
   samples.push(a);if(samples.length>2500)samples.shift();
   const p=Number($('mtPpr').value),v=[
     a[1]/10+' / '+a[2]/10+'%',a[3]/10+' / '+a[4]/10+'%',
     a[0]+'ms',a[5],a[6],a[7],a[8],
     p>0?(a[7]*60/p).toFixed(1):'—',p>0?(a[8]*60/p).toFixed(1):'—'];
   v.forEach((x,i)=>$('mtS'+i).textContent=x);
   print(line);
 }else if(line.startsWith('MTDATA,')){
   const a=line.slice(7).split(',').map(Number);
   if(a.length!==8||a.some(x=>!Number.isFinite(x))){print('Malformed '+line);return}
   // Legacy v2 telemetry; applied PWM unavailable.
   const p=Number($('mtPpr').value),v=[a[1]+' / '+a[2]+'%','—',
     a[0]+'ms',a[3],a[4],a[5],a[6],
     p>0?(a[5]*60/p).toFixed(1):'—',p>0?(a[6]*60/p).toFixed(1):'—'];
   v.forEach((x,i)=>$('mtS'+i).textContent=x);
   samples.push([a[0],a[1]*10,a[2]*10,NaN,NaN,a[3],a[4],a[5],a[6],a[7]]);
   print(line);
 }
 refresh();
}
function pageSwitch(open){if(!open&&(running||pending)){stop();clearHeartbeat();}page.classList.toggle('open',open);document.querySelector('.workspace').style.display=open?'none':'flex';if(!open){sizeCanvas();renderMaze()}else if(isConnected()&&!ready)ping();refresh()}
window.motorSwitchPage=()=>pageSwitch(!page.classList.contains('open'));
$('mtConnect').onclick=async()=>{if(isConnected()&&(running||pending)){try{await write('MT STOP')}catch(e){print('STOP send failed: '+e.message)}clearHeartbeat()}toggleConnection()};$('mtPing').onclick=ping;$('mtRun').onclick=run;$('mtUpdate').onclick=updatePWM;$('mtBrake').onclick=brake;$('mtStop').onclick=stop;
$('mtClear').onclick=()=>{$('motorLog').textContent='';samples=[]};
$('mtExport').onclick=()=>{let csv=['elapsed_ms,requested_left_permille,requested_right_permille,applied_left_permille,applied_right_permille,encoder_left,encoder_right,pps_left,pps_right,dt_ms',...samples.map(a=>a.join(','))].join('\n'),u=URL.createObjectURL(new Blob([csv],{type:'text/csv'})),a=document.createElement('a');a.href=u;a.download='motor_'+Date.now()+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(u),1000)};
const oldLog=addBLEEntry;addBLEEntry=function(line,type){oldLog(line,type);if(/^MT(READY|ACK|ERR|END|DATA3|DATA),/.test(line))process(line)};
document.addEventListener('visibilitychange',()=>{if(document.hidden&&(running||pending)){stop();clearHeartbeat();print('Background tab: STOP requested')}});
const oldDisconnect=onBLEDisconnected;onBLEDisconnected=function(){oldDisconnect();clearHeartbeat();ready=false;fwVersion=0;running=false;pending=false;status('BLE disconnected; firmware timeout applies');refresh();print('Disconnected')};
// Original BLE connect handler does not automatically ping from Motor tab.
// Wrap connection completion to keep RUN discoverable and show protocol errors.
const originalBleConnect=bleConnect;
bleConnect=async function(...args){
 await originalBleConnect(...args);
 if(isConnected()){
   bleDevice.addEventListener('gattserverdisconnected',()=>{
     clearHeartbeat();ready=false;fwVersion=0;running=false;pending=false;
     status('BLE disconnected; MCU heartbeat should stop test within 1.5s');
     refresh();
   },{once:true});
   print('BLE connected');
   ping();
 }
};
refresh();
})();