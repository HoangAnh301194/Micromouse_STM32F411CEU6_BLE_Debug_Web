
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
<p class="hint">Lift both wheels. Motor rating unknown: use low PWM and brief tests. Firmware limit: ±30%, 100–1000 ms.</p>
<div class="fields">
<label>Wheel<select id="mtWheel"><option value="left">Left</option><option value="right">Right</option><option value="both">Both</option></select></label>
<label>Duration (ms)<input id="mtDuration" type="number" min="100" max="1000" value="400"></label>
<label>PWM Left (%)<input id="mtLeft" type="number" min="-30" max="30" value="15"></label>
<label>PWM Right (%)<input id="mtRight" type="number" min="-30" max="30" value="15"></label>
<label>Log period<select id="mtRate"><option value="100">100 ms</option><option selected value="200">200 ms</option><option value="500">500 ms</option><option value="1000">1000 ms</option></select></label>
<label>Counts/wheel rev<input id="mtPpr" type="number" min="1" max="100000" value="1430"></label>
</div><div class="actions"><button class="btn btn-ble" id="mtConnect">Connect BLE</button><button class="btn" id="mtPing">Verify firmware</button><button class="btn btn-run" id="mtRun" disabled>RUN TEST</button><button class="btn btn-reset" id="mtStop">STOP</button></div>
<p class="hint">RUN requires firmware MTREADY. Control loop remains at 1 kHz. Telemetry is generated in main loop only. Firmware stops by elapsed-time deadline even if BLE disconnects.</p></div>
<div class="panel"><h3>DIAGNOSTICS</h3><p class="hint">PWM, elapsed time, signed encoder counts, pulses/s, and RPM estimate (browser-only). Confirm counts/rev before trusting RPM.</p></div></aside>
<article><div style="display:flex;align-items:center;justify-content:space-between;gap:10px;padding:12px;background:#161b22">
<strong>MOTOR TERMINAL</strong><div style="display:flex;align-items:center;gap:8px"><label><input id="mtAuto" type="checkbox" checked> Auto-scroll</label><button class="btn" id="mtClear">Clear</button><button class="btn" id="mtExport">Export CSV</button></div></div>
<div id="mtStatus" style="padding:10px;border-bottom:1px solid #30363d;color:#d29922;font-size:11px">Disconnected</div>
<div class="stats">${['PWM L/R','Elapsed','Enc L','Enc R','PPS L','PPS R','RPM L','RPM R'].map((name,i)=>'<div class="stat"><small>'+name+'</small><strong id="mtS'+i+'">—</strong></div>').join('')}</div>
<div id="motorLog">Connect BLE and verify firmware.\n</div></article>`;
document.querySelector('.workspace').after(page);
const $=id=>document.getElementById(id);
let ready=false,running=false,pending=false,samples=[],writeTail=Promise.resolve();
function isConnected(){return !!(bleChar&&bleDevice&&bleDevice.gatt.connected)}
function print(t){let n=document.createElement('div');n.textContent='['+new Date().toLocaleTimeString('en-GB')+'] '+t;$('motorLog').appendChild(n);while($('motorLog').children.length>600)$('motorLog').firstChild.remove();if($('mtAuto').checked)$('motorLog').scrollTop=$('motorLog').scrollHeight}
function status(t){$('mtStatus').textContent=t}
function refresh(){$('mtRun').disabled=!ready||!isConnected()||running||pending;$('mtConnect').textContent=isConnected()?'Disconnect BLE':'Connect BLE'}
function write(cmd){let job=writeTail.catch(()=>{}).then(async()=>{if(!isConnected())throw Error('BLE disconnected');let b=new TextEncoder().encode(cmd+'\n');for(let i=0;i<b.length;i+=20)await bleChar.writeValue(b.slice(i,i+20))});writeTail=job;return job}
function ping(){if(!isConnected()){print('Connect BLE first');return}ready=false;refresh();write('MT PING').then(()=>print('TX MT PING')).catch(e=>print('ERROR '+e.message))}
function num(id,min,max){let n=Number($(id).value);if(!Number.isInteger(n)||n<min||n>max)throw Error(id+' must be an integer between '+min+' and '+max);return n}
async function run(){if(!ready||running||pending||!isConnected())return;try{let l=num('mtLeft',-30,30),r=num('mtRight',-30,30),dur=num('mtDuration',100,1000),rate=num('mtRate',100,1000);num('mtPpr',1,100000);if($('mtWheel').value==='left')r=0;if($('mtWheel').value==='right')l=0;if(!l&&!r)throw Error('PWM must be nonzero');pending=true;refresh();await write('MT RUN '+l+' '+r+' '+dur+' '+rate);print('TX RUN '+l+' '+r+' '+dur+' '+rate)}catch(e){pending=false;refresh();print('ERROR '+e.message)}}
function stop(){if(!isConnected()){print('STOP unavailable (disconnected); firmware timeout applies');return}write('MT STOP').then(()=>print('TX STOP')).catch(e=>print('ERROR '+e.message))}
function process(line){if(line==='MTREADY,1'){ready=true;status('Firmware ready');print(line)}
else if(line.startsWith('MTACK,RUN,')){running=true;pending=false;status('RUNNING');print(line)}
else if(line==='MTACK,STOP'){print(line)}
else if(line.startsWith('MTERR,')){running=false;pending=false;status('Command rejected');print(line)}
else if(line.startsWith('MTEND,')){running=false;pending=false;status('FINISHED: '+line.slice(6));print(line)}
else if(line.startsWith('MTDATA,')){let a=line.slice(7).split(',').map(Number);if(a.length!==8||a.some(x=>!Number.isFinite(x))){print('Malformed '+line);return}samples.push(a);if(samples.length>2500)samples.shift();let p=Number($('mtPpr').value),values=[a[1]+'/'+a[2]+'%',a[0]+'ms',a[3],a[4],a[5],a[6],p>0?(a[5]*60/p).toFixed(1):'—',p>0?(a[6]*60/p).toFixed(1):'—'];values.forEach((v,i)=>$('mtS'+i).textContent=v);print(line)}refresh()}
function pageSwitch(open){if(!open&&(running||pending))stop();page.classList.toggle('open',open);document.querySelector('.workspace').style.display=open?'none':'flex';if(!open){sizeCanvas();renderMaze()}else if(isConnected()&&!ready)ping();refresh()}
window.motorSwitchPage=()=>pageSwitch(!page.classList.contains('open'));
$('mtConnect').onclick=()=>toggleConnection();$('mtPing').onclick=ping;$('mtRun').onclick=run;$('mtStop').onclick=stop;
$('mtClear').onclick=()=>{$('motorLog').textContent='';samples=[]};
$('mtExport').onclick=()=>{let csv=['elapsed_ms,pwm_left,pwm_right,encoder_left,encoder_right,pps_left,pps_right,dt_ms',...samples.map(a=>a.join(','))].join('\n'),u=URL.createObjectURL(new Blob([csv],{type:'text/csv'})),a=document.createElement('a');a.href=u;a.download='motor_'+Date.now()+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(u),1000)};
const oldLog=addBLEEntry;addBLEEntry=function(line,type){oldLog(line,type);if(/^MT(READY|ACK|ERR|END|DATA),/.test(line))process(line)};
const oldDisconnect=onBLEDisconnected;onBLEDisconnected=function(){oldDisconnect();ready=false;running=false;pending=false;status('BLE disconnected; firmware timeout applies');refresh();print('Disconnected')};
refresh();
})();