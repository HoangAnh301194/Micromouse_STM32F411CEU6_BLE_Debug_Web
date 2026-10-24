#!/usr/bin/env node
/* Verify the FFE1 Flash Map request, previews, bounded retry; no BLE hardware. */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const html = fs.readFileSync(path.join(__dirname,'../../web/ble_debug_console.html'),'utf8');
const script = html.split('<script>')[1].split('</script>')[0];
const start = script.indexOf('// Flash Map diagnostic protocol:');
const end = script.indexOf('function bleDisconnect()',start);
if (start < 0 || end <= start) throw Error('Map control source not found');
const sent = [], timers = new Map(), logs = [], statuses = [];
let timerId = 0;
const ctx = vm.createContext({
    Uint8Array, sent,
    mapStatus: s => statuses.push(s),
    mapError: s => statuses.push('ERROR '+s),
    addBLEEntry: (s,t) => logs.push([s,t]),
    setTimeout: (fn,ms) => {
        if(ms < 5000 || ms > 15000) throw Error('unexpected timeout '+ms);
        const id=++timerId;
        timers.set(id,fn);
        return id;
    },
    clearTimeout: id => timers.delete(id),
});
vm.runInContext(`let bleChar = {
  properties: {writeWithoutResponse:true,write:true},
  async writeValueWithoutResponse(bytes) { sent.push(bytes[0]); },
  async writeValueWithResponse() { throw Error('must prefer without-response'); }
};
${script.slice(start,end)}`,ctx);
const execute=expression=>vm.runInContext(expression,ctx);
const tick=async()=>{ await new Promise(resolve=>setImmediate(resolve)); };
function fireTimer(){
    const next=timers.entries().next().value;
    if(!next)throw Error('no pending timeout');
    const [id,callback]=next;
    timers.delete(id); // Fired browser timers are not pending anymore.
    callback();
}
async function main(){
    await execute('requestCurrentMap()');
    await execute('requestPreviewMap16()');
    await execute('requestPreviewMap5()');
    if (sent.join(',') !== '63,86,118') throw Error('wrong GATT command bytes '+sent);
    if(timers.size!==1)throw Error('map request timeout not armed');
    fireTimer();
    await tick();
    if(sent.at(-1)!==118 || sent.length!==4)throw Error('retry changed requested mode');
    if(timers.size!==1)throw Error('retry did not rearm');
    fireTimer();
    await tick();
    if(sent.length!==5)throw Error('second retry missing');
    fireTimer();
    await tick();
    if(sent.length!==5 || !statuses.at(-1).startsWith('ERROR no complete'))
        throw Error('bounded 3-attempt fail state incorrect');
    if(!logs.some(([s])=>s.includes('retrying')))throw Error('retry not logged');
    await execute('requestCurrentMap()');
    execute('finishMapRequest()');
    if(timers.size || execute('flashMapWaiting'))throw Error('successful map did not clear timer');
    console.log('PASS: GATT read/preview, write mode, 3-attempt bound, retry and completion');
}
main().catch(e=>{console.error(e.stack);process.exitCode=1;});
