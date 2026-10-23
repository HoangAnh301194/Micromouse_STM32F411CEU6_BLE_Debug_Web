#!/usr/bin/env node
/* Offline browser protocol regression without browser/BLE hardware. */
'use strict';
const fs = require('fs'), path = require('path'), vm=require('vm');
const html=fs.readFileSync(path.join(__dirname,'ble_debug_console.html'),'utf8');
const script=html.split('<script>')[1].split('</script>')[0];
new vm.Script(script);  // Validate entire browser source syntax.
const start=script.indexOf('/* Versioned map protocol:');
const end=script.indexOf('/* Chunked protocol:',start);
if(start<0||end<start) throw Error('Missing dynamic Flash map parser');
const elements={};
function element(id){return elements[id] ||= {textContent:'',style:{}}}
const state={size:16,rendered:0,log:[]};
const sandbox={
    Uint8Array, Set, Math, Number, parseInt, RegExp,
    document:{getElementById:element},
    setMazeSize:n=>{state.size=n;},
    addBLEEntry:(s,t)=>state.log.push({s,t}),
    renderMaze:()=>{state.rendered++},
    calcFlood:()=>[[0]],
    setRobotPosition:()=>{},
    mazeWalls:null, mazeFlood:null,
    visitedCells:new Set(),
};
const context=vm.createContext(sandbox);
vm.runInContext(script.slice(start,end),context);
const exec=(expression)=>vm.runInContext(expression,context);
function mk(n) {
    let a=[77,65,80,49,n,n,0,0];
    for(let y=0;y<n;y++)for(let x=0;x<n;x++){
        let w=0;
        if(y===n-1||(3*x+5*(y+1))%11===0)w|=1;
        if(x===n-1||(7*(x+1)+2*y)%13===0)w|=2;
        if(y===0||(3*x+5*y)%11===0)w|=4;
        if(x===0||(7*x+2*y)%13===0)w|=8;
        a.push(w);
    }
    return a;
}
function checksum(a) {
    let c=0xFFFFFFFF;
    for(const b of a){c^=b;for(let k=0;k<8;k++) c=(c>>>1)^((c&1)?0xEDB88320:0);}
    return ((c^0xFFFFFFFF)>>>0).toString(16).toUpperCase().padStart(8,'0');
}
function stream(n,mutate=false,drop=false){
    const data=mk(n),crc=checksum(data);
    exec('parseFlashMapConfig('+JSON.stringify('MAPCFG,w='+n+',h='+n+',fmt=1,len='+data.length+',crc='+crc+',src=FLASH')+')');
    for(let y=0;y<n;y++){
        if(drop&&y===n-1)continue;
        let hex=data.slice(8+y*n,8+(y+1)*n).map(x=>x.toString(16).toUpperCase()).join('');
        if(mutate&&y===1)hex=hex.slice(0,-1)+(hex.endsWith('0')?'1':'0');
        exec('parseFlashMapRow('+JSON.stringify('MAPROW,y='+y+',hex='+hex)+')');
    }
    exec('parseFlashMapEnd('+JSON.stringify('MAPEND,crc='+crc+',rows='+n)+')');
}
stream(16);
if(state.size!==16 || state.rendered!==1 || exec('lastFlashMap.status')!=='PASS')
    throw Error('16x16 rejected');
stream(5);
if(state.size!==5 || state.rendered!==2 || exec('lastFlashMap.size')!==5)
    throw Error('5x5 auto-resize failed');
const saved=state.rendered;
stream(16,true);
if(state.rendered!==saved || !element('mapVerifyStatus').textContent.includes('FAIL'))
    throw Error('CRC corrupted row was rendered');
stream(5,false,true);
if(state.rendered!==saved)throw Error('Missing row was rendered');
console.log('PASS: browser JS syntax, 16/5 auto-size, valid CRC render, corrupt/missing rejection');
