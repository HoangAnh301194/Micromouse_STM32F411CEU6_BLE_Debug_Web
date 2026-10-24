#!/usr/bin/env node
/* One-way BLE Flash Map contract regression: no GATT writes, no RX commands. */
'use strict';
const fs = require('fs'), path = require('path'), vm=require('vm');
const root = path.join(__dirname,'../..');
const html = fs.readFileSync(path.join(root,'web/ble_debug_console.html'),'utf8');
const js = html.split('<script>')[1].split('</script>')[0];
const fw = fs.readFileSync(path.join(root,'src/flash_map_diag.c'),'utf8');
new vm.Script(js);
function requireRule(value,msg){if(!value)throw new Error(msg);}
requireRule(html.includes('Monitor only · Auto map'),'Missing passive Web mode label');
requireRule(!/id="btn(?:Preview16|Preview5|RequestMap)"/.test(html),'Obsolete Web map control buttons');
requireRule(!js.includes('.writeValue'),'Web may NOT write to FFE1 in TX-only mode');
requireRule(!js.includes('requestCurrentMap('),'Web may NOT request map over BLE RX');
requireRule(js.includes('characteristic.startNotifications()'),'BLE notification subscription missing');
requireRule(js.includes("line.startsWith('MAPREADY,')"),'Missing passive heartbeat parser');
requireRule(js.includes("line.startsWith('MAPCFG,')"),'Missing passive map parser');
requireRule(js.includes("line.startsWith('MAPROW,')"),'Missing passive row parser');
requireRule(js.includes("line.startsWith('MAPEND,')"),'Missing passive CRC end parser');
requireRule(fw.includes('USART2->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE);'),
            'Diagnostic USART2 RX must be disabled');
requireRule(!fw.includes('BT_ReadByte(') && !fw.includes('BT_ReceiveChar(') &&
            !fw.includes('BT_Available('),'Unexpected inbound BLE command code');
requireRule(fw.includes('DoPreview(16U);'),'Boot should broadcast a RAM map without Flash writes');
requireRule(fw.includes('ReplayCachedMap();'),'Late Web connections need periodic replay');
requireRule(fw.includes('#define FM_REBROADCAST_MS 12000UL'),'Repeat rate contract changed');
requireRule(fw.includes('FM_REPLAY,'),'Need wired UART evidence for auto retransmissions');
console.log('PASS: passive Web and motor-locked Flash Map TX-only contracts verified');
