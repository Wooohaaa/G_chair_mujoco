// ui_check.js — 在 Node 中用最小 DOM stub 执行 host_app.html 的 update()，
// 验证 Web 界面不会因 TDZ/异常而停止渲染（node 不可用则 preflight 跳过）。
const fs = require('fs');
const path = require('path');

const root = path.join(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'host_app.html'), 'utf8');
const m = html.match(/<script>([\s\S]*?)<\/script>/);
if (!m) { console.error('未找到 <script>'); process.exit(1); }
const js = m[1].replace(/WS_PORT/g, '8766');

const els = {};
const ctx = {
  clearRect(){}, beginPath(){}, moveTo(){}, lineTo(){}, stroke(){},
  setLineDash(){}, strokeStyle:'', lineWidth:0, globalAlpha:1
};
global.location = { hostname: 'localhost' };
global.document = {
  getElementById(id) {
    if (!els[id]) els[id] = {
      innerHTML:'', textContent:'', style:{}, addEventListener(){}, value:'110',
      width:700, height:200, getContext(){ return ctx; }
    };
    return els[id];
  }
};
global.WebSocket = function(){ this.readyState=0; this.send=()=>{}; };

try { eval(js); } catch (e) { console.error('脚本加载失败:', e.message); process.exit(1); }

const base = {
  mode:'real', connected:true, frames:100, lost:0, pose:[0,0,0,0.1,0,0],
  disp_mm:[1,2,3,4,5,6], speed:[10,20,30,40,50,60], recording:false,
  manual_disp:null, hw_age:0.05, tele_stale:0,
  params:{lpf:0.15,washout:2.5,max_speed:110,roll:35,pitch:30,yaw:10,sway:100},
  hw:{limit:0,watchdog:0,latch:0,estop:0,enabled:1,frames:60,
      cur:[19200,19200,19200,19200,19200,19200],
      tgt:[19200,19200,19200,19200,19200,19200]}
};

let pass = 0, fail = 0;
function check(name, cond) {
  if (cond) { pass++; console.log('  ✅ ' + name); }
  else { fail++; console.log('  ❌ ' + name); }
}
try {
  update(base);
  check('正常状态渲染', els['status'].innerHTML.includes('缸'));
  check('状态栏非空', els['status'].innerHTML.length > 100);

  // 告警分支：限位+锁存+急停+状态中断
  update(Object.assign({}, base, {
    hw:Object.assign({}, base.hw, {limit:3, latch:1, estop:1}), hw_age:2.0
  }));
  const alarm = els['alarm'].textContent;
  check('限位告警', alarm.includes('限位触发'));
  check('锁存告警', alarm.includes('限位锁存'));
  check('急停告警', alarm.includes('软件急停'));
  check('断传告警', alarm.includes('状态回传中断'));

  // 手动模式 + 固件实际位置（虚线数据）
  update(Object.assign({}, base, {manual_disp:[0,0,30,0,0,0],
    hw:Object.assign({}, base.hw, {cur:[19500,19200,19200,19200,19200,19200]})}));
  check('手动/实际位置渲染', els['status'].innerHTML.includes('实际mm'));
} catch (e) {
  console.error('update() 抛异常:', e.message);
  fail++;
}
console.log('  UI runtime: %d PASS, %d FAIL', pass, fail);
process.exit(fail === 0 ? 0 : 1);
