let ws=null,ack=null;
function connect(){
  const port=location.port?parseInt(location.port):80;
  const url='ws://'+location.hostname+':'+(port+1);
  ws=new WebSocket(url);
  ws.onopen=()=>{document.getElementById('conn').textContent='已连接';document.getElementById('conn').className='good'};
  ws.onclose=()=>{document.getElementById('conn').textContent='断开，重试中';document.getElementById('conn').className='bad';setTimeout(connect,1500)};
  ws.onmessage=e=>{
    const s=JSON.parse(e.data);
    if(s.ack){ack=s.ack;return;}
    update(s);
  };
}
function send(cmd){if(ws&&ws.readyState===1)ws.send(JSON.stringify(Object.assign({cmd:cmd},collect())))}
function collect(){
  return {scale:parseFloat(document.getElementById('scale').value),
    max_mm:parseFloat(document.getElementById('maxmm').value),
    base_steps:parseInt(document.getElementById('base').value),
    serial1:document.getElementById('p1').value,
    serial2:document.getElementById('p2').value};
}
function update(s){
  const btnS=document.getElementById('btnStart'),btnX=document.getElementById('btnStop');
  btnS.disabled=s.running||s.starting;btnX.disabled=!s.running&&!s.starting;
  const st=document.getElementById('status');
  st.textContent=s.running?'运行中':(s.starting?'启动中':(s.stopping?'回零中':'已停止'));
  st.className=s.running?'good':(s.stopping?'warn':'bad');
  document.getElementById('tele').textContent=s.tele_frames||0;
  document.getElementById('sent').textContent=s.frames||0;
  const age=s.age;
  document.getElementById('age').textContent=(age>=0?age.toFixed(2)+'s':'—');
  document.getElementById('error').textContent=s.error||'';
  document.getElementById('error').className=s.error?'bad':'good';
  for(let i=0;i<6;i++){
    document.getElementById('t'+i).textContent=(s.target||[])[i]||0;
    document.getElementById('a'+i).textContent=(s.applied||[])[i]||0;
  }
  const alarm=document.getElementById('alarm');let msgs=[];
  if(s.error)msgs.push('⛔ '+s.error);
  if(s.running&&age>1)msgs.push('⛔ 遥测断流 '+age.toFixed(1)+'s（保持当前位置）');
  if(msgs.length){alarm.style.display='block';alarm.textContent=msgs.join(' | ')}else alarm.style.display='none';
}
for(const id of ['scale','maxmm','base']){
  document.getElementById(id).addEventListener('input',()=>{
    const v=document.getElementById(id).value;
    document.getElementById(id+'v').textContent=(id==='base')?v:(id==='scale'?parseFloat(v).toFixed(2):v);
  });
}
connect();
