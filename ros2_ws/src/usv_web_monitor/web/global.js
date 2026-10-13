(() => {
  'use strict';
  const $=id=>document.getElementById(id),svg=$('field'),ns='http://www.w3.org/2000/svg';
  let payload=null,received=0,online=false,zoom=1,busy=false;
  const fmt=v=>Number.isFinite(v)?v.toFixed(2):'—';
  const point=p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite);
  const age=v=>Number.isFinite(v)?v+(performance.now()-received)/1000:Infinity;
  function el(tag,attrs,text){const n=document.createElementNS(ns,tag);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,String(v)));if(text!==undefined)n.textContent=text;svg.appendChild(n);return n;}
  function render(){
    const g=payload?.global||{},n=g.navigation||{},geometry=g.geometry||{},size=geometry.size_m;
    const navFresh=online&&age(g.navigation_age_sec)<1,visualFresh=online&&age(g.observation_age_sec)<1;
    const m=g.mission||{},missionFresh=online&&age(g.mission_age_sec)<1;
    $('connection').textContent=online?(payload?.demo?'演示数据 · 不可执行':'网页已连接'):'网页未连接';
    $('mapBadge').textContent=visualFresh?(g.objects?.vessel?'视觉实时':'已收到数据 · 船未定位'):navFresh&&Array.isArray(n.predicted_pose)?'导航预测 / 检查定位':point(size)?'场地缓存 · 船位不可用':'场地 / 船位不可用';
    const states={navigating:'正在驶向球门',arrived:'已深入球门 · 动力停止',goal_entry_verifying:'预测已到停动力线 · 等待定位确认',manual_or_mode_unavailable:'手动模式 / 等待自动',waiting_for_map_and_position:'等待场地、球门及船位',imu_timeout:'IMU超时 · 停止',visual_timeout:'位置超时 · 停止',pwm_feedback_timeout:'动力反馈超时 · 停止',actuator_stopped:'急停或执行器保护',goal_boundary_stop:'非球门通道越界 · 停止',goal_overshoot_stop:'超出进门余量 · 停止',goal_geometry_invalid:'球门几何数据无效',stream_recovered_wait_manual:'数据已恢复 · 先切手动再切自动'};
    $('navState').textContent=navFresh?(states[n.state]||n.state||'—'):'导航节点未连接';
    if(navFresh&&n.navigation_degraded)$('navState').textContent+=' · 降级推算';
    $('goal').textContent=navFresh?({left:'左球门',right:'右球门'}[n.goal_side]||'—'):'—';
    $('mode').textContent=navFresh?(n.automatic_selected?'自动':'手动'):'—';
    $('missionPanel').hidden=!missionFresh;
    $('pageTitle').textContent=missionFresh?'拿球，驶向球门。':'直达球门。';
    $('pageSubtitle').textContent=missionFresh?'全局接近 · 视觉对准 · 联合拿球确认 · 球门送球':'遥控切自动 · 穿过所选门线 · 深入0.5米后停止动力';
    if(missionFresh){
      const stages={manual:'手动控制',wait_global:'等待无人机船位与球位',approach_ball:'全局导航接近小球',visual_acquire:'船载视觉对准 / 拿球',deliver_goal:'拿球已确认 · 驶向球门',complete:'进门完成 · 动力停止',wait_manual_rearm:'数据源变更 · 请切手动再切自动'};
      const reasons={manual_or_mode_unavailable:'等待CH8自动',stream_changed_wait_manual:'等待手动重新确认',imu_timeout:'IMU超时',pwm_feedback_timeout:'动力反馈超时',actuator_stopped:'急停或执行器保护',at_ball_wait_camera:'到达球附近 · 等待可靠视觉',camera_unreliable:'相机检测暂时不可靠',visual_command_timeout:'视觉控制输出超时',goal_command_timeout:'球门控制输出超时',global_position_unavailable:'全局定位暂不可用',ball_position_unavailable:'无人机球位缺失或过期'};
      $('missionState').textContent=(stages[m.stage]||m.stage)+(m.blocked_reason?' · '+(reasons[m.blocked_reason]||m.blocked_reason):'');
      $('navState').textContent=$('missionState').textContent;
      $('mode').textContent=m.automatic?'自动':'手动';
      $('missionConfidence').textContent=fmt(m.camera_confidence)+' / '+fmt(m.camera_confidence_threshold);
      $('missionDistance').textContent=fmt(m.ball_distance_m)+' m';
      $('missionHeld').textContent=m.drone_held_fresh?'本船 held · 新鲜':'未确认 / 过期';
      $('missionArea').textContent=fmt(Number.isFinite(m.camera_area_ratio)?m.camera_area_ratio*100:null)+' / '+fmt(Number.isFinite(m.capture_area_ratio_threshold)?m.capture_area_ratio_threshold*100:null)+' %';
      $('missionEvidence').textContent=fmt(m.capture_evidence_duration_sec)+' s';
      $('missionCaptured').textContent=m.has_ball?'已确认':'等待联合确认';
    }
    const own=g.objects?.vessel,pose=navFresh?n.predicted_pose:null;
    $('position').textContent=pose?fmt(pose[0])+' / '+fmt(pose[1])+' m（融合）':own?fmt(own.xy_m[0])+' / '+fmt(own.xy_m[1])+' m（视觉'+(visualFresh?'':'已过期')+'）':'—';
    $('heading').textContent=pose?fmt(pose[2]*180/Math.PI)+'°':'—';
    $('depth').textContent=navFresh?fmt(n.entry_depth_m)+' / '+fmt(n.goal_entry_depth_m)+' m':'—';
    $('speed').textContent=navFresh?fmt(n.desired_speed_m_s)+' / '+fmt(n.estimated_speed_m_s)+' m/s':'—';
    $('yaw').textContent=navFresh?fmt(n.yaw_rate_rad_s*180/Math.PI)+'°/s':'—';
    const rx=g.receiver||{},counts=rx.counts||{};
    $('freshness').textContent='视觉年龄 '+fmt(age(g.observation_age_sec))+' s · UDP收到 '+(counts.received??'—')+' / 接受 '+(counts.accepted??'—')+' / 拒绝 '+(counts.rejected??'—')+' · 本船定位'+(rx.own_boat_valid?'有效':'无效')+' · 场地 '+(rx.field_status||'未收到 / 过期');
    for(const side of ['left','right']){const b=$(side+'Goal');b.disabled=busy||!navFresh||(missionFresh?!!m.automatic:!!n.automatic_selected)||!!payload?.demo;b.classList.toggle('primary',n.goal_side===side);}
    svg.replaceChildren();$('empty').hidden=point(size);if(!point(size))return;
    const [w,h]=size,extra=2,screen=p=>[p[0],h-p[1]];
    svg.setAttribute('viewBox',[w/2-(w+extra*2)/zoom/2,h/2-(h+extra*2)/zoom/2,(w+extra*2)/zoom,(h+extra*2)/zoom].join(' '));
    $('fieldInfo').textContent=fmt(w)+' × '+fmt(h)+' m · '+geometry.coord;
    const line=(points,color,dash=false)=>{if(!points.every(point))return;el('polyline',{points:points.map(p=>screen(p).join(',')).join(' '),fill:'none',stroke:color,'stroke-width':.06,'stroke-dasharray':dash?'.18 .15':''});};
    el('rect',{x:0,y:0,width:w,height:h,fill:'#f8fcfd',stroke:'#9dbfc6','stroke-width':.05});
    // Bound rendering work even if an upstream size is unexpectedly large.
    const step=Math.max(1,Math.ceil(Math.max(w,h)/100));
    for(let x=step;x<w;x+=step)line([[x,0],[x,h]],'#e7eef3');
    for(let y=step;y<h;y+=step)line([[0,y],[w,y]],'#e7eef3');
    Object.entries(geometry.goals||{}).forEach(([side,ends])=>{if(!Array.isArray(ends)||!ends.every(point))return;line(ends,side===n.goal_side?'#078b92':'#91b4b4');const p=screen(ends[0]);el('text',{x:p[0],y:p[1]+.4,'font-size':.28,'text-anchor':'middle',fill:'#087f8c'},side==='left'?'左球门':'右球门');});
    const destination=missionFresh?m.target_xy_m:n.target_xy_m;
    if(navFresh&&point(destination)){
      if(pose)line([pose.slice(0,2),destination],'#078b92',true);
      const p=n.target_xy_m,gate=n.gate;
      if(point(p)&&(!missionFresh||['deliver_goal','complete'].includes(m.stage)))line([[p[0],gate?.y_min??p[1]-.5],[p[0],gate?.y_max??p[1]+.5]],'#d55a58');
    }
    const ball=g.objects?.ball,ballPoint=visualFresh&&point(ball?.xy_m)?ball.xy_m:missionFresh&&m.ball_age_sec<2&&point(m.ball_xy_m)?m.ball_xy_m:null;
    if(ballPoint){const [x,y]=screen(ballPoint);el('circle',{cx:x,cy:y,r:.13,fill:'#ee922b'});el('text',{x:x+.2,y:y-.18,'font-size':.24,fill:'#b66c19'},'球');}
    function boat(p,heading,color){if(!point(p)||!Number.isFinite(heading))return;const [x,y]=screen(p);el('path',{d:'M .3 0 L -.22 -.17 L -.13 0 L -.22 .17 Z',fill:color,transform:'translate('+x+' '+y+') rotate('+(-heading*180/Math.PI)+')'});}
    if(own)boat(own.xy_m,own.heading_rad,visualFresh?'#2879ee':'#a7afb7');
    if(pose)boat(pose.slice(0,2),pose[2],'#9765cf');
  }
  for(const side of ['left','right'])$(side+'Goal').onclick=async()=>{
    if(busy||payload?.demo)return;busy=true;render();
    try{const r=await fetch('/api/goal',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({goal_side:side})});const d=await r.json();if(!r.ok||!d.ok)throw Error(d.error||'切换失败');$('selectionStatus').textContent='节点已确认'+(side==='left'?'左':'右')+'球门；CH8切自动后执行。';}
    catch(e){$('selectionStatus').textContent=e.message;}finally{busy=false;render();}
  };
  $('zoomIn').onclick=()=>{zoom=Math.min(4,zoom*1.25);render();};$('zoomOut').onclick=()=>{zoom=Math.max(1,zoom/1.25);render();};$('fitMap').onclick=()=>{zoom=1;render();};
  function connect(){const ws=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws');ws.onopen=()=>{online=true;};ws.onmessage=e=>{try{const p=JSON.parse(e.data);if(p.type==='telemetry'){payload=p;received=performance.now();render();}}catch(err){console.warn(err);}};ws.onclose=()=>{online=false;render();setTimeout(connect,2000);};ws.onerror=()=>ws.close();}
  setInterval(render,500);connect();render();
})();
