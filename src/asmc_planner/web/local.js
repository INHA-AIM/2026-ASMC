(() => {
  const $=x=>document.getElementById(x),canvas=$('view');
  const renderer=window.createLocalExperimentRenderer(canvas);
  let lastRow=null,lastDetail=null;
  function draw(row,fresh,detail){renderer.draw(row,fresh,detail,Object.fromEntries(['road','family','objects','candidates','sweep','rejected'].map(k=>[k,$(k).checked])));}
  function cameraEvidence(d,r){const rows=[];for(const c of d?.cameras||[]){const delta=(Number(c.stamp_ns)-Number(d.stamp_ns))/1e9;rows.push(c.view+' · Δscene='+delta.toFixed(3)+'s · '+(c.valid?'유효':'무효'));
    rows.push(...c.boxes.map(b=>'  '+b.name+' / '+b.group+' '+b.confidence.toFixed(2)+' bbox='+b.bbox.map(x=>x.toFixed(0)).join(',')));}
    $('camera').textContent=rows.join('\n')||'YOLO 관측 수신 대기';
    $('evidence').textContent=JSON.stringify({raster:d?.reason||'대기',fusion:r?.scene?.obstacles||[],omitted:r?.scene?.obstacles_omitted||0},null,2);}
  async function update(){try{const res=await fetch('/api/snapshot',{cache:'no-store'});if(!res.ok)throw Error(res.status);const s=await res.json(),r=s.local_experiment,c=s.local_control;
    const age=Date.now()/1000-(s.received_at?.local_experiment||0),fresh=age>=-1&&age<.6;
    let detail=null;if(r){const response=await fetch('/api/local-scene?sequence='+r.plan_sequence,{cache:'no-store'});if(response.ok)detail=await response.json();}
    lastRow=r;lastDetail=detail;
    $('status').textContent=r?(fresh?'':'[오래된 표시] ')+(r.valid?(r.recovery?'저속 회복 경로':'경로 유효'):'계산 불가 · '+r.reason):'LOCAL-1 수신 대기';
    $('status').className=r&&r.valid&&fresh?'':'bad';
    $('info').textContent=r?JSON.stringify({run:r.run_id,frame:r.frame,plan:r.plan_sequence,inputs:r.inputs,reason:r.reason,
      source_age_s:r.age_s,target_kph:r.requested_kph,selected_kph:r.selected_kph,speed_reason:r.speed_reason,
      curve_kph:r.curve_kph,stopping_space_kph:r.space_kph,planning_ms:r.processing_ms,
      required_stopping_m:r.required_stopping_m,available_stopping_m:r.available_stopping_m,recovery_extension_m:r.recovery_extension_m,
      assisted:r.assisted,recovery_obb_overlap:r.recovery_obb_overlap,expanded:r.expanded,
      candidates_omitted:r.candidates_omitted,budget_exhausted:r.budget_exhausted,journal_dropped:r.journal_dropped},null,2):'local.launch는 기본 계산 전용입니다.';
    $('failures').textContent=JSON.stringify(r?.rejected||{},null,2);$('control').textContent=JSON.stringify(c||{},null,2);
    cameraEvidence(detail,r);draw(r,fresh,detail);
  }catch(e){$('status').textContent='연결 실패: '+e.message;draw(null,false,null);}finally{setTimeout(update,200);}}
  for(const c of document.querySelectorAll('input[type=checkbox]'))c.addEventListener('change',()=>draw(lastRow,true,lastDetail));
  update();
})();
