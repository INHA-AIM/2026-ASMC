window.createLocalExperimentRenderer = canvas => {
  const ctx=canvas.getContext("2d");
  const xy=(x,y)=>[320-30*y,680-30*x];
  let raster=null, rasterKey='';
  const dot=(x,y,color,r=4)=>{const p=xy(x,y);ctx.fillStyle=color;ctx.beginPath();ctx.arc(...p,r,0,Math.PI*2);ctx.fill();};
  function line(points,color,width=1,dash=[]){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();points.forEach((p,i)=>{const q=xy(p[0],p[1]);i?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.stroke();ctx.setLineDash([]);ctx.lineWidth=1;}
  function body(p,v,color){const [x,y,yaw]=p,c=Math.cos(yaw),s=Math.sin(yaw),points=[];
    for(const [a,b] of [[-v.rear,-v.width/2],[v.front,-v.width/2],[v.front,v.width/2],[-v.rear,v.width/2]])points.push([x+c*a-s*b,y+s*a+c*b]);
    line([...points,points[0]],color);}
  function makeRaster(d){if(!d?.raster_available){raster=null;return;}if(raster && rasterKey===d.stamp_ns)return;
    const p=atob(d.probability), c=atob(d.coverage), off=document.createElement('canvas');off.width=d.width;off.height=d.height;
    const oc=off.getContext('2d'),im=oc.createImageData(d.width,d.height);
    for(let i=0;i<p.length;i++){const cv=c.charCodeAt(i),v=p.charCodeAt(i),j=i*4;
      const color=cv===255?(v>=153?[25,130,72,150]:[143,43,57,150]):cv===128?[121,106,73,100]:[32,38,47,160];
      im.data.set(color,j);}
    oc.putImageData(im,0,0);raster=off;rasterKey=d.stamp_ns;
  }
  function draw(row,fresh,detail,options={}){
    makeRaster(detail);ctx.save();
    const rect=canvas.getBoundingClientRect(),density=window.devicePixelRatio||1;
    ctx.setTransform(density,0,0,density,0,0);ctx.clearRect(0,0,rect.width,rect.height);
    const toolbar=canvas.parentElement?.querySelector?.('.toolbar');
    const top=toolbar?toolbar.offsetTop+toolbar.offsetHeight+8:8;
    const scale=Math.min(rect.width/640,Math.max(100,rect.height-top-16)/720);
    // Fit x=-2..20m below the wrapping toolbar, including rear axle/body.
    ctx.translate((rect.width-640*scale)/2,top-40*scale);ctx.scale(scale,scale);
    ctx.globalAlpha=fresh?1:.3;
    if(row&&raster&&detail?.raster_available&&(options.road!==false)){const R=row.scene?.rotation||[1,0,0,0,1,0,0,0,1],z=row.scene?.ground_z??-.36;
      // Display the producer's plane grid in exactly the planner's level frame.
      // This is an affine display transform, not another camera projection.
      const t=row.scene?.source_in_plan||[0,0,0],c=Math.cos(t[2]),s=Math.sin(t[2]);
      const a=R[0]-R[2]*R[6]/R[8],b=R[1]-R[2]*R[7]/R[8],d=R[3]-R[5]*R[6]/R[8],e=R[4]-R[5]*R[7]/R[8];
      const A=c*a-s*d,B=c*b-s*e,D=s*a+c*d,E=s*b+c*e,r=detail.resolution;
      const x=t[0]+A*detail.xmax+B*detail.ymax+(c*R[2]-s*R[5])*z/R[8], y=t[1]+D*detail.xmax+E*detail.ymax+(s*R[2]+c*R[5])*z/R[8];
      const origin=xy(x,y);ctx.save();ctx.transform(30*E*r,30*B*r,30*D*r,30*A*r,origin[0],origin[1]);ctx.imageSmoothingEnabled=false;ctx.drawImage(raster,0,0);ctx.fillStyle='rgba(80,170,200,.55)';(row.scene?.recent_road_runs||[]).forEach(([r,a,b])=>ctx.fillRect(a,r,b-a+1,1));ctx.restore();}
    ctx.strokeStyle='#314254';ctx.fillStyle='#94abbd';ctx.font='12px sans-serif';
    for(let x=-2;x<=20;x+=2){const y=xy(x,0)[1];ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(640,y);ctx.stroke();ctx.fillText(x+'m',8,y-4);}
    for(let y=-10;y<=10;y+=2){const x=xy(0,y)[0];ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,760);ctx.stroke();}
    if(!row){ctx.restore();return;}const v=row.vehicle||{width:2.160835,front:3.845,rear:.79};
    if(options.family!==false){(row.scene?.boundaries||[]).forEach(b=>line(b.points,b.role===2?'#ffe46d':'#e1e6f0',2));line(row.scene?.home||[],'#7ab9ff',2,[8,5]);}
    if(options.objects!==false)(row.scene?.obstacles||[]).forEach(o=>{const points=o.polygon||[];if(!points.length)return;if(o.motion_sweep?.length)line([...o.motion_sweep,o.motion_sweep[0]],'#6ac7ff',1,[6,4]);line([...points,points[0]],o.retained_measurement?'#cc9aff':o.id===row.blocking_id?'#ff6b5f':'#f1a66f',2,o.retained_measurement?[5,4]:[]);const p=xy(...points[0]);ctx.fillStyle=o.retained_measurement?'#cc9aff':'#ffc594';if(options.labels!==false)ctx.fillText('#'+o.id+' '+(o.camera_class||o.classification||'unknown')+(o.retained_measurement?' · 과거 측정 '+Math.round(o.source_age_s*1000)+'ms':''),p[0],p[1]-4);});
    if(options.candidates!==false)(row.candidates||[]).forEach(c=>line(c.path,c.reason==='sweep_passed'?'#99aabb':'#9e5e71',1,[3,4]));
    const path=row.path||[];if(options.trajectory!==false)line(path,'#48ffe0',3);
    if(options.sweep!==false)path.filter((p,i)=>i%5===0).forEach(p=>body(p,v,'#278982'));
    body([0,0,0],v,'white');dot(0,0,'white');(row.goals||[]).forEach(p=>dot(p[0],p[1],'#c899ff'));
    if(options.rejected!==false)(row.rejected_examples||[]).forEach(p=>dot(p.pose[0],p.pose[1],'#fa7c81',2));
    if(row.blocked&&Object.keys(row.rejected||{}).length)dot(...row.blocked,'#ffb24b',5);ctx.globalAlpha=1;ctx.restore();
  }
  return {draw};
};
