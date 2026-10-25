import {rectDistance,roomBounds} from './planner.mjs';
/** Small dependency-free, Z-up point-cloud renderer. All coordinates are meters. */
export function createScene(canvas, options = {}) {
  if (!canvas || typeof canvas.getContext !== 'function') throw new TypeError('A canvas is required');
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('Canvas 2D rendering is unavailable');
  let data = { points: [], pose: null, trail: [], target: null };
  let mode = options.mode === 'top' ? 'top' : 'perspective';
  let follow = options.follow !== false, autoFit = options.autoFit === true, center = [0, 0, 1], distance = mode === 'top' ? 32 : 34;
  let azimuth = -.64, elevation = .77, drag = null, destroyed = false, dirty = true, fitted = false, raf = null;
  let width = 1, height = 1, ratio = 1, lastPaint = -Infinity;
  const handlers = [];
  const listen = (type, handler, settings) => { canvas.addEventListener(type, handler, settings); handlers.push([type, handler, settings]); };
  function resize() {
    const rect = canvas.getBoundingClientRect();
    const nextW = Math.max(1, rect.width), nextH = Math.max(1, rect.height);
    const nextR = Math.max(1, Math.min(2, globalThis.devicePixelRatio || 1));
    if (nextW === width && nextH === height && ratio === nextR) return;
    width = nextW; height = nextH; ratio = nextR;
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio); dirty = true;
  }
  function project(p) {
    const x = p[0] - center[0], y = p[1] - center[1], z = p[2] - center[2];
    if (mode === 'top') { const scale = Math.min(width, height) / distance; return [width/2 + x*scale, height/2 - y*scale, scale]; }
    const ca = Math.cos(azimuth), sa = Math.sin(azimuth), ce = Math.cos(elevation), se = Math.sin(elevation);
    const sx = -sa*x + ca*y, sy = -se*(ca*x+sa*y)+ce*z, depth = distance-ce*(ca*x+sa*y)-se*z;
    if (depth < .1) return null;
    const scale = Math.min(width,height)*1.28/depth;
    return [width/2+sx*scale,height/2-sy*scale,scale];
  }
  function pick(event,z=0){
    resize();const rect=canvas.getBoundingClientRect(),px=event.clientX-(rect.left??0),py=event.clientY-(rect.top??0),small=Math.min(width,height);
    if(mode==='top'){const scale=small/distance;return [center[0]+(px-width/2)/scale,center[1]-(py-height/2)/scale,z];}
    const ca=Math.cos(azimuth),sa=Math.sin(azimuth),ce=Math.cos(elevation),se=Math.sin(elevation),nx=(px-width/2)/(small*1.28),ny=-(py-height/2)/(small*1.28);
    const camera=[center[0]+distance*ce*ca,center[1]+distance*ce*sa,center[2]+distance*se],dir=[-sa*nx-se*ca*ny-ce*ca,ca*nx-se*sa*ny-ce*sa,ce*ny-se];
    const t=(z-camera[2])/dir[2];return t>0?[camera[0]+t*dir[0],camera[1]+t*dir[1],z]:null;
  }
  function pickObject(event){
    const rect=canvas.getBoundingClientRect(),x=event.clientX-(rect.left??0),y=event.clientY-(rect.top??0),hits=[];
    for(const b of data.placed??[]){const vertices=[];for(const z of [0,2.2])for(const px of [b[0],b[2]])for(const py of [b[1],b[3]])vertices.push(project([px,py,z]));const q=vertices.filter(Boolean);if(q.length&&x>=Math.min(...q.map(p=>p[0]))&&x<=Math.max(...q.map(p=>p[0]))&&y>=Math.min(...q.map(p=>p[1]))&&y<=Math.max(...q.map(p=>p[1]))){const p=[(b[0]+b[2])/2,(b[1]+b[3])/2,0],screen=project([p[0],p[1],1.1]);hits.push({p,d:Math.hypot(screen[0]-x,screen[1]-y)});}}
    for(const o of data.movers??[]){const q=project(o.position);if(q&&Math.hypot(q[0]-x,q[1]-y)<=Math.max(7,q[2]*o.radius+4))hits.push({p:[...o.position],d:Math.hypot(q[0]-x,q[1]-y)});}
    return hits.sort((a,b)=>a.d-b.d)[0]?.p??null;
  }
  function polygon(vertices,fill,stroke=null){const q=vertices.map(project);if(q.some(p=>!p))return;ctx.beginPath();q.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=fill;ctx.fill();if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=1;ctx.stroke();}}
  function roomFloor(){
    const [x0,y0,x1,y1]=data.bounds??roomBounds(data.obstacles);
    polygon([[x0,y0,0],[x1,y0,0],[x1,y1,0],[x0,y1,0]],'#172c32','#4a6b70');
    // A cutaway room keeps the drone and all user objects visible.
    for(const [a,b]of [[[x0,y1],[x1,y1]],[[x1,y1],[x1,y0]]])polygon([[...a,0],[...b,0],[...b,3.3],[...a,3.3]],'#75a7a511','#6e9d9c44');
    if(data.heat){
      for(let x=x0+.3;x<x1-.3;x+=.65)for(let y=y0+.3;y<y1-.3;y+=.65){
        let d=Math.min(...(data.obstacles??[]).map(r=>rectDistance([x,y],r)),...(data.movers??[]).map(o=>Math.max(0,Math.hypot(x-o.position[0],y-o.position[1])-o.radius)));
        const heat=Math.exp(-d*d/(2*(data.staticMargin+.6)**2));if(heat<.06)continue;
        polygon([[x,y,.015],[x+.65,y,.015],[x+.65,y+.65,.015],[x,y+.65,.015]],`rgba(255,${Math.round(165-110*heat)},65,${heat*.36})`);
      }
    }
  }
  function fixtureBox(rect,height,color,bottom=0,wall=false){
    const [x0,y0,x1,y1]=rect,corners=[[x0,y0,bottom],[x1,y0,bottom],[x1,y1,bottom],[x0,y1,bottom]];
    for(let i=0;i<4;i++){const a=corners[i],b=corners[(i+1)%4];polygon([a,b,[b[0],b[1],height],[a[0],a[1],height]],color+(wall?'66':i%2?'aa':'dd'),color+'77');}
    polygon(corners.map(p=>[p[0],p[1],height]),color+(wall?'99':'ee'),color);
  }
  function drawFixtures(){
    for(const f of data.fixtures??[]){
      const [x0,y0,x1,y1]=f.rect;
      if(f.kind==='sofa'){
        fixtureBox(f.rect,.45,f.color);fixtureBox([x0,y1-.28,x1,y1],f.height,f.color,.45);
        fixtureBox([x0,y0,x0+.23,y1],f.height*.8,f.color,.45);fixtureBox([x1-.23,y0,x1,y1],f.height*.8,f.color,.45);
      }else if(f.kind==='table'){
        fixtureBox(f.rect,f.height,f.color,f.height-.14);
        for(const x of [x0+.15,x1-.27])for(const y of [y0+.15,y1-.27])fixtureBox([x,y,x+.12,y+.12],f.height-.14,'#647b7e');
      }else{
        fixtureBox(f.rect,f.height,f.color,0,f.kind==='wall');
        if(f.kind==='shelf')for(let z=.55;z<f.height;z+=.55)line([x0,y0-.01,z],[x1,y0-.01,z],'#e4d3b788',1);
      }
      if(f.kind!=='wall'){const p=project([(x0+x1)/2,(y0+y1)/2,f.height+.15]);if(p){ctx.font='9px Segoe UI';ctx.fillStyle='#c4d5ce';ctx.fillText(f.label,p[0]+5,p[1]-4);}}
    }
  }
  function line(a,b,color,lineWidth=1,dash=[]) {
    const p=project(a),q=project(b);if(!p||!q)return;
    ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
  }
  function fit() {
    const points=data.points;
    if (!points.length && !data.pose) return;
    const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];
    for (const p of points.length ? points : [data.pose.position]) for(let i=0;i<3;i++){lo[i]=Math.min(lo[i],p[i]);hi[i]=Math.max(hi[i],p[i]);}
    center=lo.map((v,i)=>(v+hi[i])/2);if(data.room)center[2]=.6;
    distance=Math.max(mode==='top'?12:16,Math.hypot(hi[0]-lo[0],hi[1]-lo[1])*(mode==='top'?1.13:1.4));
    fitted=true;dirty=true;
  }
  function drawDrone() {
    const pose=data.pose;if(!pose)return;
    const p=project(pose.position);if(!p)return;
    const [qx,qy,qz,qw]=pose.orientation??[0,0,0,1];
    const yaw=Number.isFinite(pose.yaw)?pose.yaw:Math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
    const heading=project([pose.position[0]+Math.cos(yaw),pose.position[1]+Math.sin(yaw),pose.position[2]]);
    const angle=heading?Math.atan2(heading[1]-p[1],heading[0]-p[0]):0;
    ctx.save();ctx.translate(p[0],p[1]);
    ctx.strokeStyle='#fb897b33';ctx.lineWidth=1;ctx.beginPath();ctx.arc(0,0,data.room?p[2]*(data.bodyRadius??.62):18,0,Math.PI*2);ctx.stroke();
    ctx.fillStyle='#fb897b18';ctx.beginPath();ctx.arc(0,0,data.room?p[2]*(data.bodyRadius??.62)*.8:13,0,Math.PI*2);ctx.fill();
    ctx.rotate(angle);if(data.room)ctx.scale(p[2]*.025,p[2]*.025);ctx.fillStyle='#ff9586';ctx.strokeStyle='#ffd1b8';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(9,0);ctx.lineTo(-6,-6);ctx.lineTo(-3,0);ctx.lineTo(-6,6);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
    if(data.room){
      const pos=pose.position;for(const a of [-Math.PI/4,Math.PI/4,3*Math.PI/4,5*Math.PI/4]){const tip=[pos[0]+Math.cos(a+yaw)*.45,pos[1]+Math.sin(a+yaw)*.45,pos[2]],q=project(tip);line(pos,tip,'#e1f8fa',3);if(q){ctx.strokeStyle='#9aeeee';ctx.lineWidth=2;ctx.beginPath();ctx.arc(q[0],q[1],Math.max(1,q[2]*.16),0,Math.PI*2);ctx.stroke();}}
    }
    if(mode!=='top') {line([pose.position[0],pose.position[1],0],pose.position,'#efb3a544',1,[3,5]);ctx.fillStyle='#d6e3ce';ctx.font='9px Segoe UI';ctx.fillText('NX01',p[0]+14,p[1]-13);}
  }
  function paint() {
    resize();
    if (!dirty || width < 2 || height < 2) return;
    lastPaint=Date.now();dirty=false;ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);
    if(data.room)roomFloor();
    const extent=Math.min(1000,Math.max(18,Math.ceil(distance/2))), spacing=distance>90?10:2;
    const startX=Math.floor((center[0]-extent)/spacing)*spacing,startY=Math.floor((center[1]-extent)/spacing)*spacing;
    for(let i=0;i<=Math.min(150,extent*2/spacing);i++){
      const x=startX+i*spacing,y=startY+i*spacing;
      line([x,center[1]-extent,0],[x,center[1]+extent,0],'#87b4a30d');
      line([center[0]-extent,y,0],[center[0]+extent,y,0],'#87b4a30d');
    }
    if(data.room)drawFixtures();
    // Only cells traversed by sensor rays are supplied for hidden environments.
    // Dark canvas remains unknown; there is no ground-truth floor overlay.
    for(const p of data.freeCells??[]){const h=(data.cellSize??.5)/2;
      polygon([[p[0]-h,p[1]-h,0],[p[0]+h,p[1]-h,0],[p[0]+h,p[1]+h,0],[p[0]-h,p[1]+h,0]],'#73cbb51a');}
    // Render bounded source geometry. Dots represent occupied measurements only.
    for(const p of data.room?[]:data.points){
      const q=project(p);if(!q||q[0]<-5||q[0]>width+5||q[1]<-5||q[1]>height+5)continue;
      const size=mode==='top'?1.6:Math.max(1.1,Math.min(3,q[2]*.06));
      const z=Math.max(0,Math.min(3,p[2]));ctx.fillStyle=z>1.8?'#9fc9b780':z>.7?'#7fb4a59c':'#628e8990';ctx.fillRect(q[0],q[1],size,size);
    }
    for(const p of data.observations??[]){const q=project(p);if(q){ctx.fillStyle='#8fe0d1';ctx.fillRect(q[0]-1.5,q[1]-1.5,3,3);}}
    for(const box of data.placed??[]){
      const corners=[[box[0],box[1],0],[box[2],box[1],0],[box[2],box[3],0],[box[0],box[3],0]];
      const q=corners.map(p=>project(p));if(q.every(Boolean)){ctx.beginPath();q.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle='#da795b88';ctx.fill();ctx.strokeStyle=data.selectedBox?.every((v,i)=>v===box[i])?'#8ef7ff':'#ffc29c';ctx.lineWidth=data.selectedBox?.every((v,i)=>v===box[i])?3:2;ctx.stroke();}
      if(data.room&&mode!=='top'){
        for(let i=0;i<4;i++){const a=corners[i],b=corners[(i+1)%4];polygon([a,b,[b[0],b[1],2.2],[a[0],a[1],2.2]],i%2?'#bd8663dd':'#946d55dd','#ffc79e55');}
        polygon(corners.map(p=>[p[0],p[1],2.2]),'#e6ae79','#ffdab4');
      }
      if(mode!=='top')for(let i=0;i<4;i++){const a=corners[i],b=corners[(i+1)%4],at=[a[0],a[1],2.2],bt=[b[0],b[1],2.2];line(a,at,'#ffb789',2);line(at,bt,'#ffb789',2);}
      if(mode==='top'){const pad=data.staticMargin??.69;const r=[[box[0]-pad,box[1]-pad,0],[box[2]+pad,box[1]-pad,0],[box[2]+pad,box[3]+pad,0],[box[0]-pad,box[3]+pad,0]];for(let i=0;i<4;i++)line(r[i],r[(i+1)%4],'#ffbc8255',1,[3,4]);}
    }
    for(let i=1;i<data.trail.length;i++)line(data.trail[i-1],data.trail[i],'#c9f76baa',mode==='top'?1.5:1.7);
    if(data.target&&data.pose){line(data.pose.position,data.target,'#f4c68e99',1,[4,5]);const q=project(data.target);if(q){ctx.strokeStyle='#f4c68e';ctx.beginPath();ctx.arc(q[0],q[1],5,0,Math.PI*2);ctx.stroke();}}
    // Dashed cyan future path, distinct from the lime recorded trail.
    for(let i=1;i<(data.planned?.length??0);i++)line(data.planned[i-1],data.planned[i],'#78dbea',2,[6,5]);
    if(data.goal){const q=project(data.goal);if(q){ctx.strokeStyle='#78dbea';ctx.lineWidth=2;ctx.beginPath();ctx.arc(q[0],q[1],7,0,Math.PI*2);ctx.stroke();ctx.font='9px Segoe UI';ctx.fillStyle='#a2e9ef';ctx.fillText('GOAL',q[0]+10,q[1]-8);}}
    for(const actor of data.movers??[]){
      const p=project(actor.position),edge=project([actor.position[0]+actor.radius,actor.position[1],actor.position[2]]);if(!p||!edge)continue;
      const radius=Math.max(5,Math.abs(edge[0]-p[0]),p[2]*actor.radius);
      ctx.fillStyle='#ffbd6b44';ctx.strokeStyle=data.selectedId===actor.id?'#8ef7ff':'#ffbd6b';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(p[0],p[1],radius,0,Math.PI*2);ctx.fill();ctx.stroke();
      ctx.setLineDash([3,4]);ctx.strokeStyle='#ffbd6b55';ctx.beginPath();ctx.arc(p[0],p[1],radius+p[2]*(data.margin??.79),0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
      ctx.font='9px Segoe UI';ctx.fillStyle='#ffcf90';ctx.fillText(actor.id+(actor.axis==='x'?' ↔ X':actor.axis==='y'?' ↕ Y':''),p[0]+radius+4,p[1]-5);
    }
    if(data.heat)for(const prediction of data.predictions??[])for(let i=1;i<prediction.length;i++)line(prediction[i-1],prediction[i],'#ffbd6b88',1,[3,5]);
    drawDrone();
    // A restrained world orientation indicator in the corner.
    if(mode!=='top'){ctx.font='8px Segoe UI';ctx.fillStyle='#849b90';ctx.fillText('Z',width-26,height-53);ctx.strokeStyle='#8da894';ctx.beginPath();ctx.moveTo(width-23,height-27);ctx.lineTo(width-23,height-45);ctx.stroke();ctx.strokeStyle='#6d8e81';ctx.beginPath();ctx.moveTo(width-23,height-27);ctx.lineTo(width-38,height-20);ctx.moveTo(width-23,height-27);ctx.lineTo(width-9,height-20);ctx.stroke();}
  }
  function loop(){if(destroyed)return;paint();raf=requestAnimationFrame(loop);}
  function endDrag(){drag=null;}
  if(options.interactive!==false){
    listen('pointerdown',event=>{if(event.isPrimary===false||event.button!==undefined&&event.button!==0)return;autoFit=false;drag={x:event.clientX,y:event.clientY,startX:event.clientX,startY:event.clientY,pan:event.shiftKey||mode==='top'};canvas.setPointerCapture?.(event.pointerId);});
    listen('pointermove',event=>{if(!drag)return;const dx=event.clientX-drag.x,dy=event.clientY-drag.y;if(drag.pan){const scale=distance/Math.max(1,Math.min(width,height));center[0]-=dx*scale;center[1]+=dy*scale;}else{azimuth-=dx*.008;elevation=Math.max(.12,Math.min(1.5,elevation+dy*.008));}drag.x=event.clientX;drag.y=event.clientY;follow=false;options.onFollowChange?.(false);dirty=true;});
    listen('dblclick',event=>{if(mode!=='top'||!options.onGoal)return;const rect=canvas.getBoundingClientRect(),scale=Math.min(width,height)/distance;options.onGoal([center[0]+(event.clientX-(rect.left??0)-width/2)/scale,center[1]-(event.clientY-(rect.top??0)-height/2)/scale,data.pose?.position[2]??1.8]);});
    listen('pointerup',event=>{
      if(drag&&options.onMapClick&&Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)<5){const p=pick(event,0);if(p)options.onMapClick([p[0],p[1],data.pose?.position[2]??1.8],pickObject(event));}
      endDrag();
    });listen('pointercancel',endDrag);listen('lostpointercapture',endDrag);
    listen('wheel',event=>{autoFit=false;event.preventDefault();distance=Math.max(3,Math.min(1200,distance*Math.exp(Math.max(-200,Math.min(200,event.deltaY))*.002)));dirty=true;},{passive:false});
    listen('keydown',event=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-','0'].includes(event.key))return;event.preventDefault();autoFit=false;if(event.key==='0'){follow=false;options.onFollowChange?.(false);fit();return;}if(event.key==='+'||event.key==='=')distance=Math.max(3,distance*.85);else if(event.key==='-')distance=Math.min(1200,distance*1.15);else if(mode==='top'){const d=distance*.04;if(event.key==='ArrowLeft')center[0]-=d;if(event.key==='ArrowRight')center[0]+=d;if(event.key==='ArrowUp')center[1]+=d;if(event.key==='ArrowDown')center[1]-=d;follow=false;options.onFollowChange?.(false);}else{if(event.key==='ArrowLeft')azimuth-=.1;if(event.key==='ArrowRight')azimuth+=.1;if(event.key==='ArrowUp')elevation=Math.min(1.5,elevation+.1);if(event.key==='ArrowDown')elevation=Math.max(.12,elevation-.1);}dirty=true;});
  }
  const observer=typeof ResizeObserver==='function'?new ResizeObserver(()=>{dirty=true;}):null;observer?.observe(canvas);
  raf=requestAnimationFrame(loop);
  return Object.freeze({
    setData(next){if(destroyed||!next||!Array.isArray(next.points)||!Array.isArray(next.trail))return false;const changed=data.points!==next.points;if(data.points.length&&!next.points.length){fitted=false;autoFit=options.autoFit===true;}data=next;if((!fitted||autoFit&&changed)&&data.points.length)fit();if(follow&&data.pose){center=[...data.pose.position];if(mode==='top'&&options.local)distance=19;}dirty=true;if(Date.now()-lastPaint>500)paint();return true;},
    project(point){resize();return project(point);},
    fit(){follow=false;options.onFollowChange?.(false);fit();},
    setMode(value){if(!['top','perspective'].includes(value))return false;mode=value;fit();dirty=true;return true;},
    setFollow(value){if(typeof value!=='boolean')return false;follow=value;if(value&&data.pose)center=[...data.pose.position];dirty=true;return true;},
    destroy(){destroyed=true;if(raf!==null)cancelAnimationFrame(raf);observer?.disconnect();for(const [type,handler,settings]of handlers)canvas.removeEventListener(type,handler,settings);}
  });
}
