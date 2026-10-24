/** Small dependency-free, Z-up point-cloud renderer. All coordinates are meters. */
export function createScene(canvas, options = {}) {
  if (!canvas || typeof canvas.getContext !== 'function') throw new TypeError('A canvas is required');
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('Canvas 2D rendering is unavailable');
  let data = { points: [], pose: null, trail: [], target: null };
  let mode = options.mode === 'top' ? 'top' : 'perspective';
  let follow = options.follow !== false, autoFit = options.autoFit === true, center = [0, 0, 1], distance = mode === 'top' ? 32 : 34;
  let azimuth = -.64, elevation = .77, drag = null, destroyed = false, dirty = true, fitted = false, raf = null;
  let width = 1, height = 1, ratio = 1;
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
  function line(a,b,color,lineWidth=1,dash=[]) {
    const p=project(a),q=project(b);if(!p||!q)return;
    ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
  }
  function fit() {
    const points=data.points;
    if (!points.length && !data.pose) return;
    const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];
    for (const p of points.length ? points : [data.pose.position]) for(let i=0;i<3;i++){lo[i]=Math.min(lo[i],p[i]);hi[i]=Math.max(hi[i],p[i]);}
    center=lo.map((v,i)=>(v+hi[i])/2);
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
    ctx.strokeStyle='#fb897b33';ctx.lineWidth=1;ctx.beginPath();ctx.arc(0,0,18,0,Math.PI*2);ctx.stroke();
    ctx.fillStyle='#fb897b18';ctx.beginPath();ctx.arc(0,0,13,0,Math.PI*2);ctx.fill();
    ctx.rotate(angle);ctx.fillStyle='#ff9586';ctx.strokeStyle='#ffd1b8';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(9,0);ctx.lineTo(-6,-6);ctx.lineTo(-3,0);ctx.lineTo(-6,6);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
    if(mode!=='top') {line([pose.position[0],pose.position[1],0],pose.position,'#efb3a544',1,[3,5]);ctx.fillStyle='#d6e3ce';ctx.font='9px Segoe UI';ctx.fillText('NX01',p[0]+14,p[1]-13);}
  }
  function paint() {
    resize();
    if (!dirty || width < 2 || height < 2) return;
    dirty=false;ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);
    const extent=Math.min(1000,Math.max(18,Math.ceil(distance/2))), spacing=distance>90?10:2;
    const startX=Math.floor((center[0]-extent)/spacing)*spacing,startY=Math.floor((center[1]-extent)/spacing)*spacing;
    for(let i=0;i<=Math.min(150,extent*2/spacing);i++){
      const x=startX+i*spacing,y=startY+i*spacing;
      line([x,center[1]-extent,0],[x,center[1]+extent,0],'#87b4a30d');
      line([center[0]-extent,y,0],[center[0]+extent,y,0],'#87b4a30d');
    }
    // Render bounded source geometry. Dots represent occupied measurements only.
    for(const p of data.points){
      const q=project(p);if(!q||q[0]<-5||q[0]>width+5||q[1]<-5||q[1]>height+5)continue;
      const size=mode==='top'?1.6:Math.max(1.1,Math.min(3,q[2]*.06));
      const z=Math.max(0,Math.min(3,p[2]));ctx.fillStyle=z>1.8?'#9fc9b780':z>.7?'#7fb4a59c':'#628e8990';ctx.fillRect(q[0],q[1],size,size);
    }
    for(let i=1;i<data.trail.length;i++)line(data.trail[i-1],data.trail[i],'#c9f76baa',mode==='top'?1.5:1.7);
    if(data.target&&data.pose){line(data.pose.position,data.target,'#f4c68e99',1,[4,5]);const q=project(data.target);if(q){ctx.strokeStyle='#f4c68e';ctx.beginPath();ctx.arc(q[0],q[1],5,0,Math.PI*2);ctx.stroke();}}
    drawDrone();
    // A restrained world orientation indicator in the corner.
    if(mode!=='top'){ctx.font='8px Segoe UI';ctx.fillStyle='#849b90';ctx.fillText('Z',width-26,height-53);ctx.strokeStyle='#8da894';ctx.beginPath();ctx.moveTo(width-23,height-27);ctx.lineTo(width-23,height-45);ctx.stroke();ctx.strokeStyle='#6d8e81';ctx.beginPath();ctx.moveTo(width-23,height-27);ctx.lineTo(width-38,height-20);ctx.moveTo(width-23,height-27);ctx.lineTo(width-9,height-20);ctx.stroke();}
  }
  function loop(){if(destroyed)return;paint();raf=requestAnimationFrame(loop);}
  function endDrag(){drag=null;}
  if(options.interactive!==false){
    listen('pointerdown',event=>{autoFit=false;drag={x:event.clientX,y:event.clientY,pan:event.shiftKey||mode==='top'};canvas.setPointerCapture?.(event.pointerId);});
    listen('pointermove',event=>{if(!drag)return;const dx=event.clientX-drag.x,dy=event.clientY-drag.y;if(drag.pan){const scale=distance/Math.max(1,Math.min(width,height));center[0]-=dx*scale;center[1]+=dy*scale;}else{azimuth-=dx*.008;elevation=Math.max(.12,Math.min(1.5,elevation+dy*.008));}drag.x=event.clientX;drag.y=event.clientY;follow=false;options.onFollowChange?.(false);dirty=true;});
    listen('pointerup',endDrag);listen('pointercancel',endDrag);listen('lostpointercapture',endDrag);
    listen('wheel',event=>{autoFit=false;event.preventDefault();distance=Math.max(3,Math.min(1200,distance*Math.exp(Math.max(-200,Math.min(200,event.deltaY))*.002)));dirty=true;},{passive:false});
    listen('keydown',event=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-','0'].includes(event.key))return;event.preventDefault();autoFit=false;if(event.key==='0'){follow=false;options.onFollowChange?.(false);fit();return;}if(event.key==='+'||event.key==='=')distance=Math.max(3,distance*.85);else if(event.key==='-')distance=Math.min(1200,distance*1.15);else if(mode==='top'){const d=distance*.04;if(event.key==='ArrowLeft')center[0]-=d;if(event.key==='ArrowRight')center[0]+=d;if(event.key==='ArrowUp')center[1]+=d;if(event.key==='ArrowDown')center[1]-=d;follow=false;options.onFollowChange?.(false);}else{if(event.key==='ArrowLeft')azimuth-=.1;if(event.key==='ArrowRight')azimuth+=.1;if(event.key==='ArrowUp')elevation=Math.min(1.5,elevation+.1);if(event.key==='ArrowDown')elevation=Math.max(.12,elevation-.1);}dirty=true;});
  }
  const observer=typeof ResizeObserver==='function'?new ResizeObserver(()=>{dirty=true;}):null;observer?.observe(canvas);
  raf=requestAnimationFrame(loop);
  return Object.freeze({
    setData(next){if(destroyed||!next||!Array.isArray(next.points)||!Array.isArray(next.trail))return false;const changed=data.points!==next.points;if(data.points.length&&!next.points.length){fitted=false;autoFit=options.autoFit===true;}data=next;if((!fitted||autoFit&&changed)&&data.points.length)fit();if(follow&&data.pose){center=[...data.pose.position];if(mode==='top'&&options.local)distance=19;}dirty=true;return true;},
    fit(){follow=false;options.onFollowChange?.(false);fit();},
    setMode(value){if(!['top','perspective'].includes(value))return false;mode=value;fit();dirty=true;return true;},
    setFollow(value){if(typeof value!=='boolean')return false;follow=value;if(value&&data.pose)center=[...data.pose.position];dirty=true;return true;},
    destroy(){destroyed=true;if(raf!==null)cancelAnimationFrame(raf);observer?.disconnect();for(const [type,handler,settings]of handlers)canvas.removeEventListener(type,handler,settings);}
  });
}
