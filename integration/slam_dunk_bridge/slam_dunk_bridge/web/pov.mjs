/** Local synthetic camera. Ray/box projection of the same collision scene.
 * No network, camera footage, WebGL library, or prerecorded animation.
 */
import {STATIC,WALLS} from './planner.mjs';
export function rayBox(origin,direction,box){
  let near=-Infinity,far=Infinity;
  for(let i=0;i<2;i++){
    if(Math.abs(direction[i])<1e-10){if(origin[i]<box[i]||origin[i]>box[i+2])return null;continue;}
    let a=(box[i]-origin[i])/direction[i],b=(box[i+2]-origin[i])/direction[i];
    if(a>b)[a,b]=[b,a];near=Math.max(near,a);far=Math.min(far,b);
  }
  if(far<Math.max(0,near))return null;
  return near>.001?near:far>.001?far:null;
}
export function createPov(canvas){
  const ctx=canvas.getContext('2d');if(!ctx)throw new Error('Canvas rendering is unavailable');
  let destroyed=false,lastFrame=null;
  return Object.freeze({
    setData(data){
      if(destroyed||!data?.position)return;
      const bounds=canvas.getBoundingClientRect(),w=Math.max(1,bounds.width),h=Math.max(1,bounds.height),ratio=Math.min(2,globalThis.devicePixelRatio||1);
      const frame=`${data.simTime}/${data.yaw}/${data.position.join(',')}/${data.obstacles?.length}/${w}/${h}/${ratio}`;if(frame===lastFrame)return;lastFrame=frame;
      const pixelWidth=Math.round(w*ratio),pixelHeight=Math.round(h*ratio);
      if(canvas.width!==pixelWidth||canvas.height!==pixelHeight){canvas.width=pixelWidth;canvas.height=pixelHeight;}
      ctx.setTransform(ratio,0,0,ratio,0,0);
      const horizon=h*.46,focal=w*.72,z=data.position[2],yaw=data.yaw??0,c=Math.cos(yaw),s=Math.sin(yaw);
      ctx.fillStyle='#142a31';ctx.fillRect(0,0,w,horizon);
      ctx.fillStyle='#253737';ctx.fillRect(0,horizon,w,h-horizon);
      // Floor depth bands, confined to the ground plane.
      for(let d=2;d<50;d+=2){const y=horizon+z*focal/d;if(y<h){ctx.strokeStyle='#92ada61a';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}}
      const obstacles=data.obstacles??STATIC,columns=Math.min(640,Math.ceil(w/2)),stripe=w/columns;
      for(let col=0;col<columns;col++){
        const x=(col+.5)*stripe,offset=(x-w/2)/focal,direction=[c-s*offset,s+c*offset];
        const hits=[];
        obstacles.forEach((box,index)=>{const depth=rayBox(data.position,direction,box);if(depth!==null&&depth>.03)hits.push({depth,box,index});});
        for(const actor of data.movers??[]){const [ax,ay]=actor.position,r=actor.radius,box=[ax-r,ay-r,ax+r,ay+r],depth=rayBox(data.position,direction,box);if(depth!==null&&depth>.03)hits.push({depth,box,index:-1});}
        hits.sort((a,b)=>b.depth-a.depth);
        for(const hit of hits){
          const {depth,box,index}=hit,custom=index>=STATIC.length,patrol=index<0,wall=index>=0&&index<WALLS.length,topZ=wall?2.7:2.2;
          const top=Math.max(0,horizon-(topZ-z)*focal/depth),bottom=Math.min(h,horizon+z*focal/depth);
          if(bottom<=top)continue;
          const px=data.position[0]+direction[0]*depth,py=data.position[1]+direction[1]*depth;
          const side=Math.min(Math.abs(px-box[0]),Math.abs(px-box[2]))<.015;
          const light=Math.max(.18,1/(1+depth*.06))*(side?.78:1);
          const color=patrol?[235,164,77]:custom?[219,117,86]:wall?[106,156,155]:[83,119,118];
          ctx.fillStyle=`rgb(${color.map(v=>Math.round(v*light)).join(',')})`;ctx.fillRect(col*stripe,top,stripe+.5,bottom-top);
          ctx.fillStyle=custom?'#ffcf9970':'#c2e9df25';ctx.fillRect(col*stripe,top,stripe+.5,Math.max(1,2/depth));
          const seam=Math.abs((side?py:px)%1);if(seam<.025||seam>.975){ctx.fillStyle='#0c1c2040';ctx.fillRect(col*stripe,top,stripe+.5,bottom-top);}
          if(custom||patrol){const band=top+(bottom-top)*.25;ctx.fillStyle=patrol?'#ffd281':'#342e2a';ctx.fillRect(col*stripe,band,stripe+.5,Math.max(2,(bottom-top)*.035));}
        }
      }
      // Route cue is a HUD bearing, not a claim of visible camera perception.
      const next=data.path?.[1]??data.goal;
      if(next){const bearing=Math.atan2(next[1]-data.position[1],next[0]-data.position[0])-yaw,angle=Math.atan2(Math.sin(bearing),Math.cos(bearing));
        ctx.save();ctx.translate(w/2,h*.78);ctx.rotate(angle);ctx.strokeStyle='#c9f76b';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(-10,8);ctx.lineTo(0,-7);ctx.lineTo(10,8);ctx.stroke();ctx.restore();}
      ctx.strokeStyle='#d9f5ef75';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(w/2-12,horizon);ctx.lineTo(w/2-4,horizon);ctx.moveTo(w/2+4,horizon);ctx.lineTo(w/2+12,horizon);ctx.moveTo(w/2,horizon-12);ctx.lineTo(w/2,horizon-4);ctx.stroke();
      ctx.font='11px monospace';ctx.fillStyle='#d2e8df';ctx.fillText(`ALT ${z.toFixed(1)} m   HDG ${((yaw*180/Math.PI+360)%360).toFixed(0)}°`,20,h-25);
    },
    destroy(){destroyed=true;ctx.clearRect(0,0,canvas.width,canvas.height);}
  });
}
