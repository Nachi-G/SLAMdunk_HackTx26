/** Browser SANDO approximation: inflated A*, cubic segments and temporal checks.
 * Reference: car_sando_planner/src/sando/gurobi_solver.cpp (getPos/Vel/Accel/Jerk)
 * and config/sando.yaml (inflation_hgp, dyn_base_inflation_m).
 * This does not run Gurobi, MINVO optimization or ROS control.
 */
export const WALLS = [
  [-13,-9,13,-8.8],[12.8,-9,13,9],[-13,8.8,13,9],[-13,-9,-12.8,9],
  [-3.1,-9,-2.9,-3],[-3.1,0,-2.9,9],[3.9,-9,4.1,0],[3.9,3,4.1,9],
  [-13,1.9,-9,2.1],[-6,1.9,-3,2.1],[4,-2.1,9,-1.9],[11,-2.1,13,-1.9]
];
export const FURNITURE = [
  [-11,-7,-8,-6],[-6.5,-7,-5,-4.5],[-11,4,-9,6.5],[-6,6,-4.5,7],
  [-1,3,1,5.5],[.5,-7,2.5,-5.5],[.5,-.1,2,1.3],[6,4,7.5,6.5],
  [9,5.5,11,7],[6,-7,7.5,-4.5],[10,-6,11.5,-4.5]
];
export const STATIC = [...WALLS,...FURNITURE];
export const MISSIONS = [[-9,-4,1.8],[-7,5,1.8],[1,7,1.8],[9,3.5,1.8],[9,-7,1.8],[-1,-5,1.8]];
export const BODY_RADIUS=.24;
export function moversAt(t) {
  return [{id:'P01',position:[-7+2.1*Math.sin(t*.42),-.8,1.2],radius:.45},
    {id:'P02',position:[6.6,1.3+1.3*Math.sin(t*.55+1),1.2],radius:.5},
    {id:'P03',position:[.2+1.5*Math.sin(t*.38),-3.2,1.2],radius:.4}];
}
/** Minimum XY separation during simultaneous linear motions, including endpoints. */
export function sweptDistance(a0,a1,b0,b1){
  const x=a0[0]-b0[0],y=a0[1]-b0[1],dx=(a1[0]-a0[0])-(b1[0]-b0[0]),dy=(a1[1]-a0[1])-(b1[1]-b0[1]),length=dx*dx+dy*dy;
  const u=length>1e-12?Math.max(0,Math.min(1,-(x*dx+y*dy)/length)):0;
  return Math.hypot(x+dx*u,y+dy*u);
}
export function rectDistance(p,r) {return Math.hypot(Math.max(r[0]-p[0],0,p[0]-r[2]),Math.max(r[1]-p[1],0,p[1]-r[3]));}
export function clearance(p,t=0,dynamic=true,actors=moversAt,obstacles=STATIC) {
  return Math.min(...obstacles.map(r=>rectDistance(p,r)-BODY_RADIUS),...(dynamic?actors(t).map(o=>Math.hypot(p[0]-o.position[0],p[1]-o.position[1])-o.radius-BODY_RADIUS):[Infinity]));
}
const boundsCache=new WeakMap();
export function roomBounds(obstacles=STATIC){
  if(boundsCache.has(obstacles))return boundsCache.get(obstacles);
  const walls=obstacles.slice(0,4);if(!walls.length)return [-13,-9,13,9];
  const bounds=Object.freeze([Math.min(...walls.map(r=>r[0])),Math.min(...walls.map(r=>r[1])),Math.max(...walls.map(r=>r[2])),Math.max(...walls.map(r=>r[3]))]);
  boundsCache.set(obstacles,bounds);return bounds;
}
export function free(p,margin,t=0,dynamic=false,dynamicBase=.1,actors=moversAt,obstacles=STATIC) {
  const bounds=roomBounds(obstacles);
  if(!p.every(Number.isFinite)||p[0]<bounds[0]+.5||p[0]>bounds[2]-.5||p[1]<bounds[1]+.5||p[1]>bounds[3]-.5)return false;
  if(obstacles.some(r=>rectDistance(p,r)<BODY_RADIUS+margin))return false;
  return !dynamic||actors(t).every(o=>Math.hypot(p[0]-o.position[0],p[1]-o.position[1])>=o.radius+BODY_RADIUS+margin+dynamicBase);
}
export function segmentFree(a,b,margin,t=0,duration=0,dynamic=false,dynamicBase=.1,actors=moversAt,obstacles=STATIC) {
  const n=Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1])/.08),Math.ceil(duration/.04));
  for(let i=0;i<=n;i++){const u=i/n;if(!free([a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u],margin,t+duration*u,dynamic,dynamicBase,actors,obstacles))return false;}
  return true;
}
/** Cubic p(u)=p0+(3u²−2u³)Δp. Duration bounds speed, acceleration and jerk. */
export function cubicSegment(a,b,vmax,amax=2,jmax=4) {
  const delta=b.map((v,i)=>v-a[i]),d=Math.hypot(...delta);
  const duration=Math.max(.1,1.5*d/vmax,Math.sqrt(6*d/amax),Math.cbrt(12*d/jmax));
  return {duration,sample(time){const u=Math.max(0,Math.min(1,time/duration)),s=3*u*u-2*u*u*u;return {position:a.map((v,i)=>v+delta[i]*s),velocity:delta.map(v=>v*6*u*(1-u)/duration)};}};
}
/** 8-connected graph with swept edges; diagonal corner cutting is forbidden. */
export function planPath(start,goal,margin,t=0,dynamicBase=.1,actors=moversAt,obstacles=STATIC) {
  if(!free(start,margin,t,false,dynamicBase,actors,obstacles)||!free(goal,margin,t,false,dynamicBase,actors,obstacles))return [];
  const key=p=>`${p[0]},${p[1]}`, grid=p=>[Math.round(p[0]*2)/2,Math.round(p[1]*2)/2,p[2]];
  const nearest=p=>{const g=grid(p),candidates=[];for(const dx of [-.5,0,.5])for(const dy of [-.5,0,.5]){const q=[g[0]+dx,g[1]+dy,p[2]];if(segmentFree(p,q,margin,t,0,true,dynamicBase,actors,obstacles))candidates.push(q);}return candidates.sort((a,b)=>Math.hypot(a[0]-p[0],a[1]-p[1])-Math.hypot(b[0]-p[0],b[1]-p[1]))[0];};
  const first=nearest(start),last=nearest(goal);if(!first||!last)return [];
  if(!segmentFree(start,first,margin,t,0,false,dynamicBase,actors,obstacles)||!segmentFree(last,goal,margin,t,0,false,dynamicBase,actors,obstacles))return [];
  const open=[{p:first,g:0,f:Math.hypot(first[0]-last[0],first[1]-last[1])}],cost=new Map([[key(first),0]]),parent=new Map(),closed=new Set();
  while(open.length){
    let best=0;for(let i=1;i<open.length;i++)if(open[i].f<open[best].f)best=i;
    const node=open.splice(best,1)[0],k=key(node.p);if(closed.has(k))continue;closed.add(k);
    if(k===key(last)){
      const route=[goal,node.p];let current=k;while(parent.has(current)){const p=parent.get(current);route.push(p);current=key(p);}route.push(start);route.reverse();
      const smooth=[start];let i=0;while(i<route.length-1){let j=route.length-1;while(j>i+1&&(!segmentFree(route[i],route[j],margin,t,2,true,dynamicBase,actors,obstacles)||!segmentFree(route[i],route[j],margin,t,0,true,dynamicBase,actors,obstacles)))j--;smooth.push(route[j]);i=j;}return smooth.filter((p,i,a)=>!i||Math.hypot(p[0]-a[i-1][0],p[1]-a[i-1][1])>.001);
    }
    for(const dx of [-.5,0,.5])for(const dy of [-.5,0,.5]){
      if(!dx&&!dy)continue;const p=[node.p[0]+dx,node.p[1]+dy,start[2]],pk=key(p);
      if(closed.has(pk)||!segmentFree(node.p,p,margin,t,0,true,dynamicBase,actors,obstacles))continue;
      // Predicted near-term occupancy biases the spatial route around patrols.
      const near=actors(t+1).some(o=>Math.hypot(p[0]-o.position[0],p[1]-o.position[1])<o.radius+BODY_RADIUS+margin+dynamicBase+.4);
      const g=node.g+Math.hypot(dx,dy)+(near?2:0);if(g>=(cost.get(pk)??Infinity))continue;
      cost.set(pk,g);parent.set(pk,node.p);open.push({p,g,f:g+Math.hypot(p[0]-last[0],p[1]-last[1])});
    }
  }
  return [];
}
export function geometryPoints(seed=26,obstacles=STATIC) {
  const points=[];for(const [index,r] of obstacles.entries()){
    const height=index<WALLS.length?2.7:1.4;
    const edges=[[[r[0],r[1]],[r[2],r[1]]],[[r[2],r[1]],[r[2],r[3]]],[[r[2],r[3]],[r[0],r[3]]],[[r[0],r[3]],[r[0],r[1]]]];
    for(const [a,b]of edges){const n=Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1])/.25));for(let i=0;i<=n;i++)for(let z=0;z<=height;z+=.3){const j=Math.sin(i*17+z*31+seed)*.008;points.push([a[0]+(b[0]-a[0])*i/n+j,a[1]+(b[1]-a[1])*i/n+j,z]);}}
  }return points;
}
