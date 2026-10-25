#!/usr/bin/env node
// Deterministic, dependency-free authoring source for Gazebo Harmonic (gz-sim 8).
import {mkdirSync, writeFileSync} from 'node:fs';
import {dirname, join} from 'node:path';
import {fileURLToPath} from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const models = [];
const bounds = [];
const palette = {
  wall: '.82 .83 .78 1', wood: '.40 .23 .12 1', lightwood: '.66 .46 .27 1',
  blue: '.12 .32 .58 1', green: '.22 .46 .31 1', red: '.65 .17 .12 1',
  white: '.88 .88 .84 1', metal: '.32 .36 .40 1', black: '.07 .09 .11 1',
  yellow: '.95 .65 .10 1', fabric: '.55 .35 .30 1', leaf: '.15 .42 .14 1',
};
const n = x => Number(x.toFixed(5));
const vec = a => a.map(n).join(' ');
const pose = (x,y,z,yaw=0) => `${vec([x,y,z])} 0 0 ${n(yaw)}`;
const material = color => `<material><ambient>${palette[color] || color}</ambient><diffuse>${palette[color] || color}</diffuse></material>`;
const box = (name,x,y,z,sx,sy,sz,color='wood',collision=true) => ({name,x,y,z,sx,sy,sz,color,collision,kind:'box'});
const cylinder = (name,x,y,z,radius,length,color='metal',collision=true) => ({name,x,y,z,radius,length,color,collision,kind:'cylinder'});
function geometry(p) {
  return p.kind === 'box' ? `<box><size>${vec([p.sx,p.sy,p.sz])}</size></box>` : `<cylinder><radius>${p.radius}</radius><length>${p.length}</length></cylinder>`;
}
function shapes(parts) {
  return parts.map(p => {
    const at = `<pose>${pose(p.x,p.y,p.z)}</pose><geometry>${geometry(p)}</geometry>`;
    return `<visual name="${p.name}_visual">${at}${material(p.color)}</visual>` +
      (p.collision ? `<collision name="${p.name}_collision">${at}</collision>` : '');
  }).join('\n');
}
function add(name,x,y,parts) {
  models.push(`<model name="${name}"><static>true</static><pose>${pose(x,y,0)}</pose><link name="body">${shapes(parts)}</link></model>`);
  for (const p of parts) {
    if (!p.collision) continue;
    const sx = p.sx ?? p.radius*2, sy = p.sy ?? p.radius*2, sz = p.sz ?? p.length;
    bounds.push({name:`${name}/${p.name}`,min:[x+p.x-sx/2,y+p.y-sy/2,p.z-sz/2],max:[x+p.x+sx/2,y+p.y+sy/2,p.z+sz/2]});
  }
}
function solid(name,x,y,z,sx,sy,sz,color='wood') {
  add(name,x,y,[box('body',0,0,z,sx,sy,sz,color)]);
}
function table(name,x,y,sx=1.6,sy=.9,h=.78) {
  const parts = [box('top',0,0,h-.05,sx,sy,.1,'lightwood')];
  for (const [i,dx] of [-1,1].entries()) for (const [j,dy] of [-1,1].entries())
    parts.push(box(`leg_${i}_${j}`,dx*(sx/2-.1),dy*(sy/2-.1),(h-.1)/2,.1,.1,h-.1));
  add(name,x,y,parts);
}
function chair(name,x,y) {
  add(name,x,y,[box('seat',0,0,.46,.5,.5,.1,'blue'),box('back',0,.23,.78,.5,.08,.6,'blue'),
    ...[-1,1].flatMap((dx,i)=>[-1,1].map((dy,j)=>box(`leg_${i}_${j}`,dx*.19,dy*.19,.205,.06,.06,.41,'metal')))]);
}
function shelf(name,x,y,width=1.4) {
  add(name,x,y,[box('left',-width/2+.04,0,1,.08,.45,2),box('right',width/2-.04,0,1,.08,.45,2),
    box('back',0,.21,1,width,.04,2),...[0,.5,1,1.5,2].map((z,i)=>box(`shelf_${i}`,0,0,z+.025,width,.45,.05))]);
  for(let i=0;i<8;i++) solid(`${name}_book_${i}`,x-width/2+.16+i*.14,y,1.18,.08,.25,.27,i%2?'blue':'red');
}
function plant(name,x,y) {
  add(name,x,y,[cylinder('pot',0,0,.2,.22,.4,'fabric'),cylinder('stem',0,0,.65,.035,.6,'wood'),
    cylinder('foliage',0,0,1,.32,.6,'leaf')]);
}
function bed(name,x,y,color) {
  add(name,x,y,[box('frame',0,0,.23,2,2.5,.36),box('mattress',0,0,.51,1.95,2.4,.2,'white'),
    box('duvet',0,-.35,.64,1.96,1.65,.08,color),box('pillow_left',-.48,.82,.67,.7,.4,.15,'white'),
    box('pillow_right',.48,.82,.67,.7,.4,.15,'white'),box('headboard',0,1.23,.65,2.08,.12,1.3)]);
}

// 24 x 18 m; 3.2 m walls; 2.4 m corridor; all room doorways 1.6 x 2.2 m.
solid('foundation',0,0,-.12,26,20,.24,'metal');
solid('north_wall',0,9,1.6,24.2,.2,3.2,'wall');
solid('south_wall',0,-9,1.6,24.2,.2,3.2,'wall');
solid('east_wall',12,0,1.6,.2,18,3.2,'wall');
for(const side of [-1,1]) solid(`entrance_side_${side<0?'south':'north'}`,-12,side*4.95,1.6,.2,8.1,3.2,'wall');
solid('entrance_header',-12,0,2.7,.2,1.8,1,'wall');
for(const y of [-1.2,1.2]) {
  for(let i=0;i<4;i++) {
    const x=-9+i*6, prefix=`partition_${y<0?'south':'north'}_${i}`;
    solid(`${prefix}_left`,x-1.9,y,1.6,2.2,.16,3.2,'wall');
    solid(`${prefix}_right`,x+1.9,y,1.6,2.2,.16,3.2,'wall');
    solid(`${prefix}_header`,x,y,2.7,1.6,.16,1,'wall');
  }
  for(const x of [-6,0,6]) solid(`room_divider_${x}_${y<0?'south':'north'}`,x,Math.sign(y)*5.1,1.6,.16,7.8,3.2,'wall');
}
const rooms = [
  ['living',-9,5.1,'.51 .44 .34 1'],['kitchen',-3,5.1,'.60 .63 .60 1'],
  ['dining',3,5.1,'.52 .40 .27 1'],['office',9,5.1,'.37 .45 .48 1'],
  ['bedroom',-9,-5.1,'.40 .45 .56 1'],['guest',-3,-5.1,'.46 .53 .43 1'],
  ['bathroom',3,-5.1,'.61 .70 .72 1'],['utility',9,-5.1,'.48 .48 .46 1'],
];
for(const [name,x,y,color] of rooms) {
  add(`${name}_floor`,x,y,[box('finish',0,0,.003,5.82,7.62,.006,color,false)]);
  // Tall, shallow wall accents add non-repeating RGB landmarks.
  add(`${name}_art`,x,Math.sign(y)*8.88,[box('frame',0,0,1.8,1.1,.035,.65,'wood'),box('print',0,-Math.sign(y)*.025,1.8,.98,.015,.53,Math.sign(x)===1?'blue':'yellow')]);
}
add('corridor_floor',0,0,[box('finish',0,0,.004,23.8,2.22,.008,'.69 .64 .51 1',false)]);
add('spawn_marker',-11,0,[cylinder('disk',0,0,.012,.35,.012,'green',false)]);

// Living room: keep the central 3.4 m diameter patrol sweep clear.
add('living_sofa',-9,7.9,[box('base',0,0,.36,2.8,.95,.5,'blue'),box('back',0,.42,.86,2.8,.16,.95,'blue'),
  box('left_arm',-1.34,0,.66,.15,.95,.5,'blue'),box('right_arm',1.34,0,.66,.15,.95,.5,'blue')]);
table('living_coffee_table',-9,6.65,1.5,.65,.45);
solid('living_tv_console',-11.45,4.5,.35,.65,2.2,.7);
solid('living_tv',-11.45,4.5,1.15,.12,1.5,.9,'black');
shelf('living_bookcase',-7.1,8.5,1.4);
plant('living_plant',-6.7,2.1);
solid('living_ottoman',-6.8,6.4,.3,.8,.8,.6,'fabric');

// Kitchen: counters, appliances, island, small countertop objects.
for(let i=0;i<4;i++) {
  solid(`kitchen_cabinet_${i}`,-5.05+i*1.2,8.35,.46,1.15,.95,.92,'white');
  solid(`kitchen_worktop_${i}`,-5.05+i*1.2,8.35,.95,1.18,1,.06,'metal');
  solid(`kitchen_upper_${i}`,-5.05+i*1.2,8.65,2.1,1.12,.45,.8,'lightwood');
}
solid('kitchen_fridge',-5.25,6.5,1,.95,.95,2,'white');
solid('kitchen_oven_front',-1.45,7.86,.52,.75,.025,.48,'black');
for(let i=0;i<4;i++) add(`hob_${i}`,-1.45+(i%2? .25:-.25),8.35+(i<2?-.23:.23),[cylinder('ring',0,0,.995,.15,.025,'black')]);
table('kitchen_island',-2.6,5.5,2.4,1.1,.95);
add('kitchen_sink',-3.85,8.35,[box('basin',0,0,.99,.7,.55,.02,'blue'),cylinder('tap',0,.3,1.13,.035,.3,'metal')]);
add('kitchen_kettle',-2.65,8.35,[cylinder('body',0,0,1.13,.14,.3,'red')]);
chair('kitchen_stool_a',-3.3,4.55); chair('kitchen_stool_b',-1.9,4.55);
plant('kitchen_herbs',-5.35,2.2);

table('dining_table',3,5.5,2.4,1.25,.8);
for(let i=0;i<3;i++) for(const side of [-1,1]) chair(`dining_chair_${i}_${side<0?'south':'north'}`,2.1+i*.9,5.5+side*1.05);
for(let i=0;i<4;i++) add(`dining_plate_${i}`,2.4+(i%2)*1.2,5.5+(i<2?-.34:.34),[cylinder('plate',0,0,.813,.17,.025,'white')]);
solid('dining_sideboard',3,8.45,.55,3,.65,1.1);
plant('dining_plant',5.25,2.1);

table('office_desk',8,7.7,2.4,.9,.78);
solid('office_monitor',8,7.85,1.16,.9,.08,.55,'black');
solid('office_keyboard',8,7.4,.805,.55,.18,.025,'black');
chair('office_chair',8,6.7);
shelf('office_shelf',10.75,8.45,1.5);
solid('office_filing_cabinet',11.3,4.8,.7,.75,.75,1.4,'metal');
table('office_workbench',9.5,3.4,2,.85,.9);
for(let i=0;i<3;i++) solid(`office_equipment_${i}`,9+i*.45,3.4,1.05,.3,.3,.3,i%2?'yellow':'metal');
plant('office_plant',6.7,2.1);

for(const [prefix,x,color] of [['bedroom',-9,'blue'],['guest',-3,'green']]) {
  bed(`${prefix}_bed`,x,-6.8,color);
  for(const side of [-1,1]) {
    solid(`${prefix}_nightstand_${side}`,x+side*1.45,-7.5,.3,.55,.55,.6);
    add(`${prefix}_lamp_${side}`,x+side*1.45,-7.5,[cylinder('stem',0,0,.84,.035,.48,'metal'),cylinder('shade',0,0,1.08,.2,.25,'yellow')]);
  }
  solid(`${prefix}_wardrobe`,x+1.9,-3.9,1.1,1.2,.65,2.2,'lightwood');
  table(`${prefix}_desk`,x-1.5,-3.5,1.3,.7,.75);
  chair(`${prefix}_chair`,x-1.5,-4.3);
  plant(`${prefix}_plant`,x-2.25,-8.3);
}

// Bathroom fixtures retain hollow spaces / overhangs for depth testing.
add('bath_tub',1.6,-7.5,[box('bottom',0,0,.12,1.25,2.2,.24,'white'),box('left',-.56,0,.4,.13,2.2,.55,'white'),
  box('right',.56,0,.4,.13,2.2,.55,'white'),box('head',0,1.03,.4,1.25,.14,.55,'white'),box('foot',0,-1.03,.4,1.25,.14,.55,'white')]);
solid('bath_vanity',4.65,-8.35,.45,1.6,.75,.9,'white');
solid('bath_mirror',4.65,-8.87,1.55,1.4,.025,.8,'metal');
add('bath_toilet',4.9,-5.6,[cylinder('pedestal',0,0,.22,.2,.44,'white'),cylinder('seat',0,0,.46,.32,.1,'white'),box('tank',0,-.3,.65,.65,.25,.7,'white')]);
solid('bath_linen_cabinet',1,-3,1,.7,.65,2,'white');
add('bath_towel_rail',5.85,-3.8,[box('rail',0,0,1.1,.08,1.2,.08,'metal'),box('towel',-.05,0,.9,.04,.65,.5,'blue')]);

for(let i=0;i<3;i++) shelf(`utility_rack_${i}`,7.1+i*1.9,-8.5,1.5);
solid('utility_washer',7.1,-5.4,.48,.95,.95,.96,'white');
solid('utility_dryer',8.25,-5.4,.48,.95,.95,.96,'white');
for(let i=0;i<7;i++) {
  const x=10.1+(i%2)*.8, y=-7+Math.floor(i/2)*.85;
  solid(`utility_crate_${i}`,x,y,.25,.6,.6,.5,i%2?'lightwood':'yellow');
}
solid('utility_tall_box',11.3,-2.3,.8,.6,.6,1.6,'fabric');
add('utility_bucket',7,-2.4,[cylinder('bucket',0,0,.24,.25,.48,'blue')]);

// World-anchored revolute joints create repeatable, collidable circular motion.
// These are controlled obstacles, not autonomous wheeled robot models.
const movers = [
  {name:'living_patrol',x:-9,y:4.4,z:0,r:1.3,radius:.27,height:.65,speed:.22,color:'red'},
  {name:'hall_patrol',x:3,y:0,z:0,r:.52,radius:.22,height:1.2,speed:-.35,color:'yellow'},
];
function movingModel(m, enabled) {
  const parts=[cylinder('body',m.r,0,m.height/2+.015,m.radius,m.height,m.color),
    cylinder('cap',m.r,0,m.height+.04,m.radius*.6,.05,'black')];
  return `<model name="${m.name}"><pose>${pose(m.x,m.y,m.z)}</pose><link name="body"><gravity>false</gravity>
    <inertial><pose>${pose(m.r,0,m.height/2+.015)}</pose><mass>5</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>${shapes(parts)}</link>
    <joint name="orbit" type="revolute"><pose relative_to="__model__">0 0 0 0 0 0</pose><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>100</effort><velocity>2</velocity></limit></axis></joint>
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController"><joint_name>orbit</joint_name><initial_velocity>${enabled?m.speed:0}</initial_velocity></plugin></model>`;
}
function fan(enabled) {
  return `<model name="dining_ceiling_fan"><pose>${pose(3,5.5,2.8)}</pose><link name="rotor"><gravity>false</gravity>
    <inertial><mass>1</mass><inertia><ixx>.12</ixx><iyy>.12</iyy><izz>.24</izz></inertia></inertial>
    ${shapes([box('blade_x',0,0,0,2.2,.14,.05,'wood'),box('blade_y',0,0,0,.14,2.2,.05,'wood'),cylinder('hub',0,0,0,.12,.14,'metal')])}</link>
    <joint name="spin" type="revolute"><parent>world</parent><child>rotor</child><axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>20</effort><velocity>5</velocity></limit></axis></joint>
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController"><joint_name>spin</joint_name><initial_velocity>${enabled?1.4:0}</initial_velocity></plugin></model>`;
}
function validateLayout() {
  const names=models.map(m=>m.match(/<model name="([^"]+)"/)[1]);
  if(new Set(names).size!==names.length) throw Error('Duplicate model name');
  for(const b of bounds) if([...b.min,...b.max].some(v=>!Number.isFinite(v)) || b.min.some((v,i)=>v>=b.max[i])) throw Error(`Invalid geometry: ${b.name}`);
  // Conservative AABBs cover the complete swept volume, including body caps.
  for(const m of movers) {
    const r=m.r+m.radius+.03;
    const hit=bounds.filter(b=>b.max[2]>.015 && b.min[2]<m.height+.065 &&
      b.min[0]<m.x+r && b.max[0]>m.x-r && b.min[1]<m.y+r && b.max[1]>m.y-r);
    if(hit.length) throw Error(`${m.name} sweep overlaps ${hit.map(b=>b.name).join(', ')}`);
  }
  // An unobstructed spawn cylinder and routes into each room, below door headers.
  const routes=[{name:'spawn',min:[-11.4,-.4,.02],max:[-10.6,.4,1.9]},
    ...rooms.map(([name,x,y])=>({name:`${name} entrance`,min:[x-.55,Math.min(0,Math.sign(y)*2),.05],max:[x+.55,Math.max(0,Math.sign(y)*2),2.05]}))];
  for(const route of routes) {
    const hit=bounds.filter(b=>b.min.every((v,i)=>v<route.max[i])&&b.max.every((v,i)=>v>route.min[i]));
    if(hit.length) throw Error(`${route.name} obstructed by ${hit.map(b=>b.name).join(', ')}`);
  }
  return {rooms:rooms.length,staticModels:models.length,staticCollisionShapes:bounds.length,movingModels:3,doorWidth:1.6,doorHeight:2.2,spawn:[-11,0,1]};
}
function world(enabled) {
  return `<?xml version="1.0"?>
<!-- Generated by generate_house.mjs. Units: metres, radians; Z up. -->
<sdf version="1.9"><world name="house_test">
  <gravity>0 0 -9.81</gravity>
  <physics name="step" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <scene><ambient>.65 .65 .65 1</ambient><background>.18 .22 .28 1</background><shadows>true</shadows></scene>
  <light name="sun" type="directional"><pose>0 0 12 0 0 0</pose><cast_shadows>true</cast_shadows><diffuse>.85 .85 .82 1</diffuse><specular>.2 .2 .2 1</specular><direction>-.3 .2 -1</direction></light>
  ${rooms.map(([name,x,y])=>`<light name="${name}_light" type="point"><pose>${pose(x,y,3.1)}</pose><cast_shadows>false</cast_shadows><diffuse>.7 .7 .65 1</diffuse><specular>.1 .1 .1 1</specular><attenuation><range>9</range><constant>.8</constant><linear>.1</linear><quadratic>.03</quadratic></attenuation></light>`).join('\n')}
  ${models.join('\n')}
  ${movers.map(m=>movingModel(m,enabled)).join('\n')}
  ${fan(enabled)}
</world></sdf>\n`;
}
const report=validateLayout();
mkdirSync(join(here,'worlds'),{recursive:true});
for(const [file,enabled] of [['house.sdf',true],['house_static.sdf',false]]) writeFileSync(join(here,'worlds',file),world(enabled));
writeFileSync(join(here,'layout.json'),JSON.stringify({...report,rooms:rooms.map(([name,x,y])=>({name,center:[x,y],size:[5.84,7.64]})),movers},null,2)+'\n');
// A top view derived from the same collision geometry, suitable for route planning.
const px=x=>n(520+x*35), py=y=>n(380-y*35);
const svg=`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1040 790">
<rect width="1040" height="790" fill="#f5f3ed"/>
<text x="520" y="32" text-anchor="middle" font-family="sans-serif" font-size="22" fill="#172f40">SLAMdunk · Harmonic house test world</text>
${rooms.map(([name,x,y])=>`<rect x="${px(x-2.92)}" y="${py(y+3.82)}" width="204.4" height="267.4" fill="#e3e3d8"/>`).join('')}
${bounds.filter(b=>b.max[2]>.02 && b.min[2]<2.2).map(b=>`<rect x="${px(b.min[0])}" y="${py(b.max[1])}" width="${n((b.max[0]-b.min[0])*35)}" height="${n((b.max[1]-b.min[1])*35)}" fill="${/wall|partition|divider|entrance/.test(b.name)?'#263d49':'#967d5b'}" stroke="#f5f3ed" stroke-width=".4"/>`).join('')}
${rooms.map(([name,x,y])=>`<text x="${px(x)}" y="${py(Math.sign(y)*2.45)}" text-anchor="middle" font-family="sans-serif" font-size="14" fill="#172f40">${name.toUpperCase()}</text>`).join('')}
${movers.map(m=>`<circle cx="${px(m.x)}" cy="${py(m.y)}" r="${m.r*35}" fill="none" stroke="#c43b2e" stroke-width="2" stroke-dasharray="5 4"/><circle cx="${px(m.x+m.r)}" cy="${py(m.y)}" r="${m.radius*35}" fill="#c43b2e"/>`).join('')}
<circle cx="${px(-11)}" cy="${py(0)}" r="11" fill="#248653"/><text x="${px(-11)}" y="${py(0)+27}" text-anchor="middle" font-family="sans-serif" font-size="11">SPAWN</text>
<text x="520" y="748" text-anchor="middle" font-family="sans-serif" font-size="15" fill="#172f40">24 × 18 m · 8 rooms · 1.6 m doorways · 2.4 m hallway · Z up</text>
<text x="520" y="772" text-anchor="middle" font-family="sans-serif" font-size="13" fill="#52636c">Red dashed: moving obstacle paths. Roof omitted. Furniture shown from collision geometry.</text>
</svg>\n`;
writeFileSync(join(here,'floorplan.svg'),svg);
console.log(JSON.stringify(report,null,2));
