/** Shared furnished room geometry, used by planning, rendering and replay. */
export const ROOM_BOUNDS=Object.freeze([-16,-12,16,12]);
export const ROOM_WALLS=[[-16,-12,16,-11.8],[15.8,-12,16,12],[-16,11.8,16,12],[-16,-12,-15.8,12]];
export const ROOM_FIXTURES=[
  {kind:'wall',label:'Partition',rect:[-4,3,-3.7,10],height:2.7,color:'#4b7c88'},
  {kind:'wall',label:'Partition',rect:[4,-10,4.3,-3.5],height:2.7,color:'#4b7c88'},
  {kind:'wall',label:'Partition',rect:[7,7,14,7.3],height:2.7,color:'#4b7c88'},
  {kind:'sofa',label:'Sofa',rect:[-13.5,5,-10,6.6],height:1.1,color:'#4f9790'},
  {kind:'table',label:'Coffee table',rect:[-12,8,-9.5,9.6],height:.85,color:'#c59d73'},
  {kind:'shelf',label:'Storage',rect:[-14,-1,-12.5,2],height:2.1,color:'#718b91'},
  {kind:'table',label:'Work desk',rect:[9,-9,13,-7.3],height:.95,color:'#c59d73'},
  {kind:'shelf',label:'Bookshelf',rect:[11,9,14,10],height:2.2,color:'#a28465'},
  {kind:'table',label:'Work island',rect:[0,7,2.4,9],height:1,color:'#a0aaa1'},
  {kind:'sofa',label:'Armchair',rect:[-7,7,-5.8,8.3],height:1,color:'#598b8c'}
];
export const ROOM_OBSTACLES=[...ROOM_WALLS,...ROOM_FIXTURES.map(o=>o.rect)];
