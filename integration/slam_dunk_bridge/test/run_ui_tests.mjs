import {runIntegrationTests} from './ui_integration.test.mjs';
import {runModelTests} from './model.test.mjs';
import {runHoldStateTests} from './hold_state.test.mjs';
// Each suite prints its own named case totals; assertions are not case counts.
console.log('Model and adapter:', JSON.stringify(await runModelTests()));
console.log('Hold steering:', JSON.stringify(await runHoldStateTests()));
console.log('UI integration:', JSON.stringify(runIntegrationTests()));
const {runPlannerTests}=await import('./planner.test.mjs');
console.log('Obstacle planner:',JSON.stringify(runPlannerTests()));
const {runArenaTests}=await import('./arena.test.mjs');
console.log('Judge arena and POV:',JSON.stringify(runArenaTests()));
const {runRoomTests}=await import('./room.test.mjs');
console.log('Furnished flight room:',JSON.stringify(runRoomTests()));
const {runCollisionTests}=await import('./collision.test.mjs');
console.log('Swept collision avoidance:',JSON.stringify(runCollisionTests()));
const {runMissionMapTests}=await import('./mission_map.test.mjs');
console.log('Mission map and recording:',JSON.stringify(runMissionMapTests()));
const {runBackupTests}=await import('./backup_integration.test.mjs');
console.log('Combined Backup dashboard:',JSON.stringify(runBackupTests()));
