import {runIntegrationTests,runPinInteractionTests,runQueuedPinTest,runMissionButtonTests} from './ui_integration.test.mjs';
import {runModelTests} from './model.test.mjs';
import {runHoldStateTests} from './hold_state.test.mjs';
// Each suite prints its own named case totals; assertions are not case counts.
console.log('Model and adapter:', JSON.stringify(await runModelTests()));
console.log('Hold steering:', JSON.stringify(await runHoldStateTests()));
console.log('UI integration:', JSON.stringify(runIntegrationTests()));

console.log('Pin interaction:',JSON.stringify(runPinInteractionTests()));

console.log('Queued pin:',await runQueuedPinTest());

console.log('Mission buttons:',runMissionButtonTests());
