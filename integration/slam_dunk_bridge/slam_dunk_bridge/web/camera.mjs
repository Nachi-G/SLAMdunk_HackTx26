/** Browser-native RGB stream display. MJPEG does not expose frame timestamps:
 * onload is never treated as a per-frame heartbeat or proof of camera freshness.
 */
export function createCameraFeed({image, empty, status, note, baseUrl=globalThis.location?.href}={}) {
  if(!image||!empty||!status||!note)throw new TypeError('Camera display elements are required');
  let source=null,visible=true,generation=0,destroyed=false;
  function detach() {
    generation++;
    image.onload=null;image.onerror=null;
    image.removeAttribute('src');image.hidden=true;
  }
  function show() {
    detach();
    if(!source||!visible||destroyed){
      empty.hidden=false;
      status.textContent=source&&!visible?'FEED SUSPENDED':'NOT CONNECTED';
      note.textContent=source&&!visible?'Camera pauses outside Live operations.':'No RGB stream connected. Map data is not camera footage.';
      return;
    }
    const current=generation;
    // MJPEG may render indefinitely without firing load. Do not cover its image
    // waiting for load; the panel explicitly reports unverified freshness.
    image.hidden=false;empty.hidden=true;
    status.textContent='STREAM CONFIGURED';
    note.textContent='RGB stream · capture age and frame freshness unavailable';
    image.onload=()=>{if(current!==generation||destroyed)return;status.textContent='STREAM RECEIVED';};
    image.onerror=()=>{
      if(current!==generation||destroyed)return;
      image.hidden=true;empty.hidden=false;status.textContent='CAMERA UNAVAILABLE';
      note.textContent='No image received. Check the stream URL and camera publisher.';
    };
    image.src=source;
  }
  show();
  return Object.freeze({
    connect(raw) {
      if(destroyed||typeof raw!=='string'||!raw.trim()||raw.length>2048)return false;
      let url;try{url=new URL(raw,baseUrl);}catch{return false;}
      if(!['http:','https:'].includes(url.protocol)||url.username||url.password||url.hash)return false;
      if(globalThis.location?.protocol==='https:'&&url.protocol!=='https:')return false;
      source=url.href;show();return true;
    },
    disconnect(){if(destroyed)return false;source=null;show();return true;},
    setVisible(value){if(destroyed||typeof value!=='boolean')return false;if(value!==visible){visible=value;show();}return true;},
    destroy(){if(destroyed)return;destroyed=true;source=null;detach();},
  });
}
