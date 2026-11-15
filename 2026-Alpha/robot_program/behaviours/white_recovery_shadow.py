"""Diagnostic only: inspect same-frame camera and previous vote at white loss."""
import math

def white_recovery_shadow(obs,snapshot,now,center_x=186.0,deadband=6.0,max_age_sec=0.25):
    """Return untrusted image-side candidate; never provide motor commands.

    A majority and a fresh x-coordinate do not establish the physical line,
    camera-to-sensor geometry or safe recovery direction.
    """
    result={'status':'UNKNOWN','reason':'no_frame','actuation':False,'frame':None,'stream_id':None,'session':None,'age_ms':None,'x_px':None,'current_side':None,'vote_side':None,'vote_age_ms':None,'vote_counts':None,'track_reason':None,'excluded_bbox':None,'bottle_insight':None,'image':None}
    if not isinstance(obs,dict):return result
    result.update(frame=obs.get('frame'),stream_id=obs.get('stream_id'),session=obs.get('session'),bottle_insight=obs.get('bottle_insight'),image=obs.get('image'))
    t=obs.get('capture_time');line=obs.get('lap_line') or {};x=line.get('x_px')
    result.update(track_reason=line.get('track_reason'),excluded_bbox=line.get('excluded_bbox'))
    if not isinstance(now,(int,float)) or not math.isfinite(now) or not isinstance(t,(int,float)) or not math.isfinite(t):result['reason']='invalid_time';return result
    age=now-t;result['age_ms']=round(age*1000,1)
    if not 0<=age<=max_age_sec:result['reason']='stale_or_future';return result
    if not isinstance(obs.get('stream_id'),str) or not obs['stream_id'] or not isinstance(obs.get('session'),int) or not isinstance(obs.get('frame'),int):result['reason']='invalid_identity';return result
    if not line.get('valid') or line.get('track_state')!='CANDIDATE' or line.get('image_width')!=320 or not isinstance(x,(int,float)) or not math.isfinite(x) or not 0<=x<320:
        result['reason']='line_unavailable';return result
    result['x_px']=x
    side='RIGHT' if x-center_x>deadband else 'LEFT' if x-center_x < -deadband else 'CENTER'
    result['current_side']=side
    if side=='CENTER':result['reason']='center_ambiguous';return result
    if not isinstance(snapshot,dict):result['reason']='no_majority';return result
    st=snapshot.get('capture_time');result['vote_side']=snapshot.get('side');result['vote_counts']=snapshot.get('vote_counts')
    if not isinstance(st,(int,float)) or not math.isfinite(st):result['reason']='invalid_vote_time';return result
    result['vote_age_ms']=round((now-st)*1000,1)
    if not 0<=now-st<=max_age_sec:result['reason']='stale_vote';return result
    if snapshot.get('stream')!=(obs['stream_id'],obs['session']):result['reason']='different_stream';return result
    if not isinstance(snapshot.get('frame'),int) or snapshot['frame']>obs['frame']:result['reason']='frame_reversed';return result
    counts=snapshot.get('vote_counts');wins=counts.get(side,0) if isinstance(counts,dict) else 0
    if not isinstance(counts,dict) or any(not isinstance(n,int) or n<0 for n in counts.values()):result['reason']='invalid_counts';return result
    if snapshot.get('side')!=side or not isinstance(wins,int) or wins<3 or wins<=sum(counts.values())-wins:
        result['reason']='vote_conflict';return result
    result['status']='IMAGE_'+side;result['reason']='image_candidate_unverified';return result
