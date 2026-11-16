"""Trial-only heading arbitration for capped-PWM line trace."""
import math

def mix_heading_trace(left, right, current, reference, course, power=100, gain=2.0, heading_band=3.0, heading_limit=16.0):
    """Return bounded wheel commands and diagnostics; turn sign follows RunByGyro."""
    if course not in (-1,1) or not all(math.isfinite(x) for x in (left,right,current,reference,power,gain,heading_band,heading_limit)):
        raise ValueError('invalid heading trace input')
    if power!=100 or gain<=0 or heading_band<=0 or not 0<heading_limit<=50:
        raise ValueError('invalid heading trace limits')
    error=(reference-current+180.0)%360.0-180.0
    line=(left-right)/2.0
    correction=max(-heading_limit,min(heading_limit,-course*gain*error))
    differential=max(-50.0,min(50.0,line+correction));mode='BLEND_TRIAL'
    base=100.0-abs(differential)
    return round(base+differential),round(base-differential),error,line,correction,mode
