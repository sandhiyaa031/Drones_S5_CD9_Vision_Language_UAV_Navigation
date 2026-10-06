import numpy as np, dataclasses
import replay_risk_models as rr
from uav_autonomy.debris_avoidance import AvoidanceParams, simulate_response
from scipy.optimize import least_squares
def collect(run, legs_only, tmax, step=0.2):
    uav=rr.load_uav(run); out=[]
    d=np.diff(uav['sp'],axis=0)
    jumps=np.where(np.linalg.norm(d,axis=1)>0.5)[0]
    tj=uav['ts'][jumps+1]; size=np.linalg.norm(d[jumps][:,:2],axis=1)
    ok=(tj>uav['t'][0]+12)&(tj<uav['t'][-1]-8)
    tj,size=tj[ok],size[ok]
    if legs_only:
        k=np.where(size>2)[0][:1]; tj=tj[k]
    for k,t0 in enumerate(tj):
        tend=min(t0+tmax, tj[k+1] if k+1<len(tj) else 1e9)
        for dt0 in np.arange(0.02,tmax-0.2,step):
            st=rr.state_at(uav,t0+dt0,window=0.06)
            if st: out.append((uav,t0+dt0,st[0],st[1],st[2],st[3],tend))
    return out
hover=collect('results/phase6/calibration_step',False,1.9)
legs=collect('results/phase6/baseline/H_transit',True,1.7)+collect('results/phase6/baseline/I_search',True,1.7)
HS=(0.2,0.4,0.6,0.8,1.0,1.2)
def errs(x,data,use_a,vec=False):
    P=dataclasses.replace(AvoidanceParams(),tau_xy_s=x[0],kp_xy=x[1],kv_xy=x[2],a_xy_max=x[3],horizon_s=1.3)
    out=[]
    for uav,t,p,v,a,sp,tend in data:
        tt,pos,_=simulate_response(p,v,[sp],P,sp,a0=a if use_a else None)
        out.append([np.linalg.norm(pos[0,int(round(h/0.02)),:2]-np.array([np.interp(t+h,uav['t'],uav['p'][:,i]) for i in range(2)])) if t+h<=tend else np.nan for h in HS])
    return np.array(out)
cur=[0.336,0.819,3.373,6.46]
print(len(hover),len(legs))
for use in (False,True):
    print('a0 measured' if use else 'a0 from setpoint','hover',np.nanmax(errs(cur,hover,use),axis=0).round(2),'legs',np.nanmax(errs(cur,legs,use),axis=0).round(2))
res=least_squares(lambda x: np.nan_to_num(np.concatenate([errs(x,hover,True),errs(x,legs,True)])).ravel(),[0.4,0.87,2.9,9.0],bounds=([0.02,0.3,0.5,2],[0.6,3,8,15]),max_nfev=40)
print('refit with measured a0',res.x.round(3),'hover',np.nanmax(errs(res.x,hover,True),axis=0).round(2),'legs',np.nanmax(errs(res.x,legs,True),axis=0).round(2))
