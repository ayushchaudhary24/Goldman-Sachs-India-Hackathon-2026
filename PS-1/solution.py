#!/usr/bin/env python3
import json,sys,math,heapq,time
from itertools import permutations
BATTERY_CAP=5e2
CHARGE_RATE=2.
NFZ_MARGIN=5.
EPS=1e-09
TIME_EPS=.001
MAX_WAYPOINTS=52
MAX_PERM_SIZE=6
MAX_2OPT_N=60
MAX_2OPT_CHECKS=4000
MAX_TRIP_ITEMS_NO_NFZ=100
MAX_TRIP_ITEMS_WITH_NFZ=24
MAX_ROUTE_WAIT_LOOPS=8
RESCUE_LIMIT_WITH_NFZ=400
RESCUE_LIMIT_NO_NFZ=1200
CHARGE_BUFFER=.1
LARGE_DLV_THRESH=800
LARGE_SEED_COUNT=3
LARGE_POOL_SIZE=40
LARGE_NEARBY=15
LARGE_SKIP_LIMIT=2
LARGE_2OPT_N=15
LARGE_MAX_TRIP_NO_NFZ=20
DENSE_NFZ_MODE=False
VERY_DENSE_NFZ_MODE=False
NFZ_MAX_END=.0
LARGE_MODE=False
_hypot=math.hypot
_SOLVE_START=.0
def dist(a,b):return _hypot(a[0]-b[0],a[1]-b[1])
def energy_cost(d,p):return d*(1.+p)
def rr(x):return round(float(x),6)
def time_left():return 9.-(time.time()-_SOLVE_START)
def normalize_nfzs(raw):
	out=[]
	for n in raw:
		n=dict(n);n['T_start']=float(n.get('T_start',.0));n['T_end']=float(n.get('T_end',.0))
		if n['shape']=='circle':cx,cy=n['center'];cx,cy=float(cx),float(cy);r=float(n['radius']);n['center']=cx,cy;n['radius']=r;n['bbox']=cx-r,cy-r,cx+r,cy+r
		elif n['shape']=='rectangle':(x0,y0),(x1,y1)=n['corners'];n['x_min']=float(min(x0,x1));n['y_min']=float(min(y0,y1));n['x_max']=float(max(x0,x1));n['y_max']=float(max(y0,y1));n['bbox']=n['x_min'],n['y_min'],n['x_max'],n['y_max']
		out.append(n)
	return out
def _circle_param(ax,ay,bx,by,cx,cy,r):
	dx,dy=bx-ax,by-ay;fx,fy=ax-cx,ay-cy;a=dx*dx+dy*dy
	if a<EPS:return(.0,.0)if fx*fx+fy*fy<=r*r+EPS else None
	b=2.*(fx*dx+fy*dy);c=fx*fx+fy*fy-r*r;disc=b*b-4.*a*c
	if disc<-EPS:return
	sq=math.sqrt(max(.0,disc));s1=(-b-sq)/(2.*a);s2=(-b+sq)/(2.*a);s1,s2=max(.0,s1),min(1.,s2);return(s1,s2)if s1<=s2+EPS else None
def _rect_param(ax,ay,bx,by,xmn,ymn,xmx,ymx):
	dx,dy=bx-ax,by-ay;t0,t1=.0,1.
	for(p,q)in((-dx,ax-xmn),(dx,xmx-ax),(-dy,ay-ymn),(dy,ymx-ay)):
		if abs(p)<EPS:
			if q<-EPS:return
		else:
			t=q/p
			if p<0:t0=max(t0,t)
			else:t1=min(t1,t)
	if t0>t1+EPS:return
	t0,t1=max(.0,t0),min(1.,t1);return(t0,t1)if t0<=t1+EPS else None
def _shape_param(ax,ay,bx,by,nfz):
	if nfz['shape']=='circle':cx,cy=nfz['center'];return _circle_param(ax,ay,bx,by,cx,cy,nfz['radius'])
	if nfz['shape']=='rectangle':return _rect_param(ax,ay,bx,by,nfz['x_min'],nfz['y_min'],nfz['x_max'],nfz['y_max'])
def seg_blocked(ax,ay,bx,by,t_depart,nfz):
	param=_shape_param(ax,ay,bx,by,nfz)
	if param is None:return False
	d=_hypot(ax-bx,ay-by);s1,s2=param
	if d<EPS:return nfz['T_start']<=t_depart<nfz['T_end']
	t_enter=t_depart+s1*d;t_leave=t_depart+s2*d;return t_enter<nfz['T_end']-EPS and t_leave>nfz['T_start']+EPS
def any_blocked(ax,ay,bx,by,t_depart,nfzs):
	if not nfzs:return False
	sxmn,sxmx=min(ax,bx),max(ax,bx);symn,symx=min(ay,by),max(ay,by)
	for n in nfzs:
		bx0,by0,bx1,by1=n['bbox']
		if bx1<sxmn or bx0>sxmx or by1<symn or by0>symx:continue
		if seg_blocked(ax,ay,bx,by,t_depart,n):return True
	return False
def gen_waypoints(nfzs):
	wps=[]
	for nfz in nfzs:
		if nfz['shape']=='circle':
			cx,cy=nfz['center']
			for margin in(NFZ_MARGIN,NFZ_MARGIN*1.7):
				r=nfz['radius']+margin
				for i in range(16):a=2.*math.pi*i/16.;wps.append((cx+r*math.cos(a),cy+r*math.sin(a)))
		elif nfz['shape']=='rectangle':
			x0,y0=nfz['x_min'],nfz['y_min'];x1,y1=nfz['x_max'],nfz['y_max']
			for m in(NFZ_MARGIN,NFZ_MARGIN*1.7):wps.extend([(x0-m,y0-m),(x1+m,y0-m),(x1+m,y1+m),(x0-m,y1+m),((x0+x1)*.5,y0-m),(x1+m,(y0+y1)*.5),((x0+x1)*.5,y1+m),(x0-m,(y0+y1)*.5)])
	seen=set();out=[]
	for p in wps:
		k=round(p[0],4),round(p[1],4)
		if k not in seen:seen.add(k);out.append(p)
	return out
def _filtered_nodes(start,end,nfzs,waypoints):
	if not waypoints:return[]
	sx0,sx1=min(start[0],end[0]),max(start[0],end[0]);sy0,sy1=min(start[1],end[1]),max(start[1],end[1]);pad=5e1
	for n in nfzs:
		bx0,by0,bx1,by1=n['bbox']
		if bx1<sx0-pad or bx0>sx1+pad or by1<sy0-pad or by0>sy1+pad:continue
		if n['shape']=='circle':pad=max(pad,n['radius']*2.4+NFZ_MARGIN)
		else:pad=max(pad,max(n['x_max']-n['x_min'],n['y_max']-n['y_min'])*1.4+NFZ_MARGIN)
	xlo,xhi=sx0-pad,sx1+pad;ylo,yhi=sy0-pad,sy1+pad;nodes=[(x,y)for(x,y)in waypoints if xlo<=x<=xhi and ylo<=y<=yhi];cap=18 if DENSE_NFZ_MODE else MAX_WAYPOINTS
	if len(nodes)>cap:
		mx,my=(start[0]+end[0])*.5,(start[1]+end[1])*.5;ax,ay=start;bx,by=end;dx,dy=bx-ax,by-ay;denom=dx*dx+dy*dy
		def ns(p):
			if denom>EPS:s=max(.0,min(1.,((p[0]-ax)*dx+(p[1]-ay)*dy)/denom));px,py=ax+s*dx,ay+s*dy;ld=(p[0]-px)**2+(p[1]-py)**2
			else:ld=.0
			return ld+.25*((p[0]-mx)**2+(p[1]-my)**2)
		nodes.sort(key=ns);nodes=nodes[:cap]
	return nodes
def edge_wait_delay(a,b,t_depart,nfzs):
	if not nfzs:return .0
	ax,ay=a;bx,by=b;t=t_depart;sxmn,sxmx=min(ax,bx),max(ax,bx);symn,symx=min(ay,by),max(ay,by);cands=[]
	for n in nfzs:
		bx0,by0,bx1,by1=n['bbox']
		if bx1<sxmn or bx0>sxmx or by1<symn or by0>symx:continue
		if _shape_param(ax,ay,bx,by,n)is not None:cands.append(n)
	if not cands:return .0
	for _ in range(MAX_ROUTE_WAIT_LOOPS):
		blockers=[n for n in cands if seg_blocked(ax,ay,bx,by,t,n)]
		if not blockers:return max(.0,t-t_depart)
		nt=max(n['T_end']for n in blockers)+TIME_EPS
		if nt<=t+EPS:return
		t=nt
def _dijkstra_route(nodes,end_idx,t0,nfzs):
	n=len(nodes);inf=float('inf');best_t=[inf]*n;best_d=[inf]*n;prev=[None]*n;best_t[0]=t0;best_d[0]=.0;pq=[(t0,.0,0)]
	while pq:
		t_u,d_u,u=heapq.heappop(pq)
		if t_u>best_t[u]+1e-07:continue
		if u==end_idx:break
		ux,uy=nodes[u];ovs=range(n)
		if DENSE_NFZ_MODE and n>20:
			nearest=sorted(((ux-nodes[v][0])**2+(uy-nodes[v][1])**2,v)for v in range(n)if v!=u)[:12];ovs=[v for(_,v)in nearest]
			if end_idx not in ovs:ovs.append(end_idx)
		for v in ovs:
			if v==u:continue
			vx,vy=nodes[v];leg=_hypot(ux-vx,uy-vy)
			if leg<EPS:continue
			wait=edge_wait_delay(nodes[u],nodes[v],t_u,nfzs)
			if wait is None:continue
			nt=t_u+wait+leg;nd=d_u+leg
			if nt<best_t[v]-1e-07 or abs(nt-best_t[v])<=1e-07 and nd<best_d[v]-1e-07:best_t[v]=nt;best_d[v]=nd;prev[v]=u,wait;heapq.heappush(pq,(nt,nd,v))
	if best_t[end_idx]==inf:return
	rn=[];rw=[];cur=end_idx
	while cur!=0:
		rn.append(nodes[cur]);p=prev[cur]
		if p is None:return
		cur,wait=p;rw.append(wait)
	rn.append(nodes[0]);rn.reverse();rw.reverse();return rn,rw,best_d[end_idx],best_t[end_idx]-t0
def find_route(start,end,t0,nfzs,waypoints):
	d0=dist(start,end)
	if d0<EPS:return[start,end],[.0],.0,.0
	if not nfzs or NFZ_MAX_END and t0>=NFZ_MAX_END-EPS:return[start,end],[.0],d0,d0
	best=None;dw=edge_wait_delay(start,end,t0,nfzs)
	if dw is not None:
		best=[start,end],[dw],d0,dw+d0
		if dw<=EPS:return best
		if VERY_DENSE_NFZ_MODE:return best
		if DENSE_NFZ_MODE and dw<=12e1:return best
	fl=_filtered_nodes(start,end,nfzs,waypoints)
	if fl:
		nodes=[start]+fl+[end];routed=_dijkstra_route(nodes,len(nodes)-1,t0,nfzs)
		if routed is not None and(best is None or routed[3]<best[3]-1e-07):best=routed
	return best
def fly_segment(start,end,t_now,bat_now,payload,nfzs,waypoints,final_action):
	route=find_route(start,end,t_now,nfzs,waypoints)
	if route is None:return
	pts,waits,pd,_=route;need=energy_cost(pd,payload)
	if bat_now<need-.01:return
	acts=[];pos=start;t=t_now;bat=bat_now
	for i in range(len(pts)-1):
		wait=waits[i]if i<len(waits)else .0
		if wait>EPS:t+=wait;acts.append({'action':'WAIT','x':rr(pos[0]),'y':rr(pos[1]),'t':rr(t)})
		nxt=pts[i+1];leg=dist(pos,nxt)
		if leg>EPS:bat-=energy_cost(leg,payload);t+=leg
		pos=nxt;act=final_action if i==len(pts)-2 else'WAYPOINT';acts.append({'action':act,'x':rr(pos[0]),'y':rr(pos[1]),'t':rr(t)})
	return acts,t,bat,pos
class ChargeScheduler:
	def __init__(self,stations):self.stations=stations;self.intervals=[[]for _ in stations]
	def snapshot(self):return[lst[:]for lst in self.intervals]
	def restore(self,snap):self.intervals=[lst[:]for lst in snap]
	def earliest(self,idx,arrival,duration):
		if duration<=EPS:return arrival
		slots=max(1,int(self.stations[idx].get('slots',1)));t=arrival;ints=self.intervals[idx]
		for _ in range(len(ints)+3):
			ol=[(s,e)for(s,e)in ints if s<t+duration-EPS and e>t+EPS]
			if len(ol)<slots:return t
			t=min(e for(_,e)in ol if e>t+EPS)+TIME_EPS
		return t
	def reserve(self,idx,arrival,duration):
		start=self.earliest(idx,arrival,duration)
		if duration>EPS:self.intervals[idx].append((start,start+duration))
		return start
def charge_here(actions,pos,t,bat,target_bat,cs_idx,scheduler):
	target_bat=min(BATTERY_CAP,max(bat,target_bat));needed=max(.0,target_bat-bat)
	if needed<=.01:return actions,t,bat
	dur=needed/CHARGE_RATE;start=scheduler.reserve(cs_idx,t,dur)
	if start>t+EPS:actions.append({'action':'WAIT','x':rr(pos[0]),'y':rr(pos[1]),'t':rr(start)})
	actions.append({'action':'CHARGE','x':rr(pos[0]),'y':rr(pos[1]),'t':rr(start)});et=start+dur;actions.append({'action':'CHARGE_COMPLETE','x':rr(pos[0]),'y':rr(pos[1]),'t':rr(et)});return actions,et,min(BATTERY_CAP,bat+needed)
def fly_to_station(pos,cs_pos,t,bat,payload,nfzs,waypoints):
	if dist(pos,cs_pos)<EPS:return[],t,bat,cs_pos
	return fly_segment(pos,cs_pos,t,bat,payload,nfzs,waypoints,'WAYPOINT')
def plan_charge_before_target(pos,t,bat,payload,target,nfzs,waypoints,stations,scheduler):
	best=None;bs=scheduler.snapshot()
	for(idx,cs)in enumerate(stations):
		scheduler.restore(bs);cs_pos=float(cs['x']),float(cs['y']);to_cs=fly_to_station(pos,cs_pos,t,bat,payload,nfzs,waypoints)
		if to_cs is None:continue
		ca,t_cs,bat_cs,_=to_cs;route=find_route(cs_pos,target,t_cs,nfzs,waypoints)
		if route is None:continue
		na=min(BATTERY_CAP,energy_cost(route[2],payload)+CHARGE_BUFFER);dur=max(.0,na-bat_cs)/CHARGE_RATE;st=scheduler.earliest(idx,t_cs,dur);fe=st+dur+route[3]
		if best is None or fe<best[0]:best=fe,idx,cs_pos,ca,t_cs,bat_cs,na
	scheduler.restore(bs)
	if best is None:return
	_,idx,cs_pos,ca,t_cs,bat_cs,na=best;acts=list(ca);acts,ta,ba=charge_here(acts,cs_pos,t_cs,bat_cs,na,idx,scheduler);return acts,ta,ba,cs_pos
def eval_order(wh,order,t_start=.0,include_return=True):
	pos=wh;t=t_start;payload=sum(d['weight']for d in order);energy=.0
	for d in order:
		tgt=d['x'],d['y'];leg=dist(pos,tgt);t+=leg;energy+=energy_cost(leg,payload)
		if t>d['deadline']+.5:return False,energy,t
		payload-=d['weight'];pos=tgt
	if include_return:leg=dist(pos,wh);t+=leg;energy+=energy_cost(leg,payload)
	return True,energy,t
def _greedy_order(wh,dlvs,t_start,kf):
	rem=list(dlvs);order=[];pos=wh;t=t_start
	while rem:nxt=min(rem,key=lambda d:kf(d,pos,t));order.append(nxt);t+=dist(pos,(nxt['x'],nxt['y']));pos=nxt['x'],nxt['y'];rem.remove(nxt)
	return order
def best_order(wh,dlvs,t_start=.0):
	n=len(dlvs)
	if n<=1:return dlvs[:]
	eff2opt=LARGE_2OPT_N if LARGE_MODE else MAX_2OPT_N;eff2optChecks=MAX_2OPT_CHECKS//2 if LARGE_MODE else MAX_2OPT_CHECKS
	if n<=MAX_PERM_SIZE:
		best=None;bo=float('inf')
		for pm in permutations(dlvs):
			o=list(pm);ok,e,t=eval_order(wh,o,t_start)
			if not ok:continue
			obj=e+.08*(t-t_start)
			if obj<bo:bo=obj;best=o
		if best is not None:return best
	def k1(d,pos,t):leg=dist(pos,(d['x'],d['y']));arr=t+leg;late=max(.0,arr-d['deadline']);slack=max(.0,d['deadline']-arr);return late*1e4+leg+.012*slack-1e1*d['weight'],d['deadline']
	def k2(d,pos,t):leg=dist(pos,(d['x'],d['y']));arr=t+leg;late=max(.0,arr-d['deadline']);return late*1e4-25.*d['weight']+leg,d['deadline']
	def k3(d,pos,t):leg=dist(pos,(d['x'],d['y']));arr=t+leg;late=max(.0,arr-d['deadline']);return late*1e4+leg*.6-8.*d['weight'],leg
	def k4(d,pos,t):leg=dist(pos,(d['x'],d['y']));arr=t+leg;slack=d['deadline']-arr;late=max(.0,-slack);return late*1e4+max(.0,slack)*.01+leg-5.*d['weight'],d['deadline']
	def k5(d,pos,t):leg=dist(pos,(d['x'],d['y']));arr=t+leg;late=max(.0,arr-d['deadline']);ew=leg*d['weight'];return late*1e4+leg-ew*.3,d['deadline']
	def k6(d,pos,t):leg=dist(pos,(d['x'],d['y']));arr=t+leg;slack=d['deadline']-arr;return-max(.0,slack)+leg*2.,d['deadline']
	cands=[];greedy_kfs=(k1,k2,k3)if LARGE_MODE else(k1,k2,k3,k4,k5,k6)
	for kf in greedy_kfs:
		c=_greedy_order(wh,dlvs,t_start,kf);ok,e,te=eval_order(wh,c,t_start)
		if ok:cands.append((e+.08*(te-t_start),c))
	if LARGE_MODE:sort_keys=[lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))),lambda d:(d['deadline']-dist(wh,(d['x'],d['y']))*.5,d['deadline'])]
	else:sort_keys=[lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))),lambda d:(-d['weight'],d['deadline']),lambda d:(d['deadline']-dist(wh,(d['x'],d['y']))*.5,d['deadline']),lambda d:(d['deadline'],d['weight']),lambda d:(dist(wh,(d['x'],d['y'])),d['deadline'])]
	for sk in sort_keys:
		c=sorted(dlvs,key=sk);ok,e,te=eval_order(wh,c,t_start)
		if ok:cands.append((e+.08*(te-t_start),c))
	if not cands:return sorted(dlvs,key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))))
	cands.sort(key=lambda x:x[0]);best=list(cands[0][1])
	if n<=eff2opt:
		ok,be,bt=eval_order(wh,best,t_start);bo=be+.08*(bt-t_start)if ok else float('inf');checks=0;imp=True
		while imp and checks<eff2optChecks:
			imp=False
			for i in range(n-1):
				for j in range(i+1,n):
					checks+=1;c=best[:i]+best[i:j+1][::-1]+best[j+1:];ok,e,tt=eval_order(wh,c,t_start);obj=e+.08*(tt-t_start)if ok else float('inf')
					if obj<bo-1e-07:best=c;bo=obj;imp=True;break
				if imp or checks>=eff2optChecks:break
		if not LARGE_MODE and n>=4 and n<=MAX_2OPT_N:
			imp2=True;oc=0
			while imp2 and oc<MAX_2OPT_CHECKS//2:
				imp2=False
				for i in range(n):
					item=best[i];rest=best[:i]+best[i+1:]
					for j in range(len(rest)+1):
						oc+=1
						if oc>MAX_2OPT_CHECKS//2:break
						if j==i:continue
						c=rest[:j]+[item]+rest[j:];ok,e,tt=eval_order(wh,c,t_start);obj=e+.08*(tt-t_start)if ok else float('inf')
						if obj<bo-1e-07:best=c;bo=obj;imp2=True;break
					if imp2 or oc>MAX_2OPT_CHECKS//2:break
	return best
def estimate_trip_energy(dlvs,wh):
	if not dlvs:return .0
	o=best_order(wh,dlvs,.0)if len(dlvs)<=14 else dlvs;ok,e,_=eval_order(wh,o,.0);return e
def assign_deliveries(drones,dlvs,wh):
	sd=sorted(dlvs,key=lambda d:d['deadline']);asgn={d['id']:[]for d in drones};loads={d['id']:.0 for d in drones}
	for dlv in sd:
		best=None;ml=float('inf')
		for dr in drones:
			if dlv['weight']>dr['max_payload']+EPS:continue
			if loads[dr['id']]<ml:ml=loads[dr['id']];best=dr
		if best:asgn[best['id']].append(dlv);loads[best['id']]+=2.*dist(wh,(dlv['x'],dlv['y']))
	return asgn
def assign_spatial(drones,dlvs,wh):
	asgn={d['id']:[]for d in drones};loads={d['id']:.0 for d in drones};cap_d=sorted(drones,key=lambda d:(-d['max_payload'],d['id']))
	if not cap_d:return asgn
	ord_d=sorted(dlvs,key=lambda d:(math.atan2(d['y']-wh[1],d['x']-wh[0]),dist(wh,(d['x'],d['y']))));n=len(cap_d)
	for(idx,dlv)in enumerate(ord_d):
		pref=cap_d[idx*n//max(1,len(ord_d))];ch=[dr for dr in cap_d if dlv['weight']<=dr['max_payload']+EPS]
		if not ch:continue
		if dlv['weight']<=pref['max_payload']+EPS:b=min(ch,key=lambda dr:(0 if dr['id']==pref['id']else 1,loads[dr['id']]))
		else:b=min(ch,key=lambda dr:loads[dr['id']])
		asgn[b['id']].append(dlv);loads[b['id']]+=2.*dist(wh,(dlv['x'],dlv['y']))
	return asgn
def assign_dd(drones,dlvs,wh):
	asgn={d['id']:[]for d in drones};loads={d['id']:.0 for d in drones};sd=sorted(dlvs,key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))))
	for dlv in sd:
		ch=[dr for dr in drones if dlv['weight']<=dr['max_payload']+EPS]
		if not ch:continue
		dwh=dist(wh,(dlv['x'],dlv['y']));b=min(ch,key=lambda dr:loads[dr['id']]+.35*dwh/max(.1,dr['max_payload']));asgn[b['id']].append(dlv);loads[b['id']]+=2.*dwh
	return asgn
def assign_closest(drones,dlvs,wh):
	asgn={d['id']:[]for d in drones};loads={d['id']:.0 for d in drones};sd=sorted(dlvs,key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))))
	for dlv in sd:
		ch=[dr for dr in drones if dlv['weight']<=dr['max_payload']+EPS]
		if not ch:continue
		dwh=dist(wh,(dlv['x'],dlv['y']));b=min(ch,key=lambda dr:loads[dr['id']]/max(.1,dr['max_payload'])+dwh/max(.1,dr['max_payload'])*.2);asgn[b['id']].append(dlv);loads[b['id']]+=2.*dwh
	return asgn
def group_trips(dlvs,mp,wh,hn,hc,mode='deadline'):
	if not dlvs:return[]
	if LARGE_MODE:mi=LARGE_MAX_TRIP_NO_NFZ if not hn else MAX_TRIP_ITEMS_WITH_NFZ
	else:mi=MAX_TRIP_ITEMS_WITH_NFZ if hn else MAX_TRIP_ITEMS_NO_NFZ
	if mode=='spatial':o=sorted(dlvs,key=lambda d:(math.atan2(d['y']-wh[1],d['x']-wh[0]),d['deadline']))
	elif mode=='distance':o=sorted(dlvs,key=lambda d:(dist(wh,(d['x'],d['y'])),d['deadline']))
	elif mode=='urgent':mi=min(mi,10 if hn else 16);o=sorted(dlvs,key=lambda d:(d['deadline']-dist(wh,(d['x'],d['y'])),d['deadline'],dist(wh,(d['x'],d['y']))))
	elif mode=='weight_first':o=sorted(dlvs,key=lambda d:(-d['weight'],d['deadline'],dist(wh,(d['x'],d['y']))))
	else:o=sorted(dlvs,key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))))
	trips=[];cur=[];cw=.0
	for d in o:
		w=cur+[d];tm=len(w)>mi;th=cw+d['weight']>mp+EPS;te=False
		if not hc and len(w)>1:
			e_est=estimate_trip_energy(w,wh)
			if hn:e_est*=1.3
			te=e_est>BATTERY_CAP*.95
		elif hc and len(w)>1 and hn:e_est=estimate_trip_energy(w,wh)*1.3;te=e_est>BATTERY_CAP*2.5
		if cur and(tm or th or te):trips.append(cur);cur=[d];cw=d['weight']
		else:cur=w;cw+=d['weight']
	if cur:trips.append(cur)
	return trips
def split_trip(trip,wh,hc,hn):
	mi=MAX_TRIP_ITEMS_WITH_NFZ if hn else MAX_TRIP_ITEMS_NO_NFZ
	if len(trip)<=1:return[trip]
	if len(trip)<=mi and(hc or estimate_trip_energy(trip,wh)<=BATTERY_CAP*.99):return[trip]
	mid=len(trip)//2;return split_trip(trip[:mid],wh,hc,hn)+split_trip(trip[mid:],wh,hc,hn)
def plan_return(pos,t,bat,payload,wh,nfzs,wps,stations,scheduler):
	direct=fly_segment(pos,wh,t,bat,payload,nfzs,wps,'RETURN')
	if direct is not None:return direct[0]
	best=None;be=float('inf');bs=None;ss=scheduler.snapshot()
	for(idx,cs)in enumerate(stations):
		scheduler.restore(ss);cp=float(cs['x']),float(cs['y']);tc=fly_to_station(pos,cp,t,bat,payload,nfzs,wps)
		if tc is None:continue
		ca,tcs,bcs,_=tc;route=find_route(cp,wh,tcs,nfzs,wps)
		if route is None:continue
		need=min(BATTERY_CAP,energy_cost(route[2],payload)+CHARGE_BUFFER);acts=list(ca);acts,ta,ba=charge_here(acts,cp,tcs,bcs,need,idx,scheduler);tw=fly_segment(cp,wh,ta,ba,payload,nfzs,wps,'RETURN')
		if tw is None or tw[2]<-.01:
			scheduler.restore(ss);acts=list(ca);acts,ta,ba=charge_here(acts,cp,tcs,bcs,BATTERY_CAP,idx,scheduler);tw=fly_segment(cp,wh,ta,ba,payload,nfzs,wps,'RETURN')
			if tw is None or tw[2]<-.01:continue
		full=acts+tw[0]
		if tw[1]<be:be=tw[1];best=full;bs=scheduler.snapshot()
	if best:scheduler.restore(bs)
	else:scheduler.restore(ss)
	return best
def build_trip_partial(wh,dlvs,t_start,nfzs,wps,stations,scheduler):
	if not dlvs:return
	acts=[{'action':'PICKUP','x':rr(wh[0]),'y':rr(wh[1]),'t':rr(t_start),'delivery_ids':[d['id']for d in dlvs]}];pos=wh;t=t_start;bat=BATTERY_CAP;payload=sum(d['weight']for d in dlvs);delivered_in_trip=[];skipped_ids=set()
	for d in dlvs:
		tgt=d['x'],d['y']
		if t+dist(pos,tgt)>d['deadline']+1.:payload-=d['weight'];skipped_ids.add(d['id']);continue
		r=fly_segment(pos,tgt,t,bat,payload,nfzs,wps,'DELIVER')
		if r is None and stations:
			cr=plan_charge_before_target(pos,t,bat,payload,tgt,nfzs,wps,stations,scheduler)
			if cr:ca,t,bat,pos=cr;acts.extend(ca);r=fly_segment(pos,tgt,t,bat,payload,nfzs,wps,'DELIVER')
		if r is None:payload-=d['weight'];skipped_ids.add(d['id']);continue
		sa,t,bat,pos=r
		if t>d['deadline']+.001:
			payload-=d['weight'];skipped_ids.add(d['id'])
			if sa:sa[-1]['x']=rr(d['x']);sa[-1]['y']=rr(d['y'])
			acts.extend(sa);payload-=d['weight'];skipped_ids.add(d['id']);continue
		if sa:sa[-1]['x']=rr(d['x']);sa[-1]['y']=rr(d['y']);sa[-1]['delivery_id']=d['id']
		acts.extend(sa);delivered_in_trip.append(d);payload-=d['weight'];continue
def build_trip(wh,dlvs,t_start,nfzs,wps,stations,scheduler):
	if not dlvs:return
	acts=[{'action':'PICKUP','x':rr(wh[0]),'y':rr(wh[1]),'t':rr(t_start),'delivery_ids':[d['id']for d in dlvs]}];pos=wh;t=t_start;bat=BATTERY_CAP;payload=sum(d['weight']for d in dlvs)
	for d in dlvs:
		tgt=d['x'],d['y'];r=fly_segment(pos,tgt,t,bat,payload,nfzs,wps,'DELIVER')
		if r is None and stations:
			cr=plan_charge_before_target(pos,t,bat,payload,tgt,nfzs,wps,stations,scheduler)
			if cr:ca,t,bat,pos=cr;acts.extend(ca);r=fly_segment(pos,tgt,t,bat,payload,nfzs,wps,'DELIVER')
		if r is None:return
		sa,t,bat,pos=r
		if t>d['deadline']+.001:return
		if sa:sa[-1]['x']=rr(d['x']);sa[-1]['y']=rr(d['y']);sa[-1]['delivery_id']=d['id']
		acts.extend(sa);payload-=d['weight']
		if bat<-.01:return
	ret=plan_return(pos,t,bat,payload,wh,nfzs,wps,stations,scheduler)
	if ret is None:return
	acts.extend(ret);return acts
def build_trip_skip(wh,dlvs,t_start,nfzs,wps,stations,scheduler,skip_id=None):
	if not dlvs:return
	filtered=[d for d in dlvs if d['id']!=skip_id]
	if not filtered:return
	acts=[{'action':'PICKUP','x':rr(wh[0]),'y':rr(wh[1]),'t':rr(t_start),'delivery_ids':[d['id']for d in filtered]}];pos=wh;t=t_start;bat=BATTERY_CAP;payload=sum(d['weight']for d in filtered);delivered_ids=[]
	for d in filtered:
		tgt=d['x'],d['y'];earliest_arrival=t+dist(pos,tgt)
		if earliest_arrival>d['deadline']+5.:return
		r=fly_segment(pos,tgt,t,bat,payload,nfzs,wps,'DELIVER')
		if r is None and stations:
			cr=plan_charge_before_target(pos,t,bat,payload,tgt,nfzs,wps,stations,scheduler)
			if cr:ca,t,bat,pos=cr;acts.extend(ca);r=fly_segment(pos,tgt,t,bat,payload,nfzs,wps,'DELIVER')
		if r is None:return
		sa,t,bat,pos=r
		if t>d['deadline']+.001:return
		if sa:sa[-1]['x']=rr(d['x']);sa[-1]['y']=rr(d['y']);sa[-1]['delivery_id']=d['id']
		acts.extend(sa);payload-=d['weight'];delivered_ids.append(d['id'])
		if bat<-.01:return
	ret=plan_return(pos,t,bat,payload,wh,nfzs,wps,stations,scheduler)
	if ret is None:return
	acts.extend(ret);return acts,delivered_ids
def validate_path(path,dmap,nfzs,wh,mp):
	if not path:return True
	if path[0].get('action')!='PICKUP'or path[-1].get('action')!='RETURN':return False
	bat=BATTERY_CAP;carried={};payload=.0;delivered=set()
	for(i,step)in enumerate(path):
		act=step.get('action');x,y,t=float(step['x']),float(step['y']),float(step['t'])
		if i>0:
			prev=path[i-1];px,py,pt=float(prev['x']),float(prev['y']),float(prev['t'])
			if t<pt-.001:return False
			leg=dist((px,py),(x,y))
			if leg>.001:
				if abs(t-pt-leg)>.08:return False
				if any_blocked(px,py,x,y,pt,nfzs):return False
				bat-=energy_cost(leg,payload)
				if bat<-.5:return False
		if act=='PICKUP':
			if dist((x,y),wh)>.01:return False
			bat=BATTERY_CAP;carried={}
			for did in step.get('delivery_ids',[]):
				if did in dmap and did not in delivered:carried[did]=dmap[did]['weight']
			payload=sum(carried.values())
			if payload>mp+.01:return False
		elif act=='DELIVER':
			did=step.get('delivery_id');d=dmap.get(did)
			if d is None or did not in carried or did in delivered:return False
			if abs(x-d['x'])>.01 or abs(y-d['y'])>.01:return False
			if t>d['deadline']+.01:return False
			payload-=carried[did];delivered.add(did);del carried[did]
		elif act=='CHARGE_COMPLETE':
			if i>0 and path[i-1].get('action')=='CHARGE':dt=t-float(path[i-1]['t']);bat=min(BATTERY_CAP,bat+dt*CHARGE_RATE)
		elif act=='RETURN':
			if dist((x,y),wh)<=.01:bat=BATTERY_CAP
	return True
def score_manifest(manifest,dmap):
	delivered=set();te=.0;ms=.0
	for flight in manifest:
		payload=.0;carried={};prev=None
		for step in flight.get('path',[]):
			ms=max(ms,float(step['t']))
			if prev is not None:
				leg=dist((prev['x'],prev['y']),(step['x'],step['y']))
				if leg>EPS:te+=energy_cost(leg,payload)
			act=step.get('action')
			if act=='PICKUP':
				carried={}
				for did in step.get('delivery_ids',[]):
					if did in dmap and did not in delivered:carried[did]=dmap[did]['weight']
				payload=sum(carried.values())
			elif act=='DELIVER':
				did=step.get('delivery_id')
				if did in carried:delivered.add(did);payload-=carried[did];del carried[did]
			prev=step
	return len(delivered)*1e2-te*.1-ms*.05
def state_after_prefix(path,end_idx,dmap):
	bat=BATTERY_CAP;payload=.0;carried={};prev=None
	for i in range(end_idx+1):
		step=path[i]
		if prev is not None:
			leg=dist((prev['x'],prev['y']),(step['x'],step['y']))
			if leg>EPS:bat-=energy_cost(leg,payload)
		act=step.get('action')
		if act=='PICKUP':
			bat=BATTERY_CAP;carried={}
			for did in step.get('delivery_ids',[]):
				if did in dmap:carried[did]=dmap[did]['weight']
			payload=sum(carried.values())
		elif act=='DELIVER':
			did=step.get('delivery_id')
			if did in carried:payload-=carried[did];del carried[did]
		elif act=='CHARGE_COMPLETE'and prev is not None and prev.get('action')=='CHARGE':bat=min(BATTERY_CAP,bat+(float(step['t'])-float(prev['t']))*CHARGE_RATE)
		elif act=='RETURN'and dist((step['x'],step['y']),(path[0]['x'],path[0]['y']))<=.01:bat=BATTERY_CAP
		prev=step
	return bat,payload
def improve_final_returns(manifest,dmap,drid,nfzs,wps,wh,stations):
	if not stations or not manifest:return manifest
	bs=score_manifest(manifest,dmap);sp=[(float(s['x']),float(s['y']))for s in stations];imp=[];changed=False
	for flight in manifest:
		dr=drid[flight['drone_id']];path=flight['path'];ldi=-1
		for(i,step)in enumerate(path):
			if step.get('action')=='DELIVER':ldi=i
		if ldi<0 or ldi==len(path)-1:imp.append(flight);continue
		prefix=path[:ldi+1];last=prefix[-1];pos=float(last['x']),float(last['y']);t=float(last['t']);bat,payload=state_after_prefix(path,ldi,dmap)
		if abs(payload)>.01:imp.append(flight);continue
		bp=path;bfs=score_manifest([flight],dmap)
		for target in[wh]+sp:
			r=fly_segment(pos,target,t,bat,.0,nfzs,wps,'RETURN')
			if r is None:continue
			ra,_,be,_=r
			if be<-.01:continue
			cp=prefix+ra
			if not validate_path(cp,dmap,nfzs,wh,dr['max_payload']):continue
			cf={'drone_id':flight['drone_id'],'path':cp};sc=score_manifest([cf],dmap)
			if sc>bfs+1e-07:bfs=sc;bp=cp
		if bp is not path:changed=True
		imp.append({'drone_id':flight['drone_id'],'path':bp})
	if changed and score_manifest(imp,dmap)>bs+1e-07:return imp
	return manifest
def try_append_trip(state,trip,wh,nfzs,wps,stations,scheduler,dmap):
	dr=state['drone']
	if sum(d['weight']for d in trip)>dr['max_payload']+EPS:return False
	ordered=best_order(wh,trip,state['t']);attempts=[ordered]
	if len(ordered)>1:
		for k in range(len(ordered)-1,0,-1):attempts.append(ordered[:k])
	if len(ordered)>2:
		skip_tried=set();skip_limit=LARGE_SKIP_LIMIT if LARGE_MODE else 8
		for skip_idx in range(min(len(ordered),skip_limit)):
			skip_id=ordered[skip_idx]['id']
			if skip_id in skip_tried:continue
			skip_tried.add(skip_id);subset=[d for d in ordered if d['id']!=skip_id]
			if len(subset)>=1:attempts.append(subset)
	for attempt in attempts:
		snap=scheduler.snapshot();acts=build_trip(wh,attempt,state['t'],nfzs,wps,stations,scheduler)
		if acts is not None and validate_path(acts,dmap,nfzs,wh,dr['max_payload']):state['path'].extend(acts);state['t']=float(acts[-1]['t']);state['delivered'].update(d['id']for d in attempt);return True
		scheduler.restore(snap)
	return False
def choose_dynamic_trip(remaining,drone,wh,t_now,hn,hc,mode):
	cap=drone['max_payload'];cands=[d for d in remaining if d['weight']<=cap+EPS]
	if not cands:return[]
	mi=MAX_TRIP_ITEMS_WITH_NFZ if hn else min(MAX_TRIP_ITEMS_NO_NFZ,80)
	if LARGE_MODE:mi=min(mi,LARGE_MAX_TRIP_NO_NFZ);pool_size=min(len(cands),LARGE_POOL_SIZE)
	else:pool_size=min(len(cands),200)
	pool=cands[:pool_size];seeds=[];s1=min(pool,key=lambda d:(max(.0,t_now+dist(wh,(d['x'],d['y']))-d['deadline']),d['deadline']));seeds.append(('urgent',s1));s2=min(pool,key=lambda d:(dist(wh,(d['x'],d['y'])),d['deadline']));seeds.append(('closest',s2))
	if not LARGE_MODE:s3=max(pool,key=lambda d:(d['weight'],-dist(wh,(d['x'],d['y']))));seeds.append(('heavy',s3));s4=min(pool,key=lambda d:d['deadline']);seeds.append(('earliest',s4));s5=min(pool,key=lambda d:d['deadline']-dist(wh,(d['x'],d['y']))*.5);seeds.append(('ratio',s5));s6=min(pool,key=lambda d:max(.0,d['deadline']-t_now-dist(wh,(d['x'],d['y'])))+dist(wh,(d['x'],d['y']))*.3);seeds.append(('slack',s6));s7=min(pool,key=lambda d:dist(wh,(d['x'],d['y']))*(1+d['weight']));seeds.append(('efficient',s7))
	else:s3=min(pool,key=lambda d:d['deadline']-dist(wh,(d['x'],d['y']))*.5);seeds.append(('ratio',s3))
	bt=None;bfs=-1;seen_seeds=set()
	for(name,seed)in seeds:
		if seed['id']in seen_seeds:continue
		seen_seeds.add(seed['id']);trip=[seed];used={seed['id']};tw=seed['weight'];sx,sy=seed['x'],seed['y'];ins_modes=('proximity',)if LARGE_MODE else('proximity','deadline')
		for ins_mode in ins_modes:
			t_trip=list(trip);u_trip=set(used);w_trip=tw;cx,cy=sx,sy;nearby_limit=LARGE_NEARBY if LARGE_MODE else min(len(pool),80)
			if ins_mode=='proximity':nearby=sorted([d for d in pool if d['id']not in u_trip],key=lambda d:(dist((cx,cy),(d['x'],d['y'])),d['deadline']))[:nearby_limit]
			else:nearby=sorted([d for d in pool if d['id']not in u_trip],key=lambda d:(d['deadline'],dist((cx,cy),(d['x'],d['y']))))[:nearby_limit]
			for cand in nearby:
				if cand['id']in u_trip or len(t_trip)>=mi or w_trip+cand['weight']>cap+EPS:continue
				trial=t_trip+[cand];ok,e,_=eval_order(wh,trial,t_now)
				if not ok:continue
				if not hc and e>BATTERY_CAP*.99:continue
				t_trip.append(cand);u_trip.add(cand['id']);w_trip+=cand['weight'];cx,cy=cand['x'],cand['y']
			ordered=best_order(wh,t_trip,t_now);ok,e,te=eval_order(wh,ordered,t_now)
			if ok:
				sc=len(ordered)*1000-e*.1-te*.02
				if sc>bfs:bfs=sc;bt=ordered
	return bt if bt else[s1]
def solve_fast_large(data):
	global DENSE_NFZ_MODE,VERY_DENSE_NFZ_MODE,NFZ_MAX_END,LARGE_MODE,_SOLVE_START;ms=data['map_size'];wh=float(ms[0])/2.,float(ms[1])/2.;drones=[{'id':d['id'],'max_payload':float(d['max_payload'])}for d in data.get('drones',[])];dlvs=[{'id':d['id'],'x':float(d['x']),'y':float(d['y']),'weight':float(d['weight']),'deadline':float(d['deadline'])}for d in data.get('deliveries',[])];stations=[{'x':float(s['x']),'y':float(s['y']),'slots':int(s.get('slots',1))}for s in data.get('charging_stations',[])];nfzs=normalize_nfzs(data.get('no_fly_zones',[]));NFZ_MAX_END=max((n['T_end']for n in nfzs),default=.0);nd=len(dlvs);DENSE_NFZ_MODE=len(nfzs)>=70 or len(nfzs)*max(1,nd)>=60000;VERY_DENSE_NFZ_MODE=len(nfzs)>=350 or len(nfzs)*max(1,nd)>=450000;LARGE_MODE=True;wps=gen_waypoints(nfzs)
	if not drones:return{'flight_manifest':[]}
	mc=max(d['max_payload']for d in drones)
	if not stations:dlvs=[d for d in dlvs if d['weight']<=mc+EPS and dist(wh,(d['x'],d['y']))*(2.+d['weight'])<=BATTERY_CAP+.5]
	else:dlvs=[d for d in dlvs if d['weight']<=mc+EPS]
	feasible=[]
	for d in dlvs:
		dwh=dist(wh,(d['x'],d['y']));can_carry=any(d['weight']<=dr['max_payload']+EPS for dr in drones)
		if not can_carry:continue
		detour=1.5 if nfzs else 1.
		if dwh*detour>d['deadline']+1.:continue
		e_round=energy_cost(dwh*detour,d['weight'])+energy_cost(dwh*detour,0)
		if e_round>BATTERY_CAP*2 and not stations:continue
		feasible.append(d)
	dlvs=feasible;dmap={d['id']:d for d in dlvs};dlvs_sorted=sorted(dlvs,key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))));states=[{'drone':d,'t':.0,'path':[],'delivered':set()}for d in drones];scheduler=ChargeScheduler(stations);hn=bool(nfzs);hc=bool(stations);delivered_set=set()
	for d in dlvs_sorted:
		if time_left()<1.:break
		if d['id']in delivered_set:continue
		capable=[(i,s)for(i,s)in enumerate(states)if d['weight']<=s['drone']['max_payload']+EPS]
		if not capable:continue
		capable.sort(key=lambda x:x[1]['t'])
		for(_,state)in capable:
			if time_left()<.5:break
			snap=scheduler.snapshot();ordered=best_order(wh,[d],state['t']);acts=build_trip(wh,ordered,state['t'],nfzs,wps,stations,scheduler)
			if acts is not None and validate_path(acts,dmap,nfzs,wh,state['drone']['max_payload']):state['path'].extend(acts);state['t']=float(acts[-1]['t']);state['delivered'].add(d['id']);delivered_set.add(d['id']);break
			scheduler.restore(snap)
	remaining=[d for d in dlvs_sorted if d['id']not in delivered_set]
	if len(remaining)<=200 and time_left()>.5:
		for i in range(min(len(remaining),100)):
			if time_left()<.3:break
			d1=remaining[i]
			if d1['id']in delivered_set:continue
			for j in range(i+1,min(i+8,len(remaining))):
				d2=remaining[j]
				if d2['id']in delivered_set:continue
				cw=d1['weight']+d2['weight']
				if cw>mc+EPS:continue
				for state in sorted(states,key=lambda s:s['t']):
					if cw>state['drone']['max_payload']+EPS:continue
					snap=scheduler.snapshot();ordered=best_order(wh,[d1,d2],state['t']);acts=build_trip(wh,ordered,state['t'],nfzs,wps,stations,scheduler)
					if acts is not None and validate_path(acts,dmap,nfzs,wh,state['drone']['max_payload']):
						state['path'].extend(acts);state['t']=float(acts[-1]['t'])
						for dd in[d1,d2]:state['delivered'].add(dd['id']);delivered_set.add(dd['id'])
						break
					scheduler.restore(snap)
					if d1['id']in delivered_set:break
	remaining2=[d for d in dlvs_sorted if d['id']not in delivered_set]
	for d in remaining2[:200]:
		if time_left()<.3:break
		if d['id']in delivered_set:continue
		for state in sorted(states,key=lambda s:s['t']):
			if d['weight']>state['drone']['max_payload']+EPS:continue
			snap=scheduler.snapshot();acts=build_trip(wh,[d],state['t'],nfzs,wps,stations,scheduler)
			if acts is not None and validate_path(acts,dmap,nfzs,wh,state['drone']['max_payload']):state['path'].extend(acts);state['t']=float(acts[-1]['t']);state['delivered'].add(d['id']);delivered_set.add(d['id']);break
			scheduler.restore(snap)
	manifest=[];gs=set()
	for state in states:
		path=state['path']
		if not path:continue
		drone=state['drone']
		if not validate_path(path,dmap,nfzs,wh,drone['max_payload']):continue
		local=[];bad=False
		for step in path:
			if step.get('action')=='DELIVER':
				did=step.get('delivery_id')
				if did in gs:bad=True;break
				local.append(did)
		if bad:continue
		gs.update(local);manifest.append({'drone_id':drone['id'],'path':path})
	return{'flight_manifest':manifest}
def solve_dynamic(data,strategy):
	global DENSE_NFZ_MODE,VERY_DENSE_NFZ_MODE,NFZ_MAX_END,LARGE_MODE,_SOLVE_START;ms=data['map_size'];wh=float(ms[0])/2.,float(ms[1])/2.;drones=[{'id':d['id'],'max_payload':float(d['max_payload'])}for d in data.get('drones',[])];dlvs=[{'id':d['id'],'x':float(d['x']),'y':float(d['y']),'weight':float(d['weight']),'deadline':float(d['deadline'])}for d in data.get('deliveries',[])];stations=[{'x':float(s['x']),'y':float(s['y']),'slots':int(s.get('slots',1))}for s in data.get('charging_stations',[])];nfzs=normalize_nfzs(data.get('no_fly_zones',[]));NFZ_MAX_END=max((n['T_end']for n in nfzs),default=.0);nd=len(dlvs);DENSE_NFZ_MODE=len(nfzs)>=70 or len(nfzs)*max(1,nd)>=60000;VERY_DENSE_NFZ_MODE=len(nfzs)>=350 or len(nfzs)*max(1,nd)>=450000;LARGE_MODE=nd>=LARGE_DLV_THRESH;wps=gen_waypoints(nfzs)
	if not drones:return{'flight_manifest':[]}
	mc=max(d['max_payload']for d in drones)
	if not stations:dlvs=[d for d in dlvs if d['weight']<=mc+EPS and dist(wh,(d['x'],d['y']))*(2.+d['weight'])<=BATTERY_CAP+.5]
	else:dlvs=[d for d in dlvs if d['weight']<=mc+EPS]
	feasible=[]
	for d in dlvs:
		dwh=dist(wh,(d['x'],d['y']));can_carry=any(d['weight']<=dr['max_payload']+EPS for dr in drones)
		if not can_carry:continue
		detour=1.5 if nfzs else 1.
		if dwh*detour>d['deadline']+1.:continue
		e_round=energy_cost(dwh*detour,d['weight'])+energy_cost(dwh*detour,0)
		if e_round>BATTERY_CAP*2 and not stations:continue
		feasible.append(d)
	dlvs=feasible;dmap={d['id']:d for d in dlvs};states=[{'drone':d,'t':.0,'path':[],'delivered':set()}for d in drones];scheduler=ChargeScheduler(stations);rem=sorted(dlvs,key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))));fc={};heap=[(.0,idx)for idx in range(len(states))];heapq.heapify(heap)
	if LARGE_MODE:mi=max(200,min(2000,nd//3+len(drones)*8))
	else:mi=max(300,len(rem)*4+len(drones)*15)
	iters=0;hn=bool(nfzs);hc=bool(stations);delivered_global=set()
	while rem and heap and iters<mi:
		if time_left()<1.5:break
		iters+=1;_,idx=heapq.heappop(heap);state=states[idx];capable=[d for d in rem if d['weight']<=state['drone']['max_payload']+EPS]
		if not capable:continue
		before=len(state['delivered']);trip=choose_dynamic_trip(capable,state['drone'],wh,state['t'],hn,hc,strategy)
		if trip:try_append_trip(state,trip,wh,nfzs,wps,stations,scheduler,dmap)
		if len(state['delivered'])==before:
			single_limit=10 if LARGE_MODE else 40
			for single in capable[:min(single_limit,len(capable))]:
				if try_append_trip(state,[single],wh,nfzs,wps,stations,scheduler,dmap):break
		if len(state['delivered'])>before:delivered_global.update(state['delivered']);rem=[d for d in rem if d['id']not in delivered_global];heapq.heappush(heap,(state['t'],idx))
		else:
			sid=capable[0]['id']if capable else None
			if sid:
				fc[sid]=fc.get(sid,0)+1
				if fc[sid]>=len(drones)*2:delivered_global.add(sid);rem=[d for d in rem if d['id']!=sid]
			heapq.heappush(heap,(state['t']+1.,idx))
	rescue_passes=2 if LARGE_MODE else 5
	for _ in range(rescue_passes):
		if not rem:break
		if time_left()<1.:break
		sr=[]
		for d in rem:
			rescued=False
			for st in sorted(states,key=lambda s:s['t']):
				if d['weight']>st['drone']['max_payload']+EPS:continue
				if try_append_trip(st,[d],wh,nfzs,wps,stations,scheduler,dmap):rescued=True;break
			if not rescued:sr.append(d)
		rem=sr
		if not rem:break
		pair_limit=50 if LARGE_MODE else 200;pair_inner=3 if LARGE_MODE else 6
		if len(rem)<=pair_limit:
			pair_tried=0;pair_max=100 if LARGE_MODE else 300
			for i in range(min(len(rem),pair_limit)):
				if pair_tried>pair_max or time_left()<.8:break
				for j in range(i+1,min(i+pair_inner,len(rem))):
					pair_tried+=1;d1,d2=rem[i],rem[j]
					if d1['weight']+d2['weight']>max(s['drone']['max_payload']for s in states)+EPS:continue
					for st in sorted(states,key=lambda s:s['t']):
						if d1['weight']+d2['weight']>st['drone']['max_payload']+EPS:continue
						if try_append_trip(st,[d1,d2],wh,nfzs,wps,stations,scheduler,dmap):break
	manifest=[];gs=set()
	for state in states:
		path=state['path']
		if not path:continue
		drone=state['drone']
		if not validate_path(path,dmap,nfzs,wh,drone['max_payload']):continue
		local=[];bad=False
		for step in path:
			if step.get('action')=='DELIVER':
				did=step.get('delivery_id')
				if did in gs:bad=True;break
				local.append(did)
		if bad:continue
		gs.update(local);manifest.append({'drone_id':drone['id'],'path':path})
	if not LARGE_MODE and time_left()>.3:manifest=improve_final_returns(manifest,dmap,{d['id']:d for d in drones},nfzs,wps,wh,stations)
	return{'flight_manifest':manifest}
def _do_rescue(states,dlvs,dmap,wh,nfzs,wps,stations,scheduler,drones,rl):
	delivered=set()
	for st in states.values():delivered.update(st['delivered'])
	rem=[d for d in dlvs if d['id']not in delivered]
	if nfzs:
		feasible=[]
		for d in rem:
			dwh=dist(wh,(d['x'],d['y']))
			if any(d['weight']<=dr['max_payload']+EPS for dr in drones)and dwh*1.5<=d['deadline']+1.:feasible.append(d)
		rem=feasible
	rem.sort(key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))))
	if LARGE_MODE:actual_rl=min(len(rem),200)
	else:actual_rl=min(len(rem),max(rl,len(rem)//2))
	for d in rem[:actual_rl]:
		if time_left()<1.:break
		best_st=None;bf=float('inf')
		for st in states.values():
			if d['weight']>st['drone']['max_payload']+EPS:continue
			sa=st['t']+dist(wh,(d['x'],d['y']))
			if sa>d['deadline']+3e2:continue
			snap=scheduler.snapshot();acts=build_trip(wh,[d],st['t'],nfzs,wps,stations,scheduler);scheduler.restore(snap)
			if acts is None:continue
			f=float(acts[-1]['t'])
			if f<bf:bf=f;best_st=st
		if best_st:try_append_trip(best_st,[d],wh,nfzs,wps,stations,scheduler,dmap)
	delivered=set()
	for st in states.values():delivered.update(st['delivered'])
	rem=[d for d in dlvs if d['id']not in delivered];rem.sort(key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))));pl=min(50 if LARGE_MODE else 200,len(rem))
	for i in range(pl):
		if time_left()<.8:break
		for j in range(i+1,min(i+(4 if LARGE_MODE else 12),pl)):
			d1,d2=rem[i],rem[j];cw=d1['weight']+d2['weight']
			for st in sorted(states.values(),key=lambda s:s['t']):
				if time_left()<.5:break
				if cw>st['drone']['max_payload']+EPS:continue
				sa=st['t']+min(dist(wh,(d1['x'],d1['y'])),dist(wh,(d2['x'],d2['y'])))
				if sa>min(d1['deadline'],d2['deadline'])+3e2:continue
				snap=scheduler.snapshot();acts=build_trip(wh,[d1,d2],st['t'],nfzs,wps,stations,scheduler);scheduler.restore(snap)
				if acts and validate_path(acts,dmap,nfzs,wh,st['drone']['max_payload']):
					ok=True
					for step in acts:
						if step.get('action')=='DELIVER':
							did=step.get('delivery_id');dm=dmap.get(did)
							if dm and float(step['t'])>dm['deadline']+.01:ok=False;break
					if ok:
						try_append_trip(st,[d1,d2],wh,nfzs,wps,stations,scheduler,dmap)
						if d1['id']in st['delivered']or d2['id']in st['delivered']:break
	delivered=set()
	for st in states.values():delivered.update(st['delivered'])
	rem=[d for d in dlvs if d['id']not in delivered];rem.sort(key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))));rl2=min(actual_rl,len(rem))
	for d in rem[:rl2]:
		if time_left()<.5:break
		for st in sorted(states.values(),key=lambda s:s['t']):
			if d['weight']>st['drone']['max_payload']+EPS:continue
			sa=st['t']+dist(wh,(d['x'],d['y']))
			if sa>d['deadline']+3e2:continue
			if try_append_trip(st,[d],wh,nfzs,wps,stations,scheduler,dmap):break
	if not LARGE_MODE:
		delivered=set()
		for st in states.values():delivered.update(st['delivered'])
		rem=[d for d in dlvs if d['id']not in delivered];rem.sort(key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))));rl3=min(actual_rl,300)
		for d in rem[:rl3]:
			if time_left()<.3:break
			best_st=None;bf=float('inf')
			for st in states.values():
				if d['weight']>st['drone']['max_payload']+EPS:continue
				snap=scheduler.snapshot();acts=build_trip(wh,[d],st['t'],nfzs,wps,stations,scheduler);scheduler.restore(snap)
				if acts is None:continue
				f=float(acts[-1]['t'])
				if f<bf:bf=f;best_st=st
			if best_st:try_append_trip(best_st,[d],wh,nfzs,wps,stations,scheduler,dmap)
	if not LARGE_MODE:
		delivered=set()
		for st in states.values():delivered.update(st['delivered'])
		rem=[d for d in dlvs if d['id']not in delivered];rem.sort(key=lambda d:(d['deadline'],dist(wh,(d['x'],d['y']))));tl=min(60,len(rem))
		for i in range(tl):
			if time_left()<.5:break
			for j in range(i+1,min(i+5,tl)):
				for k in range(j+1,min(j+4,tl)):
					d1,d2,d3=rem[i],rem[j],rem[k];cw=d1['weight']+d2['weight']+d3['weight']
					for st in sorted(states.values(),key=lambda s:s['t']):
						if cw>st['drone']['max_payload']+EPS:continue
						if try_append_trip(st,[d1,d2,d3],wh,nfzs,wps,stations,scheduler,dmap):
							if d1['id']in st['delivered']:break
def solve_core(data,strategy='base'):
	if strategy=='dynamic_fast':return solve_dynamic(data,strategy)
	if strategy=='fast_large':return solve_fast_large(data)
	global DENSE_NFZ_MODE,VERY_DENSE_NFZ_MODE,NFZ_MAX_END,LARGE_MODE;ms=data['map_size'];wh=float(ms[0])/2.,float(ms[1])/2.;drones=[{'id':d['id'],'max_payload':float(d['max_payload'])}for d in data.get('drones',[])];dlvs=[{'id':d['id'],'x':float(d['x']),'y':float(d['y']),'weight':float(d['weight']),'deadline':float(d['deadline'])}for d in data.get('deliveries',[])];stations=[{'x':float(s['x']),'y':float(s['y']),'slots':int(s.get('slots',1))}for s in data.get('charging_stations',[])];nfzs=normalize_nfzs(data.get('no_fly_zones',[]));NFZ_MAX_END=max((n['T_end']for n in nfzs),default=.0);DENSE_NFZ_MODE=len(nfzs)>=70 or len(nfzs)*max(1,len(dlvs))>=60000;VERY_DENSE_NFZ_MODE=len(nfzs)>=350 or len(nfzs)*max(1,len(dlvs))>=450000;wps=gen_waypoints(nfzs)
	if strategy=='wait':wps=[]
	if not drones:return{'flight_manifest':[]}
	mc=max(d['max_payload']for d in drones)
	if not stations:dlvs=[d for d in dlvs if d['weight']<=mc+EPS and dist(wh,(d['x'],d['y']))*(2.+d['weight'])<=BATTERY_CAP+.5]
	else:dlvs=[d for d in dlvs if d['weight']<=mc+EPS]
	feasible=[]
	for d in dlvs:
		dwh=dist(wh,(d['x'],d['y']));can_carry=any(d['weight']<=dr['max_payload']+EPS for dr in drones)
		if not can_carry:continue
		detour=1.5 if nfzs else 1.
		if dwh*detour>d['deadline']+1.:continue
		feasible.append(d)
	dlvs=feasible;dmap={d['id']:d for d in dlvs}
	if strategy=='spatial':asgn=assign_spatial(drones,dlvs,wh)
	elif strategy=='deadline_distance':asgn=assign_dd(drones,dlvs,wh)
	elif strategy=='capacity':asgn=assign_deliveries(sorted(drones,key=lambda d:(-d['max_payload'],d['id'])),dlvs,wh)
	elif strategy=='closest':asgn=assign_closest(drones,dlvs,wh)
	else:asgn=assign_deliveries(drones,dlvs,wh)
	scheduler=ChargeScheduler(stations);states={d['id']:{'drone':d,'t':.0,'path':[],'delivered':set()}for d in drones}
	for drone in drones:
		assigned=asgn.get(drone['id'],[])
		if not assigned:continue
		gm='deadline'
		if strategy=='spatial':gm='spatial'
		elif strategy=='distance':gm='distance'
		elif strategy=='urgent':gm='urgent'
		elif strategy=='weight_first':gm='weight_first'
		bt=group_trips(assigned,drone['max_payload'],wh,bool(nfzs),bool(stations),gm);trips=[]
		for tr in bt:trips.extend(split_trip(tr,wh,bool(stations),bool(nfzs)))
		state=states[drone['id']]
		for trip in trips:
			try_append_trip(state,trip,wh,nfzs,wps,stations,scheduler,dmap)
			for single in sorted(trip,key=lambda d:d['deadline']):
				if single['id']in state['delivered']:continue
				try_append_trip(state,[single],wh,nfzs,wps,stations,scheduler,dmap)
	rl=RESCUE_LIMIT_WITH_NFZ if nfzs else RESCUE_LIMIT_NO_NFZ;_do_rescue(states,dlvs,dmap,wh,nfzs,wps,stations,scheduler,drones,rl);manifest=[];gs=set()
	for drone in drones:
		st=states[drone['id']];path=st['path']
		if not path:continue
		if not validate_path(path,dmap,nfzs,wh,drone['max_payload']):continue
		local=[];bad=False
		for step in path:
			if step.get('action')=='DELIVER':
				did=step.get('delivery_id')
				if did in gs:bad=True;break
				local.append(did)
		if bad:continue
		gs.update(local);manifest.append({'drone_id':drone['id'],'path':path})
	manifest=improve_final_returns(manifest,dmap,{d['id']:d for d in drones},nfzs,wps,wh,stations);return{'flight_manifest':manifest}
def manifest_valid(manifest,data):
	ms=data['map_size'];wh=float(ms[0])/2.,float(ms[1])/2.;nfzs=normalize_nfzs(data.get('no_fly_zones',[]));dmap={d['id']:{'id':d['id'],'x':float(d['x']),'y':float(d['y']),'weight':float(d['weight']),'deadline':float(d['deadline'])}for d in data.get('deliveries',[])};dbid={d['id']:{'id':d['id'],'max_payload':float(d['max_payload'])}for d in data.get('drones',[])};seen=set();sp={(round(float(s['x']),3),round(float(s['y']),3))for s in data.get('charging_stations',[])};wk=round(wh[0],3),round(wh[1],3)
	for flight in manifest:
		drone=dbid.get(flight.get('drone_id'))
		if not drone:return False
		path=flight.get('path',[])
		if not validate_path(path,dmap,nfzs,wh,drone['max_payload']):return False
		if path:
			end=path[-1];ek=round(float(end['x']),3),round(float(end['y']),3)
			if ek!=wk and ek not in sp:return False
		for step in path:
			if step.get('action')=='DELIVER':
				did=step.get('delivery_id')
				if did in seen:return False
				seen.add(did)
	return True
def solve(data):
	global _SOLVE_START,LARGE_MODE;_SOLVE_START=time.time();m=len(data.get('deliveries',[]));k=len(data.get('no_fly_zones',[]));LARGE_MODE=m>=LARGE_DLV_THRESH;vd=k>=350 or k*max(1,m)>=450000;dns=k>=70 or k*max(1,m)>=60000
	if m>=LARGE_DLV_THRESH:strats=['fast_large']
	elif vd:strats=['dynamic_fast']
	elif k and(m>=1200 or k>=120 or k*max(1,m)>=120000):strats=['dynamic_fast','distance','base']
	elif k>=220 or k*max(1,m)>=200000:strats=['dynamic_fast','distance','base']
	else:
		strats=['base']
		if dns:strats.extend(['dynamic_fast','distance','wait'])
		elif m<=900:
			if k and m>500:strats.extend(['dynamic_fast','spatial','distance','wait','closest'])
			else:strats.extend(['deadline_distance','spatial','dynamic_fast','distance','capacity','urgent','closest','weight_first'])
		elif k==0 and m<=6000:strats.extend(['dynamic_fast','deadline_distance','distance','urgent','closest','weight_first'])
		elif k>0 and m>900:strats.extend(['dynamic_fast','distance','wait','closest'])
	best=None;bs=-float('inf')
	for strat in strats:
		if time_left()<(1. if m>=LARGE_DLV_THRESH else .5):break
		out=solve_core(data,strat);man=out.get('flight_manifest',[])
		if not manifest_valid(man,data):continue
		dmap={d['id']:{'id':d['id'],'x':float(d['x']),'y':float(d['y']),'weight':float(d['weight']),'deadline':float(d['deadline'])}for d in data.get('deliveries',[])};sc=score_manifest(man,dmap)
		if sc>bs+1e-07:bs=sc;best=out
	if best is None:return solve_core(data,'base')
	return best
def main():
	if len(sys.argv)>1:
		with open(sys.argv[1],'r',encoding='utf-8')as f:data=json.load(f)
	else:data=json.load(sys.stdin)
	print(json.dumps(solve(data),separators=(',',':')))
if __name__=='__main__':main()