"""Reference receiver, Python standard library only. Prints data; NEVER drives motors."""
import argparse
import json
import math
import socket
import time


class ReceiverState:
    def __init__(self, ball_type='red'):
        if ball_type not in ('red', 'multicolor'):
            raise ValueError('ball_type must be red or multicolor')
        self.ball_type = ball_type
        self.session=None;self.retired=set();self.seq=-1;self.coord=None
        self.latest=None;self.deadline=0
        self.reset_required=False;self.frame=-1
        self.retired_coordinates=set()
        self.generation=0

    def accept(self, data, now):
        if len(data)>1400:return False
        try:
            p=json.loads(data,parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            def finite_tree(v):
                if isinstance(v,float):return math.isfinite(v)
                if isinstance(v,list):return all(finite_tree(x) for x in v)
                if isinstance(v,dict):return all(finite_tree(x) for x in v.values())
                return True
            if not finite_tree(p):return False
            if not isinstance(p,dict) or p.get('protocol')!='air2s.vision' or type(p.get('version'))!=int or p.get('version')!=1:return False
            session=p['session'];seq=p['seq'];ttl=p['ttl_ms'];age=p['age_ms']
            if not isinstance(session,str) or len(session)!=32:return False
            if any(c not in '0123456789abcdefABCDEF' for c in session):return False
            if type(seq)!=int or seq<0 or type(ttl)!=int or not 100<=ttl<=2000:return False
            if type(p.get('sent_unix_ms'))!=int or p['sent_unix_ms']<0:return False
            if session in self.retired:return False
            fresh=p['fresh'];obs=p['observation']
            if type(fresh)!=bool:return False
            if fresh:
                if type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<ttl:return False
                if not isinstance(obs,dict) or not isinstance(obs.get('coord'),str):return False
                if len(obs['coord'])!=32 or any(c not in '0123456789abcdefABCDEF' for c in obs['coord']):return False
                if type(obs.get('field_valid'))!=bool or not isinstance(obs.get('boats'),list):return False
                def point(v):
                    return v is None or (isinstance(v,list) and len(v)==2 and
                        all(type(x) in (int,float) and math.isfinite(x) for x in v))
                # Normalize the selected typed target for existing downstream consumers.
                # Never fall back to a different color when the selected target is absent.
                if 'balls' in obs:
                    balls = obs['balls']
                    if not isinstance(balls,dict):return False
                    for kind, ball in balls.items():
                        if kind not in ('red','multicolor'):continue
                        if not isinstance(ball,dict) or not point(ball.get('xy_m')):return False
                        if ball.get('state') not in ('detected','predicted','lost'):return False
                        if ball.get('state')!='detected' and ball.get('xy_m') is not None:return False
                        if ball.get('xy_m') is not None and not obs['field_valid']:return False
                        if type(ball.get('score')) not in (int,float) or not math.isfinite(ball['score']):return False
                    obs['ball'] = balls.get(self.ball_type,dict(state='lost',xy_m=None,score=0.))
                elif self.ball_type != 'red':
                    # The legacy ball field identifies only the original red target.
                    obs['ball'] = dict(state='lost',xy_m=None,score=0.)
                obs['ball_type'] = self.ball_type
                if not isinstance(obs.get('ball'),dict) or not isinstance(obs.get('goals'),list):return False
                if not point(obs['ball'].get('xy_m')):return False
                if obs['ball'].get('xy_m') is not None and not obs['field_valid']:return False
                if obs['ball'].get('state') not in ('detected','predicted','lost'):return False
                if obs['ball']['state']!='detected' and obs['ball'].get('xy_m') is not None:return False
                if type(obs.get('frame'))!=int or obs['frame']<1:return False
                if type(obs.get('processed_unix_ms'))!=int or not 0<=obs['processed_unix_ms']<=p['sent_unix_ms']:return False
                if obs.get('field_status') not in ('calibrated','locked','manual_fixed','unknown','camera_unverified','disabled'):return False
                if type(obs['ball'].get('score')) not in (int,float) or not math.isfinite(obs['ball']['score']):return False
                ids=set()
                if not point(obs.get('size_m')):return False
                for boat in obs['boats']:
                    if type(boat.get('id'))!=int or not point(boat.get('xy_m')):return False
                    if boat['id']<0 or boat['id'] in ids:return False
                    ids.add(boat['id'])
                    heading=boat.get('tag_heading_deg')
                    if heading is not None and (type(heading) not in (int,float) or not math.isfinite(heading) or not -180<=heading<=180):return False
                    if boat.get('possession') not in ('held','candidate','free','unknown'):return False
                for goal in obs['goals']:
                    if goal.get('side') not in ('left','right'):return False
                    ends=goal.get('ends_m')
                    if not isinstance(ends,list) or len(ends)!=2 or not all(v is not None and point(v) for v in ends):return False
            elif obs is not None:return False
        except (ValueError,KeyError,TypeError,UnicodeError,AttributeError,RecursionError):return False
        coord=obs['coord'] if fresh else p.get('coord')
        if (session,coord) in self.retired_coordinates:return False
        changed=self.session is not None and (session!=self.session or (fresh and self.coord is not None and coord!=self.coord))
        # Only a fully validated fresh observation may replace an established stream.
        if changed and not fresh:return False
        if session==self.session and seq<=self.seq:return False
        if changed:
            self.retired_coordinates.add((self.session,self.coord))
            if session!=self.session:self.retired.add(self.session)
            self.session=None;self.coord=None;self.seq=-1;self.frame=-1
            self.latest=None;self.deadline=0;self.generation+=1
        if session!=self.session:
            if self.session:self.retired.add(self.session)
            self.session=session;self.seq=-1;self.latest=None;self.coord=None
        if seq<=self.seq:return False
        if fresh and obs['frame']<self.frame:return False
        self.seq=seq
        self.latest=None
        if fresh:
            self.coord=obs['coord']
            if obs['frame']<self.frame:return False
            deadline=now+min(.5,(ttl-age)/1000)
            self.deadline=min(self.deadline,deadline) if obs['frame']==self.frame else deadline
            self.frame=obs['frame']
            # Preserve separately valid boat coordinates even if this frame lacks field detection.
            # Consumers must require a previously established matching map identity.
            self.latest=obs
        return True

    def current(self,now):
        if now>=self.deadline:self.latest=None
        return self.latest
