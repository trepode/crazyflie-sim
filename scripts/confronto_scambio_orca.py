#!/usr/bin/env python3
"""Scambio simmetrico: confronto ORCA puro, ALAN-lite e passaggio a destra (SOLO SITL).

Due Crazyflie decollano, poi il goal di ciascuno e' la posizione iniziale
dell'altro, misurata dopo il decollo. Non ci sono waypoint casuali.

Esempi:
  python confronto_scambio_orca.py --mode pure --duration 70 --orca-radius .25 --output confronto_puro_025
  python confronto_scambio_orca.py --mode alan-lite --duration 70 --orca-radius .25 --output confronto_alan_025
  python confronto_scambio_orca.py --mode right-bias --duration 70 --orca-radius .25 --output confronto_destra_025

Le tre modalita' usano identici ORCA, PID, limiti e condizioni iniziali.
ALAN-lite e' un'euristica di fuga, non ALAN originale con apprendimento.
Right-bias e' un passaggio a destra deterministico ispirato al tie-break
di NH-ORCA, NON una riproduzione dell'algoritmo NH-ORCA.
La orca_lib.py utilizzata e' ricostruita e non verificata con RVO2 ufficiale.
Non usare su Crazyflie fisici.
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path
from threading import Lock
import warnings

from collections import deque

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# Match the REAL script (unmodified).
URI1 = "radio://0/80/2M/E7E7E7E7E9"
URI2 = "radio://0/80/2M/E7E7E7E7E7"
PREFERRED_SPEED=0.05
MAX_SPEED=0.07
DT=0.05
TIME_HORIZON=3.0
GOAL_TOLERANCE=0.12
EMERGENCY_DISTANCE=0.40
MAX_DISTANCE_FROM_START=1.20
FLIGHT_HEIGHT=0.50
MAX_VERTICAL_CHANGE=0.20
VELOCITY_ALPHA=0.25
MAX_VALID_MOTIVE_SPEED=2.0
MAX_TELEMETRY_AGE=0.40


def norm(v):return math.hypot(*v)
def dist(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def clamp(v,a,b):return max(a,min(b,v))

# Motive -> velocities
def motive_to_cf_velocity(v_motive):
    """
    Convert a horizontal velocity expressed in Motive X-Z
    coordinates into Crazyflie world X-Y coordinates.

    Experimental calibration:
        +X_CF  ->  -Z_Motive
        +Y_CF  ->  -X_Motive

    Therefore:
        vx_CF = -vz_Motive
        vy_CF = -vx_Motive

    ORCA always works in Motive X-Z.
    This transformation is applied ONLY immediately before
    sending the velocity to the Crazyflie.
    """
    vx_motive, vz_motive = v_motive
    vx_cf = -vz_motive
    vy_cf = -vx_motive
    return vx_cf, vy_cf

def create_tracker():
    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False,
    )
    tracker.connect()
    print("Motive connected.")
    return tracker

def read_two_drones(tracker):
    """
    natnet update, then read both rigid bodies
    Motive:
        X, Y (vertical), Z
    """
    tracker.update()
    info1 = tracker.get_info_by_id(MOTIVE_ID_1)
    info2 = tracker.get_info_by_id(MOTIVE_ID_2)

    if (info1 is None or info2 is None):
        return None

    return (info1["position"], info2["position"])

def send_motive_velocity(cf, v_motive):
    vx_cf, vy_cf = motive_to_cf_velocity(v_motive)
    print("sending this velocities from motive to crazyflies:")
    print(f"x_velocity: {vx_cf}")
    print(f"y_velocity: {vy_cf}")
    cf.high_level_commander.go_to(x=vx_cf * COMMAND_DURATION,y=vy_cf * COMMAND_DURATION,z=0.0,yaw=0.0,duration_s=COMMAND_DURATION,relative=True)
    return (vx_cf, vy_cf)

def velocity_toward_goal(position,goal,speed):
    dx=goal[0]-position[0]
    dy=goal[1]-position[1]
    d=math.hypot(dx,dy)
    return (0.,0.) if d<GOAL_TOLERANCE else (speed*dx/d,speed*dy/d)

class FixedSwapNavigator:
    """Goal fisso = posizione iniziale dell'altro drone. Si registra un solo arrivo."""
    def __init__(self, name, start, goal):
        self.name = name
        self.start = start
        self.goal = goal
        self.waypoints = 1
        self.reached = 0
        self.reach_time = None
        self.events = [(0.0, 'SWAP_GOAL', goal)]

    def update_goal(self, t, position, neighbour=None):
        if not self.reached and dist(position, self.goal) < GOAL_TOLERANCE:
            self.reached = 1
            self.reach_time = t
            self.events.append((t, 'GOAL_REACHED', self.goal))
            print(f'{self.name}: GOAL REACHED a t={t:.2f}s')

    def preferred_velocity(self, position):
        if self.reached:
            return (0.0, 0.0)
        return velocity_toward_goal(position, self.goal, PREFERRED_SPEED)

# ============================================================== 
# ALAN-LITE: SOLO selezione di v_pref, NON modifica i vincoli ORCA.
# ATTENZIONE: e' una nostra euristica ispirata alla selezione delle
# azioni di ALAN, NON la versione con apprendimento del paper.
# ============================================================== 
class ALANLiteSelector:
    STUCK_WINDOW = 3.0        # secondi di progresso osservato
    MIN_PROGRESS = 0.025      # metri di avvicinamento al waypoint
    NEIGHBOR_LIMIT = 1.0      # metri: vicino che puo' interferire
    MAX_BASE_PROGRESS = 0.012 # m/s: ORCA puro progredisce troppo poco
    ESCAPE_TIME = 2.5        # secondi per mantenere una direzione
    COOLDOWN = 0.5           # secondi prima di tentare ancora
    LOOKAHEAD = 1.5          # secondi per escludere manovre verso geofence

    def __init__(self, start):
        self.start = start
        self.history = deque()
        self.goal = None
        self.active_until = -1.0
        self.cooldown_until = 0.0
        self.action = 'GOAL'
        self.angle = 0.0
        self.activations = 0
        self.last_progress = float('nan')

    @staticmethod
    def rotate(v, degrees):
        theta = math.radians(degrees)
        c, s = math.cos(theta), math.sin(theta)
        return (c*v[0] - s*v[1], s*v[0] + c*v[1])

    def select(self, t, position, goal, neighbor, v_goal,
               lp, ld, orca_solver):
        """Return (preferred_velocity, action). ORCA is applied AFTER this call."""
        if self.goal != goal:
            self.goal = goal
            self.history.clear()
            self.active_until = -1.0
            self.action = 'GOAL'
        distance_goal = dist(position, goal)
        self.history.append((t, distance_goal))
        while self.history and t-self.history[0][0] > self.STUCK_WINDOW:
            self.history.popleft()
        self.last_progress = self.history[0][1] - distance_goal

        if t < self.active_until:
            return self.rotate(v_goal, self.angle), self.action
        if self.action != 'GOAL':
            self.action = 'GOAL'
            self.cooldown_until = t + self.COOLDOWN
        if t < self.cooldown_until:
            return v_goal, 'GOAL'
        if not self.history or t-self.history[0][0] < self.STUCK_WINDOW - 0.10:
            return v_goal, 'GOAL'
        if self.last_progress >= self.MIN_PROGRESS:
            return v_goal, 'GOAL'
        if dist(position, neighbor) > self.NEIGHBOR_LIMIT:
            return v_goal, 'GOAL'
        if norm(v_goal) < 1e-6:
            return v_goal, 'GOAL'

        # A deadlock ORCA, non un problema del controller, e' piu' plausibile
        # se la velocita' ORCA senza modifica produce poco progresso previsto.
        try:
            base_cmd = orca_solver(v_goal, lp, ld, MAX_SPEED)
        except ValueError:
            return v_goal, 'GOAL'
        goal_unit = (v_goal[0]/norm(v_goal), v_goal[1]/norm(v_goal))
        predicted_goal_speed = (base_cmd[0]*goal_unit[0]
                                + base_cmd[1]*goal_unit[1])
        if predicted_goal_speed > self.MAX_BASE_PROGRESS:
            return v_goal, 'GOAL'

        # Simmetrici ma separati: entrambi preferiscono prima +45 gradi
        # rispetto alla direzione del PROPRIO goal (non verso +Y globale).
        candidates = ((45, 'LEFT_45'), (-45, 'RIGHT_45'),
                      (90, 'LEFT_90'), (-90, 'RIGHT_90'))
        best = None
        for angle, name in candidates:
            pref = self.rotate(v_goal, angle)
            try:
                command = orca_solver(pref, lp, ld, MAX_SPEED)
            except ValueError:
                continue
            future = (position[0]+command[0]*self.LOOKAHEAD,
                      position[1]+command[1]*self.LOOKAHEAD)
            if dist(future, self.start) > MAX_DISTANCE_FROM_START-0.05:
                continue
            # Oltre a progredire, premiamo una velocita' NON nulla.
            # Questo e' un punteggio euristico: non e' la reward di ALAN.
            toward = command[0]*goal_unit[0]+command[1]*goal_unit[1]
            score = 0.25*norm(command)+0.20*toward
            if best is None or score > best[0]+1e-8:
                best = (score, angle, name, pref, command)
        if best is None or norm(best[4]) < 0.01:
            return v_goal, 'GOAL'

        self.angle, self.action = best[1], best[2]
        self.active_until = t + self.ESCAPE_TIME
        self.activations += 1
        return best[3], self.action

# ==============================================================
# RIGHT-BIAS: convenzione "passa a destra" (non NH-ORCA originale).
# Incontro frontale: ogni drone ruota la SUA velocita' preferita
# di -60 gradi rispetto alla direzione del proprio obiettivo.
# I due droni quindi si spostano su lati opposti dell'asse di
# incontro: CF1 verso -y, CF2 verso +y. ORCA resta invariato.
# ==============================================================
class RightPassingSelector:
    TRIGGER_DISTANCE = 0.90   # Inizia la manovra quando ci si avvicina
    RELEASE_MARGIN = 0.10     # Distanza ORCA + margine, dopo superamento
    BIAS_DEGREES = -60.0

    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.committed = False
        self.completed = False
        self.activations = 0
        self.action = 'GOAL'

    def select(self, position, neighbor, v_goal, combined_radius):
        if norm(v_goal) < 1e-8:
            return v_goal, 'GOAL'
        if self.completed:
            return v_goal, 'GOAL'

        sep = dist(position, neighbor)
        if not self.committed and sep <= self.TRIGGER_DISTANCE:
            self.committed = True
            self.activations += 1

        # Il vicino e' "dietro" quando il drone lo ha gia' superato
        # lungo l'asse iniziale start -> goal.
        axis = (self.goal[0] - self.start[0],
                self.goal[1] - self.start[1])
        axis_len = norm(axis)
        passed = False
        if axis_len > 1e-8:
            projection = ((neighbor[0] - position[0])*axis[0]
                          + (neighbor[1] - position[1])*axis[1])/axis_len
            passed = projection <= 0

        if self.committed and passed and sep > combined_radius+self.RELEASE_MARGIN:
            self.committed = False
            self.completed = True

        if self.committed:
            return ALANLiteSelector.rotate(v_goal, self.BIAS_DEGREES), 'RIGHT_60'
        return v_goal, 'GOAL'

class VelocityEstimator:
    """Same finite-difference + EMA logic as real_orca.py (horizontal only)."""
    def __init__(self,alpha=0.25):
        self.alpha=alpha
        self.previous_position=None
        self.previous_time=None
        self.velocity=(0.,0.)
    def reset(self,position):
        self.previous_position=position
        self.previous_time=time.monotonic()
        self.velocity=(0.,0.)
    def update(self,position):
        now=time.monotonic()
        if self.previous_position is None:
            self.reset(position)
            return self.velocity
        dt=now-self.previous_time
        if dt>0:
            raw=((position[0]-self.previous_position[0])/dt,
                 (position[1]-self.previous_position[1])/dt)
            if norm(raw)<=MAX_VALID_MOTIVE_SPEED:
                a=self.alpha
                self.velocity=(a*raw[0]+(1-a)*self.velocity[0],
                               a*raw[1]+(1-a)*self.velocity[1])
            self.previous_position=position
            self.previous_time=now
        return self.velocity

class Telemetry:
    def __init__(self,cf,name):
        self.name=name
        self._lock=Lock()
        self._latest=None
        self._recv_time=0.
        self.cfg=LogConfig(name='Orca'+name,period_in_ms=50)
        for var in ('x','y','z','vx','vy','vz'):
            self.cfg.add_variable('stateEstimate.'+var,'float')
        cf.log.add_config(self.cfg)
        if not self.cfg.valid:
            raise RuntimeError(f'{name}: stateEstimate variables are absent from firmware LOG TOC')
        self.cfg.data_received_cb.add_callback(self._on_data)
        self.cfg.error_cb.add_callback(self._on_error)
        self.cfg.start()
    def _on_data(self,ts,data,logconf):
        with self._lock:
            self._latest={k:float(data['stateEstimate.'+k]) for k in ('x','y','z','vx','vy','vz')}
            self._recv_time=time.monotonic()
    def _on_error(self,logconf,msg):
        print(f'Telemetry {self.name}: {msg}',file=sys.stderr)
    def read(self,required=True):
        with self._lock:
            d=self._latest.copy() if self._latest else None
            age=time.monotonic()-self._recv_time
        if required and (d is None or age>MAX_TELEMETRY_AGE):
            raise RuntimeError(f'Telemetry {self.name} missing/stale ({age:.2f}s)')
        return d
    def close(self):
        self.cfg.stop()

def run(args):
    try:
        from orca_lib import compute_orca_constraint, choose_orca_velocity
    except ImportError as exc:
        raise SystemExit('Manca orca_lib.py: mettilo nella cartella orca_test.') from exc

    cflib.crtp.init_drivers(enable_debug_driver=False)
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=True)
    rows=[]
    status='NOT_STARTED'
    with SyncCrazyflie(URI1,cf=Crazyflie(rw_cache='./cache')) as s1:
      with SyncCrazyflie(URI2,cf=Crazyflie(rw_cache='./cache')) as s2:
        # Use the PID, which supports horizontal velocity setpoints.
        for scf in (s1,s2):
            scf.cf.param.set_value('stabilizer.controller','1')
        time.sleep(.5)
        t1=Telemetry(s1.cf,'1'); t2=Telemetry(s2.cf,'2')
        airborne=False
        takeoff_ok=False
        takeoff_log=[]
        def step_send(a,b,z1,z2):
            # World-frame velocities: no Motive conversion in simulation.
            s1.cf.commander.send_velocity_world_setpoint(a[0],a[1],z1,0.0)
            s2.cf.commander.send_velocity_world_setpoint(b[0],b[1],z2,0.0)
        try:
            print('Waiting for both position estimates...')
            begin=time.monotonic()
            while time.monotonic()-begin<5:
                if t1.read(required=False) is not None and t2.read(required=False) is not None:
                    break
                time.sleep(DT)
            _=t1.read();_=t2.read()
            # Some firmware configurations require explicit arming.
            s1.cf.supervisor.send_arming_request(True)
            s2.cf.supervisor.send_arming_request(True)
            print('Takeoff: both virtual drones to 0.50 m')
            print('Starting telemetry: CF1=', t1.read(), 'CF2=', t2.read())
            # Keep both streams in the SAME mode; no MotionCommander thread.
            airborne=True
            takeoff_start=time.monotonic()
            ready_at=None
            next_print=0.0
            while time.monotonic()-takeoff_start<7.0:
                p1=t1.read();p2=t2.read()
                takeoff_elapsed=time.monotonic()-takeoff_start
                takeoff_log.append(dict(t=takeoff_elapsed, z1=p1['z'], vz1=p1['vz'],
                                        z2=p2['z'], vz2=p2['vz'], target_z=FLIGHT_HEIGHT))
                if takeoff_elapsed >= next_print:
                    print(f'TAKEOFF t={takeoff_elapsed:.1f}s '
                          f'CF1 z={p1["z"]:+.3f} vz={p1["vz"]:+.3f} '
                          f'CF2 z={p2["z"]:+.3f} vz={p2["vz"]:+.3f} '
                          f'target={FLIGHT_HEIGHT:.2f}')
                    next_print += .5
                if not all(math.isfinite(v) for v in (p1['z'],p1['vz'],p2['z'],p2['vz'])):
                    raise RuntimeError('Telemetry non finite durante decollo')
                s1.cf.commander.send_hover_setpoint(0.,0.,0.,FLIGHT_HEIGHT)
                s2.cf.commander.send_hover_setpoint(0.,0.,0.,FLIGHT_HEIGHT)
                if abs(p1['z']-FLIGHT_HEIGHT)<.08 and abs(p2['z']-FLIGHT_HEIGHT)<.08:
                    if ready_at is None: ready_at=time.monotonic()
                    if time.monotonic()-ready_at>1.0: break
                else: ready_at=None
                time.sleep(DT)
            else:
                print('TAKEOFF FAILED. Ultime quote:', p1['z'], p2['z'])
                raise RuntimeError('Takeoff altitude not reached; check estimator and controller')
            takeoff_ok=True
            print('TAKEOFF_OK: entrambi stabilizzati a quota 0.50 m')
            if args.takeoff_only:
                print('DIAGNOSTICA: --takeoff-only, nessun comando ORCA inviato')
                return

            # =====================================================
            # Scambio simmetrico: goal1 = partenza CF2 e viceversa.
            # ORCA e controller invariati.
            # =====================================================
            init1=t1.read(); init2=t2.read()
            start1=(init1['x'],init1['y'])
            start2=(init2['x'],init2['y'])
            sep0=dist(start1,start2)
            if sep0<EMERGENCY_DISTANCE+.10:
                raise RuntimeError(f'Droni troppo vicini all inizio (<{EMERGENCY_DISTANCE+0.10:.2f}m)')
            print(f'START: CF1 {start1}, CF2 {start2}, distanza={sep0:.3f}m')
            print('ORCA: radius=%.2fm, combined=%.2fm, horizon=%.1fs, DT=%.2fs' % (
                args.orca_radius,2*args.orca_radius,TIME_HORIZON,DT))
            print(f'NAVIGATION: FIXED SWAP; mode={args.mode}; emergency={EMERGENCY_DISTANCE:.2f}m')
            exp1=FixedSwapNavigator('CF1',start1,start2)
            exp2=FixedSwapNavigator('CF2',start2,start1)
            print(f'GOAL FISSI: CF1={exp1.goal} CF2={exp2.goal}')
            print('GOAL_TOLERANCE=%.2fm; i goal NON vengono riassegnati' % GOAL_TOLERANCE)
            z01,z02=init1['z'],init2['z']
            est1,est2=VelocityEstimator(VELOCITY_ALPHA),VelocityEstimator(VELOCITY_ALPHA)
            est1.reset(start1);est2.reset(start2)
            a1,a2=ALANLiteSelector(start1),ALANLiteSelector(start2)
            r1,r2=RightPassingSelector(start1,start2),RightPassingSelector(start2,start1)
            t_start=time.monotonic();status='TIMEOUT'
            last_print=-1.
            try:
                while True:
                    loop=time.monotonic()
                    t=loop-t_start
                    if t>=args.duration:
                        break
                    st1,st2=t1.read(),t2.read()
                    p1,p2=(st1['x'],st1['y']),(st2['x'],st2['y'])
                    sep=dist(p1,p2)
                    if not all(math.isfinite(z) for z in (st1['x'],st1['y'],st1['z'],
                                                          st2['x'],st2['y'],st2['z'])):
                        status='BAD_ESTIMATE';break
                    if sep < EMERGENCY_DISTANCE:
                        status=f'SAFETY_DISTANCE_{sep:.3f}';break
                    if dist(start1,p1)>MAX_DISTANCE_FROM_START or dist(start2,p2)>MAX_DISTANCE_FROM_START:
                        status='GEOFENCE';break
                    if abs(st1['z']-z01)>MAX_VERTICAL_CHANGE or abs(st2['z']-z02)>MAX_VERTICAL_CHANGE:
                        status='VERTICAL_CHANGE';break
                    exp1.update_goal(t,p1,p2)
                    exp2.update_goal(t,p2,p1)
                    if exp1.reached and exp2.reached:
                        status='BOTH_REACHED'
                        # Stop comandi di velocita' e avvia atterraggio.
                        step_send((0.,0.), (0.,0.), 0., 0.)
                        break
                    v1,v2=est1.update(p1),est2.update(p2)
                    goal_pref1=exp1.preferred_velocity(p1)
                    goal_pref2=exp2.preferred_velocity(p2)
                    # I vincoli ORCA restano identici alla baseline.
                    lp1,ld1,u1=compute_orca_constraint(p1,v1,p2,v2,args.orca_radius,TIME_HORIZON,DT)
                    lp2,ld2,u2=compute_orca_constraint(p2,v2,p1,v1,args.orca_radius,TIME_HORIZON,DT)
                    if args.mode == 'alan-lite':
                        pref1,action1=a1.select(t,p1,exp1.goal,p2,goal_pref1,lp1,ld1,choose_orca_velocity)
                        pref2,action2=a2.select(t,p2,exp2.goal,p1,goal_pref2,lp2,ld2,choose_orca_velocity)
                    elif args.mode == 'right-bias':
                        pref1,action1=r1.select(p1,p2,goal_pref1,2*args.orca_radius)
                        pref2,action2=r2.select(p2,p1,goal_pref2,2*args.orca_radius)
                    else:
                        pref1,pref2=goal_pref1,goal_pref2
                        action1=action2='GOAL'
                    try:
                        c1=choose_orca_velocity(pref1,lp1,ld1,MAX_SPEED)
                        c2=choose_orca_velocity(pref2,lp2,ld2,MAX_SPEED)
                    except ValueError as exc:
                        status='ORCA_INFEASIBLE'
                        print('ORCA non ha una velocita ammissibile:',exc)
                        break
                    # Controller verticale come in v2_debug gia funzionante.
                    vz1=clamp(1.5*(z01-st1['z']),-.12,.12)
                    vz2=clamp(1.5*(z02-st2['z']),-.12,.12)
                    step_send(c1,c2,vz1,vz2)
                    rows.append(dict(t=round(t,4),mode=args.mode,distance=sep,waypoint_count1=exp1.waypoints,waypoint_count2=exp2.waypoints,
                        reached1=exp1.reached,reached2=exp2.reached,
                        goal1x=exp1.goal[0],goal1y=exp1.goal[1],
                        goal2x=exp2.goal[0],goal2y=exp2.goal[1],
                        goal_distance1=dist(p1,exp1.goal),goal_distance2=dist(p2,exp2.goal),
                        x1=p1[0],y1=p1[1],z1=st1['z'],x2=p2[0],y2=p2[1],z2=st2['z'],
                        vx1=st1['vx'],vy1=st1['vy'],vx2=st2['vx'],vy2=st2['vy'],
                        pref1x=pref1[0],pref1y=pref1[1],pref2x=pref2[0],pref2y=pref2[1],
                        orca1x=c1[0],orca1y=c1[1],orca2x=c2[0],orca2y=c2[1],
                        orca_correction1=dist(pref1,c1),orca_correction2=dist(pref2,c2),
                        orca_u1=norm(u1),orca_u2=norm(u2),
                        alan_action1=action1,alan_action2=action2,
                        alan_activations1=a1.activations,alan_activations2=a2.activations,
                        right_activations1=r1.activations,right_activations2=r2.activations,
                        progress_window1=a1.last_progress,progress_window2=a2.last_progress))
                    if t-last_print>=1.0:
                        print(f't={t:5.1f}s d={sep:.3f}m | '
                              f'CF1 cmd={tuple(round(x,3) for x in c1)} '
                              f'CF2 cmd={tuple(round(x,3) for x in c2)} '
                              f'goal_d=({dist(p1,exp1.goal):.3f}, {dist(p2,exp2.goal):.3f}) '
                              f'actions=({action1},{action2})')
                        last_print=t
                    pause=DT-(time.monotonic()-loop)
                    if pause>0:time.sleep(pause)
            except KeyboardInterrupt:
                status='INTERRUPTED'
                print('Interruzione utente: atterraggio SITL...')
            print(f'End of test: {status}')
            print(f'ALAN-lite attivazioni: CF1={a1.activations}, CF2={a2.activations}')
            print(f'Right-bias attivazioni: CF1={r1.activations}, CF2={r2.activations}')
            print('Goal raggiunti: CF1=%d, CF2=%d (su due goal fissi)' % (exp1.reached,exp2.reached))
            print(f'Tempi di arrivo: CF1={exp1.reach_time}, CF2={exp2.reach_time}')
            active_steps=sum((r['orca_correction1']+r['orca_correction2'])>1e-5 for r in rows)
            print(f'Cicli con intervento ORCA: {active_steps}/{len(rows)}')
            if rows:
                print(f'Min separation: {min(r["distance"] for r in rows):.3f} m')
            # File eventi: goal fissi e raggiungimenti.
            all_events=[{'t':a,'agent':name,'event':kind,'point':repr(pos)}
                        for name,exp in [('CF1',exp1),('CF2',exp2)]
                        for a,kind,pos in exp.events]
            with (output/'events.csv').open('w',newline='') as fp:
                writer=csv.DictWriter(fp,fieldnames=['t','agent','event','point'])
                writer.writeheader();writer.writerows(sorted(all_events,key=lambda x:x['t']))
        finally:
            # Salva i dati anche se si verifica un errore o Ctrl+C.
            if rows:
                path=output/'crazy_sim_orca.csv'
                with path.open('w',newline='') as fp:
                    writer=csv.DictWriter(fp,fieldnames=list(rows[0]))
                    writer.writeheader();writer.writerows(rows)
                print('Saved:',path.resolve())
            # Registra la quota anche quando il decollo fallisce.
            if takeoff_log:
                log_file=output/'takeoff_diagnostics.csv'
                with log_file.open('w',newline='') as f:
                    writer=csv.DictWriter(f,fieldnames=list(takeoff_log[0]))
                    writer.writeheader();writer.writerows(takeoff_log)
                print('Saved takeoff diagnostic:',log_file.resolve())
            # Se fallisce il decollo NON tentare la discesa usando quote stimate inattendibili.
            # Questa via di arresto e' consentita ESCLUSIVAMENTE in MuJoCo/SITL.
            if airborne and not takeoff_ok:
                print('TAKEOFF FAILED: stop motors SOLO IN SIMULAZIONE, nessun ORCA')
                s1.cf.commander.send_stop_setpoint()
                s2.cf.commander.send_stop_setpoint()
                s1.cf.supervisor.send_arming_request(False)
                s2.cf.supervisor.send_arming_request(False)
            elif airborne and takeoff_ok:
                print('Landing virtual drones...')
                st1=t1.read(required=False); st2=t2.read(required=False)
                h1=max(.05,st1['z'] if st1 else FLIGHT_HEIGHT)
                h2=max(.05,st2['z'] if st2 else FLIGHT_HEIGHT)
                try:
                    for k in range(80):
                        frac=min(1.0,k/70.0)
                        target1=max(.05,h1*(1-frac))
                        target2=max(.05,h2*(1-frac))
                        s1.cf.commander.send_hover_setpoint(0.,0.,0.,target1)
                        s2.cf.commander.send_hover_setpoint(0.,0.,0.,target2)
                        time.sleep(DT)
                except KeyboardInterrupt:
                    print('Atterraggio SITL interrotto: stop motori simulati')
                finally:
                    # Strictly simulated CrazySim UDP only.
                    s1.cf.commander.send_stop_setpoint()
                    s2.cf.commander.send_stop_setpoint()
                    s1.cf.supervisor.send_arming_request(False)
                    s2.cf.supervisor.send_arming_request(False)
            t1.close();t2.close()

if __name__=='__main__':
    warnings.filterwarnings('once',category=DeprecationWarning,message=r'Using legacy TYPE_.*')
    ap=argparse.ArgumentParser(description='CrazySim ONLY: scambio simmetrico con ORCA puro, ALAN-lite o right-bias')
    ap.add_argument('--mode', choices=['pure','alan-lite','right-bias'], required=True, help='Strategia da testare')
    ap.add_argument('--duration',type=float,default=70.0,help='Durata massima del test in secondi')
    ap.add_argument('--orca-radius',type=float,default=.25,help='Raggio di sicurezza per drone [m]; default 0.25')
    ap.add_argument('--output',default=None,help='Cartella per log CSV')
    ap.add_argument('--takeoff-only',action='store_true',help='Controlla solo il decollo doppio')
    args=ap.parse_args()
    if args.output is None:
        args.output = 'scambio_' + args.mode.replace('-', '_')
    if 2*args.orca_radius <= EMERGENCY_DISTANCE + 0.05:
        ap.error(f'2*radius deve superare EMERGENCY_DISTANCE={EMERGENCY_DISTANCE:.2f}m di almeno 0.05m')
    if args.duration <= 0:
        ap.error('La durata deve essere positiva')
    run(args)
