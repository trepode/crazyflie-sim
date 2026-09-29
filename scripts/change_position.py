#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import math
import time
from collections import deque
from pathlib import Path
from threading import Event

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.swarm import CachedCfFactory, Swarm


# ============================================================
# CONFIGURAZIONE
# ============================================================

URIS = [
    'radio://0/80/2M/E7E7E7E7E5',
    'radio://0/80/2M/E7E7E7E7E9',
]

# Evita i problemi di permessi della vecchia cartella ./cache
CACHE_DIR = Path.home() / '.cache' / 'crazyflie-swarm'
CACHE_DIR.mkdir(parents=True, exist_ok=True)

TAKEOFF_HEIGHT = 0.5
TAKEOFF_TIME = 3.0
LANDING_TIME = 3.0

# Ogni tratto della semicirconferenza dura 2 secondi
SEGMENT_TIME = 2.0

# Tempo della rotazione finale di 180°
TURN_TIME = 3.0

# Tempo massimo concesso a ogni estimatore
ESTIMATOR_TIMEOUT = 25.0

logging.basicConfig(level=logging.WARNING)


# ============================================================
# TRAIETTORIA DI SCAMBIO
# ============================================================

# I droni devono partire così:
#
#        Drone 0  →                ←  Drone 1
#                         1 m
#
# Entrambi percorrono una semicirconferenza alla propria destra.
#
# Le coordinate sono relative:
#   +X = avanti, verso l'altro drone
#   -Y = destra del drone
#   +Y = sinistra del drone
#   Z  = 0, quindi altezza invariata
#
# La somma dei quattro movimenti è:
#   X totale = +1 metro
#   Y totale = 0 metri
#
# Ogni drone termina quindi nel punto iniziale dell'altro.

SWAP_SEQUENCE = [
    (+0.146, -0.354, 0.0, SEGMENT_TIME),
    (+0.354, -0.146, 0.0, SEGMENT_TIME),
    (+0.354, +0.146, 0.0, SEGMENT_TIME),
    (+0.146, +0.354, 0.0, SEGMENT_TIME),
]

SEQUENCE_ARGS = {
    URIS[0]: [SWAP_SEQUENCE],
    URIS[1]: [SWAP_SEQUENCE],
}


# ============================================================
# LED
# ============================================================

def activate_led_bit_mask(scf: SyncCrazyflie) -> None:
    scf.cf.param.set_value('led.bitmask', '255')


def deactivate_led_bit_mask(scf: SyncCrazyflie) -> None:
    scf.cf.param.set_value('led.bitmask', '0')


def light_check(scf: SyncCrazyflie) -> None:
    print(f'Controllo LED: {scf.cf.link_uri}')

    activate_led_bit_mask(scf)
    time.sleep(2.0)

    deactivate_led_bit_mask(scf)
    time.sleep(1.0)


# ============================================================
# CONTROLLO FLOW DECK
# ============================================================

def check_flow_deck(scf: SyncCrazyflie) -> None:
    uri = scf.cf.link_uri

    # Aspetta che tutti i parametri siano disponibili
    scf.wait_for_params()

    try:
        value = scf.cf.param.get_value('deck.bcFlow2')
    except Exception as error:
        raise RuntimeError(
            f'Impossibile leggere deck.bcFlow2 su {uri}: {error}'
        ) from error

    print(f'{uri}: deck.bcFlow2 = {value}')

    if value is None or int(value) == 0:
        raise RuntimeError(
            f'Flow Deck non rilevato sul Crazyflie {uri}. '
            'Spegni il drone e controlla il collegamento del deck.'
        )


# ============================================================
# RESET DELL'ESTIMATORE CON TIMEOUT
# ============================================================

def reset_estimator_with_timeout(
    scf: SyncCrazyflie,
    timeout_s: float = ESTIMATOR_TIMEOUT,
) -> None:
    cf = scf.cf
    uri = cf.link_uri

    print(f'\nReset estimatore: {uri}')

    # Rimuove eventuali vecchi blocchi di log rimasti nel firmware
    cf.log.reset()
    time.sleep(0.5)

    # Reset del filtro Kalman
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    log_config = LogConfig(
        name='Kalman Variance',
        period_in_ms=500,
    )

    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    history_x = deque(maxlen=10)
    history_y = deque(maxlen=10)
    history_z = deque(maxlen=10)

    stable_event = Event()

    last_variations = {
        'x': None,
        'y': None,
        'z': None,
    }

    threshold = 0.001

    def log_callback(timestamp, data, logconf) -> None:
        history_x.append(data['kalman.varPX'])
        history_y.append(data['kalman.varPY'])
        history_z.append(data['kalman.varPZ'])

        if len(history_x) < history_x.maxlen:
            return

        variation_x = max(history_x) - min(history_x)
        variation_y = max(history_y) - min(history_y)
        variation_z = max(history_z) - min(history_z)

        last_variations['x'] = variation_x
        last_variations['y'] = variation_y
        last_variations['z'] = variation_z

        print(
            f'{uri}: '
            f'DeltaX={variation_x:.6f}, '
            f'DeltaY={variation_y:.6f}, '
            f'DeltaZ={variation_z:.6f}'
        )

        if (
            variation_x < threshold
            and variation_y < threshold
            and variation_z < threshold
        ):
            stable_event.set()

    cf.log.add_config(log_config)
    log_config.data_received_cb.add_callback(log_callback)

    if not log_config.valid:
        raise RuntimeError(
            f'Configurazione di logging Kalman non valida per {uri}'
        )

    log_config.start()

    try:
        estimator_is_stable = stable_event.wait(timeout=timeout_s)

        if not estimator_is_stable:
            raise RuntimeError(
                f'Estimator non stabilizzato entro {timeout_s:.0f} secondi '
                f'per {uri}. Ultime variazioni: {last_variations}'
            )

        print(f'Estimatore stabile: {uri}')

    finally:
        try:
            log_config.stop()
            time.sleep(0.1)
        except Exception:
            pass

        try:
            log_config.delete()
            time.sleep(0.1)
        except Exception:
            pass


# ============================================================
# ARMAMENTO, DECOLLO E ATTERRAGGIO
# ============================================================

def arm(scf: SyncCrazyflie) -> None:
    print(f'Armamento: {scf.cf.link_uri}')

    scf.cf.supervisor.send_arming_request(True)
    time.sleep(1.0)


def take_off(scf: SyncCrazyflie) -> None:
    uri = scf.cf.link_uri
    commander = scf.cf.high_level_commander

    print(f'Decollo: {uri}')

    commander.takeoff(
        absolute_height_m=TAKEOFF_HEIGHT,
        duration_s=TAKEOFF_TIME,
        yaw=None,
    )

    # Margine per evitare di sovrapporre il comando successivo
    time.sleep(TAKEOFF_TIME + 0.5)


def land(scf: SyncCrazyflie) -> None:
    uri = scf.cf.link_uri
    commander = scf.cf.high_level_commander

    print(f'Atterraggio: {uri}')

    commander.land(
        absolute_height_m=0.0,
        duration_s=LANDING_TIME,
        yaw=None,
    )

    time.sleep(LANDING_TIME + 0.5)
    commander.stop()

    print(f'Atterraggio completato: {uri}')


# ============================================================
# SCAMBIO DI POSIZIONE
# ============================================================

def run_sequence(
    scf: SyncCrazyflie,
    sequence,
) -> None:
    cf = scf.cf
    commander = cf.high_level_commander

    for index, (x, y, z, duration) in enumerate(sequence, start=1):
        print(
            f'{cf.link_uri} - tratto {index}/{len(sequence)}: '
            f'X={x:+.3f}, Y={y:+.3f}, Z={z:+.3f}'
        )

        commander.go_to(
            x=x,
            y=y,
            z=z,
            yaw=0.0,
            duration_s=duration,
            relative=True,
        )

        # go_to() non blocca Python
        time.sleep(duration + 0.3)

    print(f'Scambio completato: {cf.link_uri}')


# ============================================================
# ROTAZIONE FINALE DI 180 GRADI
# ============================================================

def turn_around(scf: SyncCrazyflie) -> None:
    uri = scf.cf.link_uri
    commander = scf.cf.high_level_commander

    print(f'Rotazione di 180 gradi: {uri}')

    commander.go_to(
        x=0.0,
        y=0.0,
        z=0.0,
        yaw=math.pi,
        duration_s=TURN_TIME,
        relative=True,
    )

    time.sleep(TURN_TIME + 0.5)

    print(f'Rotazione completata: {uri}')


# ============================================================
# PROCEDURE DI SICUREZZA
# ============================================================

def emergency_land(scf: SyncCrazyflie) -> None:
    commander = scf.cf.high_level_commander

    print(f'Atterraggio di emergenza: {scf.cf.link_uri}')

    try:
        commander.land(
            absolute_height_m=0.0,
            duration_s=2.0,
            yaw=None,
        )
        time.sleep(2.5)
    finally:
        try:
            commander.stop()
        except Exception:
            pass


def stop_motors(scf: SyncCrazyflie) -> None:
    try:
        scf.cf.high_level_commander.stop()
    except Exception:
        pass


# ============================================================
# MAIN
# ============================================================

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(
        rw_cache=str(CACHE_DIR)
    )

    print(f'Cache utilizzata: {CACHE_DIR}')
    print('Connessione ai Crazyflie...')

    with Swarm(URIS, factory=factory) as swarm:
        print('Connesso a entrambi i Crazyflie')

        flight_started = False

        try:
            # Controlla che entrambi abbiano il Flow Deck
            swarm.parallel_safe(check_flow_deck)
            print('Flow Deck rilevati su entrambi i droni')

            # Controllo visivo dei LED
            swarm.parallel_safe(light_check)
            print('Controllo LED completato')

            print(
                '\nLascia entrambi i droni completamente immobili '
                'durante il reset degli estimatori.'
            )

            # Eseguiti uno alla volta per identificare chiaramente
            # quale drone non riesce a stabilizzare l'estimatore
            swarm.sequential(reset_estimator_with_timeout)
            print('\nEstimatori stabilizzati')

            swarm.parallel_safe(arm)

            # Da questo punto almeno un drone potrebbe essere in volo
            flight_started = True

            swarm.parallel_safe(take_off)
            print('Decollo completato')

            # Entrambi percorrono la semicirconferenza alla propria destra
            swarm.parallel_safe(
                run_sequence,
                args_dict=SEQUENCE_ARGS,
            )

            # Ruotano di 180° nella posizione di arrivo
            swarm.parallel_safe(turn_around)

            # Atterrano mantenendo il nuovo orientamento
            swarm.parallel_safe(land)

            flight_started = False

            print('\nSequenza completata correttamente')

        except KeyboardInterrupt:
            print('\nInterruzione da tastiera ricevuta')

            if flight_started:
                swarm.parallel(emergency_land)
            else:
                swarm.parallel(stop_motors)

        except Exception as error:
            print(f'\nERRORE: {error}')

            if flight_started:
                print('Tentativo di atterraggio di emergenza...')
                swarm.parallel(emergency_land)
            else:
                swarm.parallel(stop_motors)

            raise