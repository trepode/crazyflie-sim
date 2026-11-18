
"""ORCA 2D per il test a due Crazyflie (ricostruzione compatibile).

Questa e' una nuova implementazione del metodo ORCA/RVO2, NON una copia
verificata del vecchio orca_lib.py usato nel volo reale.

Le funzioni pubbliche mantengono l'interfaccia richiesta da real_orca.py:
    compute_orca_constraint(p_a, v_a, p_b, v_b, radius, time_horizon, dt)
    choose_orca_velocity(v_pref, line_point, line_direction, max_speed)

Tutte le posizioni e velocita' devono appartenere ALLO STESSO sistema di
coordinate 2D (nel codice reale: piano Motive X-Z; nella simulazione: mondo X-Y).

Questa libreria gestisce UN solo vincolo ORCA alla volta, dunque e' progettata
per il caso di due droni senza ostacoli. Non applica vincoli di accelerazione,
ritardo del controller, o disturbi: non offre garanzie di sicurezza sul volo reale.
"""

import math

_EPS = 1e-12
_TOL = 1e-9


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _scale(v, k):
    return (v[0] * k, v[1] * k)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _det(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _norm(v):
    return math.hypot(v[0], v[1])


def compute_orca_constraint(p_a, v_a, p_b, v_b, radius, time_horizon, dt):
    """Calcola la retta ORCA causata da B nei confronti di A.

    ``radius`` e' il raggio di CIASCUN drone, non il raggio combinato.
    Restituisce (line_point, line_direction, u), ognuno una coppia (x, y).
    Il vettore u e' la correzione minima della velocita' relativa; A assume
    il 50% della correzione, supponendo che anche B esegua ORCA.

    Regione ammissibile: det(line_direction, v - line_point) >= 0.
    """
    if radius <= 0.0 or time_horizon <= 0.0 or dt <= 0.0:
        raise ValueError("radius, time_horizon e dt devono essere positivi")

    relative_position = _sub(p_b, p_a)  # B - A
    relative_velocity = _sub(v_a, v_b)  # A - B
    dist_sq = _dot(relative_position, relative_position)
    combined_radius = 2.0 * radius
    combined_radius_sq = combined_radius * combined_radius

    if dist_sq > combined_radius_sq:
        # Non sono attualmente sovrapposti: costruisci il velocity obstacle
        # con orizzonte temporale tau.
        inv_tau = 1.0 / time_horizon
        w = _sub(relative_velocity, _scale(relative_position, inv_tau))
        w_sq = _dot(w, w)
        projection = _dot(w, relative_position)

        if projection < 0.0 and projection * projection > combined_radius_sq * w_sq:
            # Proiezione sul semicerchio di cutoff del velocity obstacle.
            w_len = _norm(w)
            if w_len < _EPS:
                # Normalmente irraggiungibile in questo ramo per la condizione
                # stretta, ma evitiamo la divisione per zero.
                unit_w = (-relative_position[0] / math.sqrt(dist_sq),
                          -relative_position[1] / math.sqrt(dist_sq))
            else:
                unit_w = _scale(w, 1.0 / w_len)
            line_direction = (unit_w[1], -unit_w[0])
            u = _scale(unit_w, combined_radius * inv_tau - w_len)
        else:
            # Proiezione su una delle due tangenti del cono VO.
            leg = math.sqrt(max(0.0, dist_sq - combined_radius_sq))
            if _det(relative_position, w) > 0.0:
                line_direction = (
                    (relative_position[0] * leg - relative_position[1] * combined_radius) / dist_sq,
                    (relative_position[0] * combined_radius + relative_position[1] * leg) / dist_sq,
                )
            else:
                line_direction = (
                    -(relative_position[0] * leg + relative_position[1] * combined_radius) / dist_sq,
                    -(-relative_position[0] * combined_radius + relative_position[1] * leg) / dist_sq,
                )
            projection_on_line = _dot(relative_velocity, line_direction)
            u = _sub(_scale(line_direction, projection_on_line), relative_velocity)
    else:
        # Collisione/sovrapposizione gia' presente: il time step definisce
        # la velocita' relativa necessaria per separarsi.
        inv_dt = 1.0 / dt
        w = _sub(relative_velocity, _scale(relative_position, inv_dt))
        w_len = _norm(w)
        if w_len < _EPS:
            if dist_sq > _EPS:
                rlen = math.sqrt(dist_sq)
                unit_w = (-relative_position[0] / rlen,
                          -relative_position[1] / rlen)
            else:
                # Agenti nella stessa identica posizione: direzione arbitraria;
                # situazione da evitare gia' a monte con un safety stop.
                unit_w = (1.0, 0.0)
        else:
            unit_w = _scale(w, 1.0 / w_len)
        line_direction = (unit_w[1], -unit_w[0])
        u = _scale(unit_w, combined_radius * inv_dt - w_len)

    line_point = _add(v_a, _scale(u, 0.5))
    return line_point, line_direction, u


def choose_orca_velocity(v_pref, line_point, line_direction, max_speed):
    """Sceglie la velocita' piu' vicina a v_pref entro UN vincolo ORCA.

    Minimizza ||v-v_pref|| con:
      det(line_direction, v-line_point) >= 0
      ||v|| <= max_speed

    Se regione ammissibile e disco velocita' non si intersecano, solleva
    ValueError: e' preferibile segnalare il problema anziche' restituire
    silenziosamente una velocita' che viola i vincoli.
    """
    if max_speed <= 0.0:
        raise ValueError("max_speed deve essere positivo")
    dir_len = _norm(line_direction)
    if dir_len < _EPS:
        raise ValueError("Vincolo ORCA non valido: direzione nulla")
    direction = _scale(line_direction, 1.0 / dir_len)

    # Minimo senza il vincolo ORCA: proiezione della velocita' preferita
    # sul disco di velocita' consentite.
    pref_len = _norm(v_pref)
    if pref_len <= max_speed:
        candidate = (float(v_pref[0]), float(v_pref[1]))
    else:
        candidate = _scale(v_pref, max_speed / pref_len)

    if _det(direction, _sub(candidate, line_point)) >= -_TOL:
        return candidate

    # Il minimo vincolato si trova sulla retta ORCA. In coordinate 1D:
    # v = line_point + t*direction. Risolvi l'intersezione retta/disco.
    dot_pd = _dot(line_point, direction)
    discr = dot_pd * dot_pd + max_speed * max_speed - _dot(line_point, line_point)
    if discr < -_TOL:
        raise ValueError("ORCA infeasible: semipiano e limite di velocita' disgiunti")

    s = math.sqrt(max(0.0, discr))
    t_min, t_max = -dot_pd - s, -dot_pd + s
    t_preferred = _dot(_sub(v_pref, line_point), direction)
    t = max(t_min, min(t_max, t_preferred))
    return _add(line_point, _scale(direction, t))
