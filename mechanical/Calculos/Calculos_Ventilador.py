import sys
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

"""
Analisis de Configuracion de Resistencias para Aplicacion de Calentamiento
==========================================================================
Modelo rectangular equivalente (misma area superficial que el cilindro
original). Cada tubo tiene 4 aletas de 15mm desde el centro.

Config A: 2 resistencias por fila -> NL=3 filas, NT=4 tubos/fila
Config B: 1 resistencia por fila  -> NL=6 filas, NT=2 tubos/fila
Ambas: 6 resistencias U (12 tubos), Q_total = 6000 W

S_T fijo = 3 cm, S_L variable desde 4 cm (+1 cm)

Referencia: Cengel, "Fundamentals of Thermal-Fluid Sciences", 6a ed.,
            Cap. 3 (Aletas) y Cap. 7 (Conveccion forzada externa)
"""

import numpy as np
from CoolProp.CoolProp import PropsSI
from scipy.constants import *
from scipy.optimize import brentq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ============================================================
# 1. PARAMETROS DEL SISTEMA
# ============================================================

# --- Geometria de la resistencia (cilindrica original) ---
D_cyl = (5 / 16) * inch              # Diametro cilindrico original [m]
L_tube = 45 * centi                   # Longitud de cada tubo (pata de la U) [m]
Q_per_res = 1000                      # Potencia por resistencia [W]
N_resistances = 6                     # Numero total de resistencias U
Q_total = N_resistances * Q_per_res   # Potencia total [W]

# --- Seccion rectangular equivalente ---
# Misma area superficial (perimetro * L) que el cilindro
# Perimetro cilindro: p_cyl = pi * D_cyl
# Para seccion cuadrada equivalente: 4*a = pi*D_cyl -> a = pi*D_cyl/4
p_cyl = np.pi * D_cyl
a_rect = p_cyl / 4                    # Lado del cuadrado equivalente [m]
A_c_rect = a_rect ** 2                # Area de seccion transversal [m2]
p_rect = 4 * a_rect                   # Perimetro de la seccion rectangular [m]

# Para el Re y Nu del banco de tubos, el diametro "equivalente" sigue
# siendo D (diametro del cilindro que tiene el mismo perimetro)
D = D_cyl  # Diametro equivalente para correlaciones de banco de tubos

# --- Parametros de las aletas (4 aletas por tubo, rectangulares) ---
# Cada aleta sale 15mm desde el CENTRO del tubo en las 4 direcciones
# Altura de aleta desde la superficie = 15mm - D/2
fin_from_center = 15 * milli                          # 15 mm desde el centro
fin_height = fin_from_center - D_cyl / 2              # Altura neta de aleta [m]
N_fins_per_tube_cross = 4                             # 4 aletas por seccion
fin_thickness = 1 * milli                             # Espesor de aleta [m]
fin_gap = 2 * milli                                   # Separacion entre planos de aletas [m]
k_fin = 15                                            # Conductividad termica [W/(m*K)]

# --- Zonas sin aletas en los extremos del tubo ---
bare_bottom = 45 * milli                              # Zona inferior sin aletas [m]
bare_top = 15 * milli                                 # Zona superior sin aletas [m]
L_finned = L_tube - bare_bottom - bare_top            # Longitud aletada [m]

fin_pitch = fin_thickness + fin_gap                   # Paso entre aletas (3 mm) [m]
N_fin_planes = int(L_finned / fin_pitch)              # Num. de planos de aletas en zona aletada
# Total de aletas individuales por tubo = 4 aletas/plano * N_planos
N_fins_total_per_tube = N_fins_per_tube_cross * N_fin_planes
# Longitud sin aletas total = extremos + sobrante
L_bare = bare_bottom + bare_top + (L_finned - N_fin_planes * fin_pitch)

# --- Paso transversal fijo y longitudinal variable ---
ST_fixed = 3 * centi                  # S_T fijo = 3 cm

# --- Condiciones de operacion ---
T_inf = 25.0                          # Temperatura de entrada del aire [C]
T_s = 200.0                           # Temperatura superficial de resistencia [C]
P_atm = 101325.0                      # Presion atmosferica [Pa]


# ============================================================
# 2. PROPIEDADES DEL AIRE (CoolProp)
# ============================================================

def air_properties(T_C):
    """Propiedades del aire a 1 atm para temperatura T en [C]."""
    T_K = T_C + 273.15
    rho = PropsSI('Dmass', 'T', T_K, 'P', P_atm, 'Air')
    mu  = PropsSI('viscosity', 'T', T_K, 'P', P_atm, 'Air')
    nu  = mu / rho
    k   = PropsSI('conductivity', 'T', T_K, 'P', P_atm, 'Air')
    cp  = PropsSI('Cpmass', 'T', T_K, 'P', P_atm, 'Air')
    Pr  = PropsSI('Prandtl', 'T', T_K, 'P', P_atm, 'Air')
    return rho, mu, nu, k, cp, Pr


# ============================================================
# 3. EFICIENCIA Y AREA EFECTIVA DE ALETAS (Cap. 3)
# ============================================================

def fin_efficiency(h):
    """
    Eficiencia de aleta rectangular con punta adiabatica corregida
    (Tabla 3-3 Cengel, Ec. 3-77 con Lc corregida Ec. 3-72).
    eta_fin = tanh(mLc) / (mLc)
    m = sqrt(2h / (k_fin * t))
    Lc = L + t/2
    """
    Lc = fin_height + fin_thickness / 2.0    # Longitud corregida (Ec. 3-72)
    m = np.sqrt(2.0 * h / (k_fin * fin_thickness))  # Tabla 3-3
    mLc = m * Lc
    if mLc < 1e-8:
        return 1.0
    return np.tanh(mLc) / mLc  # Ec. 3-77


def effective_area_per_tube(h):
    """
    Area efectiva por tubo incluyendo 4 aletas rectangulares por seccion
    (Ec. 3-81 Cengel).
    Q_total = h * (A_unfin + eta_fin * A_fin) * (Tb - Tinf)

    Zonas sin aletas: 45mm inferior + 15mm superior.
    Zona aletada: L_finned = L_tube - 45mm - 15mm
    Separacion entre aletas: 2mm, espesor: 1mm
    """
    eta_f = fin_efficiency(h)

    # Area de una aleta individual (un plano, una direccion):
    # Cada aleta: 2 caras de fin_height x fin_thickness
    A_one_fin = 2.0 * fin_height * fin_thickness  # 2 caras

    # Total area de aletas por tubo
    A_fins = N_fins_total_per_tube * A_one_fin

    # Area desnuda = tubo entre planos de aletas + zonas sin aletas
    # Entre aletas en zona aletada: perimetro * N_planos * gap
    A_bare_between_fins = p_rect * N_fin_planes * fin_gap
    # Zonas sin aletas en extremos: perimetro * L_bare
    A_bare_ends = p_rect * L_bare

    A_bare = A_bare_between_fins + A_bare_ends

    # Area efectiva (Ec. 3-81)
    A_eff = A_bare + eta_f * A_fins
    return A_eff, eta_f, A_bare, A_fins


# ============================================================
# 4. CORRELACIONES DE BANCO DE TUBOS (Cap. 7 S7-4 Zukauskas)
# ============================================================

def compute_Vmax(V, ST, SL):
    """Velocidad maxima en banco escalonado (Ec. 7-40 / 7-41)."""
    SD = np.sqrt(SL**2 + (ST / 2.0)**2)  # Ec. 7-38
    if 2.0 * (SD - D) < (ST - D):
        Vmax = ST / (2.0 * (SD - D)) * V  # Ec. 7-41
    else:
        Vmax = ST / (ST - D) * V           # Ec. 7-40
    return Vmax, SD


def nusselt_staggered(Re_D, Pr, Pr_s, ST, SL, NL):
    """Nusselt promedio para banco escalonado (Tabla 7-2 + Tabla 7-3)."""
    if Re_D < 500:
        Nu = 1.04 * Re_D**0.4 * Pr**0.36 * (Pr / Pr_s)**0.25
    elif Re_D < 1000:
        Nu = 0.71 * Re_D**0.5 * Pr**0.36 * (Pr / Pr_s)**0.25
    elif Re_D < 2e5:
        Nu = 0.35 * (ST / SL)**0.2 * Re_D**0.6 * Pr**0.36 * (Pr / Pr_s)**0.25
    else:
        Nu = 0.031 * (ST / SL)**0.2 * Re_D**0.8 * Pr**0.36 * (Pr / Pr_s)**0.25

    # Factor correccion F para NL < 16 (Tabla 7-3, escalonado)
    F_table = {1: 0.64, 2: 0.76, 3: 0.84, 4: 0.89,
               5: 0.93, 7: 0.96, 10: 0.98, 13: 0.99}
    if NL >= 16:
        F = 1.0
    elif NL in F_table:
        F = F_table[NL]
    else:
        keys = sorted(F_table.keys())
        F = 1.0
        for i in range(len(keys) - 1):
            if keys[i] <= NL <= keys[i + 1]:
                t = (NL - keys[i]) / (keys[i + 1] - keys[i])
                F = F_table[keys[i]] + t * (F_table[keys[i + 1]] - F_table[keys[i]])
                break
    return F * Nu


def friction_factor_staggered(Re_D, ST, SL):
    """Factor de friccion aproximado para banco escalonado (Fig. 7-27b)."""
    PT = ST / D
    if Re_D < 10:
        f = 50.0 / Re_D
    elif Re_D < 100:
        f = 10.0 / Re_D**0.5
    elif Re_D < 1000:
        f = 4.0 / Re_D**0.25
    elif Re_D < 2e5:
        f = (0.25 + 0.12 / max(PT - 1.0, 0.1)**1.08) * Re_D**(-0.16)
    else:
        f = (0.044 + 0.08 * max(PT - 1.0, 0.1)**(-0.43)) * Re_D**(-0.12)

    SD = np.sqrt(SL**2 + (ST / 2.0)**2)
    chi = max(1.0, (ST / SD)**0.6) if abs(ST - SD) / max(ST, 1e-6) >= 0.05 else 1.0
    return f, chi


# ============================================================
# 5. ANALISIS COMPLETO DE UNA CONFIGURACION
# ============================================================

def analyze(NL, NT, ST, SL, V, T_s=T_s, T_i=T_inf):
    """Analisis completo de banco de tubos escalonado con iteracion en T_m."""
    N = NL * NT

    T_m = T_i + 5.0
    for _ in range(30):
        rho_m, mu_m, nu_m, k_m, cp_m, Pr_m = air_properties(T_m)
        _, _, _, _, _, Pr_s_val = air_properties(T_s)
        rho_i, _, _, _, _, _ = air_properties(T_i)

        Vmax, SD = compute_Vmax(V, ST, SL)
        Re_D = Vmax * D / nu_m
        Nu_D = nusselt_staggered(Re_D, Pr_m, Pr_s_val, ST, SL, NL)
        h = Nu_D * k_m / D

        A_eff_per_tube, eta_f, A_bare_1, A_fins_1 = effective_area_per_tube(h)
        A_eff_total = N * A_eff_per_tube

        m_dot = rho_i * V * (NT * ST * L_tube)

        exp_arg = -A_eff_total * h / (m_dot * cp_m)
        T_e = T_s - (T_s - T_i) * np.exp(exp_arg)

        T_m_new = (T_i + T_e) / 2.0
        if abs(T_m_new - T_m) < 0.05:
            T_m = T_m_new
            break
        T_m = T_m_new

    # LMTD (Ec. 7-45)
    dT_e = max(T_s - T_e, 0.01)
    dT_i = T_s - T_i
    dT_lm = dT_i if abs(dT_e - dT_i) < 0.01 else (dT_e - dT_i) / np.log(dT_e / dT_i)

    Q = m_dot * cp_m * (T_e - T_i)

    f, chi = friction_factor_staggered(Re_D, ST, SL)
    dP = NL * f * chi * rho_m * Vmax**2 / 2.0

    V_dot = V * (NT * ST * L_tube)
    W_pump = V_dot * dP

    return {
        'NL': NL, 'NT': NT, 'N': N,
        'ST': ST * 100, 'SL': SL * 100, 'SD': SD * 100,
        'V': V, 'Vmax': Vmax,
        'T_m': T_m, 'T_e': T_e,
        'Re_D': Re_D, 'Nu_D': Nu_D, 'h': h,
        'eta_f': eta_f,
        'A_bare': N * A_bare_1, 'A_fins': N * A_fins_1,
        'A_eff': A_eff_total,
        'm_dot': m_dot, 'dT_lm': dT_lm,
        'Q': Q, 'dP': dP,
        'V_dot': V_dot * 3600, 'W_pump': W_pump,
        'f': f, 'chi': chi,
    }


# ============================================================
# 6. ENCONTRAR VELOCIDAD REQUERIDA
# ============================================================

def find_velocity(NL, NT, ST, SL, Q_target, T_s=T_s, V_range=(0.1, 80.0)):
    """Encuentra la velocidad V que produce Q = Q_target."""
    def objective(V):
        r = analyze(NL, NT, ST, SL, V, T_s)
        return r['Q'] - Q_target
    try:
        return brentq(objective, V_range[0], V_range[1], xtol=1e-4, maxiter=100)
    except ValueError:
        return None


# ============================================================
# 7. GRAFICO DE LA MEJOR DISTRIBUCION
# ============================================================

def plot_best_distribution(best_result, best_name, best_SL_cm):
    """
    Dibuja:
    - Vista Frontal: Tubo de 440mm con aletas laterales (izq/der),
      separacion 2mm, espesor 1mm, zona sin aletas inferior 45mm
      y superior 15mm.
    - Vista Superior: Arreglo escalonado mostrando 4 aletas por tubo.
    """
    NL = best_result['NL']
    NT = best_result['NT']
    ST = best_result['ST'] / 100  # cm -> m
    SL = best_SL_cm / 100         # cm -> m

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle(f'Mejor Distribucion: {best_name}\n'
                 f'ST = {ST*100:.0f} cm (fijo), SL = {SL*100:.0f} cm, '
                 f'V = {best_result["V"]:.2f} m/s, dP = {best_result["dP"]:.2f} Pa',
                 fontsize=13, fontweight='bold')

    # ========================================================
    # VISTA FRONTAL: 2 Tubos verticales (patas de la U) con aletas
    # ========================================================
    ax1 = axes[0]
    ax1.set_title('Vista Frontal - 1 Resistencia', fontsize=11)
    ax1.set_xlabel('Ancho [mm]')
    ax1.set_ylabel('Longitud del tubo [mm]')

    # Parametros del dibujo frontal
    tube_length_mm = L_tube * 1000              # Longitud total del tubo [mm]
    tube_width_mm = a_rect * 1000               # Seccion rectangular equiv. [mm]
    bare_bottom_mm = bare_bottom * 1000          # Zona sin aletas inferior [mm]
    bare_top_mm = bare_top * 1000                # Zona sin aletas superior [mm]
    fin_gap_draw = fin_gap * 1000                # Separacion entre aletas [mm]
    fin_t_draw = fin_thickness * 1000            # Espesor aleta [mm]
    fin_h_draw = fin_height * 1000               # Altura neta de aleta [mm]
    tube_sep_mm = ST * 1000                     # Separacion entre tubos = S_T [mm]

    finned_start = bare_bottom_mm
    finned_end = tube_length_mm - bare_top_mm
    fin_pitch_draw = fin_t_draw + fin_gap_draw   # 3 mm

    # Posiciones horizontales de los 2 tubos
    tube_centers_x = [-tube_sep_mm / 2, tube_sep_mm / 2]

    for tx in tube_centers_x:
        # Tubo (rectangulo vertical)
        ax1.add_patch(patches.Rectangle(
            (tx - tube_width_mm / 2, 0), tube_width_mm, tube_length_mm,
            linewidth=1.5, edgecolor='darkred', facecolor='#FFB3B3', zorder=3
        ))

        # Zonas sin aletas (marcadas)
        ax1.add_patch(patches.Rectangle(
            (tx - tube_width_mm / 2, 0), tube_width_mm, bare_bottom_mm,
            linewidth=0, facecolor='salmon', alpha=0.5, zorder=4
        ))
        ax1.add_patch(patches.Rectangle(
            (tx - tube_width_mm / 2, tube_length_mm - bare_top_mm),
            tube_width_mm, bare_top_mm,
            linewidth=0, facecolor='salmon', alpha=0.5, zorder=4
        ))

        # Dibujar aletas laterales (izquierda y derecha)
        y = finned_start
        while y + fin_t_draw <= finned_end:
            ax1.add_patch(patches.Rectangle(
                (tx - tube_width_mm / 2 - fin_h_draw, y),
                fin_h_draw, fin_t_draw,
                linewidth=0.3, edgecolor='#666666', facecolor='#C0C0C0', zorder=2
            ))
            ax1.add_patch(patches.Rectangle(
                (tx + tube_width_mm / 2, y),
                fin_h_draw, fin_t_draw,
                linewidth=0.3, edgecolor='#666666', facecolor='#C0C0C0', zorder=2
            ))
            y += fin_pitch_draw

    # Acotaciones (sobre el tubo derecho)
    tx_r = tube_centers_x[1]
    dim_x = tx_r + tube_width_mm / 2 + fin_h_draw + 4
    ax1.annotate('', xy=(dim_x, 0), xytext=(dim_x, tube_length_mm),
                 arrowprops=dict(arrowstyle='<->', color='black', lw=1))
    ax1.text(dim_x + 1.5, tube_length_mm / 2, f'{tube_length_mm:.0f} mm',
             ha='left', va='center', fontsize=8, rotation=90)

    # Zona inferior sin aletas (sobre tubo izquierdo)
    tx_l = tube_centers_x[0]
    dim_x2 = tx_l - tube_width_mm / 2 - fin_h_draw - 4
    ax1.annotate('', xy=(dim_x2, 0), xytext=(dim_x2, bare_bottom_mm),
                 arrowprops=dict(arrowstyle='<->', color='#CC6600', lw=1))
    ax1.text(dim_x2 - 1.5, bare_bottom_mm / 2, f'{bare_bottom_mm:.0f} mm\n(sin aletas)',
             ha='right', va='center', fontsize=7, color='#CC6600')

    # Zona superior sin aletas
    ax1.annotate('', xy=(dim_x2, finned_end), xytext=(dim_x2, tube_length_mm),
                 arrowprops=dict(arrowstyle='<->', color='#CC6600', lw=1))
    ax1.text(dim_x2 - 1.5, (finned_end + tube_length_mm) / 2,
             f'{bare_top_mm:.0f} mm\n(sin aletas)',
             ha='right', va='center', fontsize=7, color='#CC6600')

    # Acotacion de aleta
    y_acot = finned_start + 8 * fin_pitch_draw
    ax1.annotate('', xy=(tx_r + tube_width_mm/2, y_acot),
                 xytext=(tx_r + tube_width_mm/2 + fin_h_draw, y_acot),
                 arrowprops=dict(arrowstyle='<->', color='blue', lw=0.8))
    ax1.text(tx_r + tube_width_mm/2 + fin_h_draw/2, y_acot + 3,
             f'{fin_h_draw:.1f} mm', ha='center', fontsize=7, color='blue')

    # Separacion entre tubos
    y_sep = tube_length_mm + 5
    ax1.annotate('', xy=(tube_centers_x[0], y_sep),
                 xytext=(tube_centers_x[1], y_sep),
                 arrowprops=dict(arrowstyle='<->', color='green', lw=1.2))
    ax1.text(0, y_sep + 3, f'{tube_sep_mm:.0f} mm', ha='center',
             fontsize=8, color='green', fontweight='bold')

    ax1.annotate(f'sep={fin_gap_draw:.0f}mm, esp={fin_t_draw:.0f}mm',
                 xy=(tx_r + tube_width_mm/2 + fin_h_draw + 1, finned_start + 20),
                 fontsize=6, color='gray')

    half_w = tube_sep_mm / 2 + tube_width_mm / 2 + fin_h_draw
    ax1.set_xlim(-half_w * 1.3, half_w * 1.3)
    ax1.set_ylim(-15, tube_length_mm + 18)
    ax1.grid(True, alpha=0.2)

    # ========================================================
    # VISTA SUPERIOR: Arreglo escalonado con 4 aletas por tubo
    # ========================================================
    ax2 = axes[1]
    ax2.set_title('Vista Superior (planta) - 4 aletas por tubo', fontsize=11)
    ax2.set_xlabel('Posicion transversal [mm]')
    ax2.set_ylabel('Posicion en dir. del flujo [mm]')
    ax2.set_aspect('equal')

    fin_len_mm = (fin_from_center - D_cyl / 2) * 1000  # Longitud de aleta desde superficie
    half_tube = a_rect * 500  # Mitad del tubo en mm

    colors = plt.cm.Set1(np.linspace(0, 0.6, max(NL, 2)))

    for row in range(NL):
        y_center = row * SL * 1000  # mm
        offset = ST * 500 if row % 2 == 1 else 0  # Offset escalonado

        for j in range(NT):
            cx = j * ST * 1000 + offset
            cy = y_center

            # Tubo (seccion cuadrada)
            ax2.add_patch(patches.Rectangle(
                (cx - half_tube, cy - half_tube),
                2 * half_tube, 2 * half_tube,
                linewidth=1.2, edgecolor='darkred', facecolor=colors[row],
                zorder=3
            ))

            # 4 aletas: izquierda, derecha, arriba, abajo
            fin_defs = [
                # (x0, y0, width, height)
                (cx - half_tube - fin_len_mm, cy - fin_thickness*500,
                 fin_len_mm, fin_thickness*1000),  # izquierda
                (cx + half_tube, cy - fin_thickness*500,
                 fin_len_mm, fin_thickness*1000),  # derecha
                (cx - fin_thickness*500, cy + half_tube,
                 fin_thickness*1000, fin_len_mm),  # arriba (dir flujo)
                (cx - fin_thickness*500, cy - half_tube - fin_len_mm,
                 fin_thickness*1000, fin_len_mm),  # abajo (dir flujo)
            ]
            for (fx, fy, fw, fh) in fin_defs:
                ax2.add_patch(patches.Rectangle(
                    (fx, fy), fw, fh,
                    linewidth=0.5, edgecolor='#666666', facecolor='#D0D0D0',
                    alpha=0.8, zorder=2
                ))

        ax2.text(-fin_len_mm - half_tube - 8, y_center, f'Fila {row+1}',
                 ha='right', va='center', fontsize=8, color=colors[row],
                 fontweight='bold')

    # Flecha del flujo
    x_arrow = -fin_len_mm - half_tube - 20
    ax2.annotate('', xy=(x_arrow, (NL - 0.5) * SL * 1000),
                 xytext=(x_arrow, -SL * 300),
                 arrowprops=dict(arrowstyle='->', color='blue', lw=2.5))
    ax2.text(x_arrow - 2, (NL - 1) * SL * 500, 'Flujo',
             rotation=90, ha='right', va='center', color='blue', fontsize=10)

    # Acotacion S_T
    ax2.annotate('', xy=(0, -SL * 200), xytext=(ST * 1000, -SL * 200),
                 arrowprops=dict(arrowstyle='<->', color='green', lw=1.5))
    ax2.text(ST * 500, -SL * 280, f'ST = {ST*100:.0f} cm',
             ha='center', color='green', fontsize=9, fontweight='bold')

    # Acotacion S_L
    if NL > 1:
        x_dim = NT * ST * 1000 + fin_len_mm + 10
        ax2.annotate('', xy=(x_dim, 0), xytext=(x_dim, SL * 1000),
                     arrowprops=dict(arrowstyle='<->', color='purple', lw=1.5))
        ax2.text(x_dim + 2, SL * 500, f'SL = {SL*100:.0f} cm',
                 ha='left', color='purple', fontsize=9, fontweight='bold')

    margin_x = fin_len_mm + half_tube + 30
    margin_y = SL * 400
    ax2.set_xlim(-margin_x, NT * ST * 1000 + margin_x)
    ax2.set_ylim(-margin_y, (NL - 1) * SL * 1000 + margin_y)
    ax2.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig('distribucion_optima.png', dpi=150, bbox_inches='tight')
    print("\n  [OK] Grafico guardado como 'distribucion_optima.png'")


# ============================================================
# 8. ESTUDIO PARAMETRICO
# ============================================================

def main():
    print("=" * 85)
    print("  ANALISIS DE CONFIGURACION DE RESISTENCIAS PARA CALENTAMIENTO")
    print("  Cengel Cap. 3 (Aletas) + Cap. 7 S7-4 (Banco de tubos escalonado)")
    print("=" * 85)

    print(f"\n  --- Geometria ---")
    print(f"  Diametro cilindrico original D = {D_cyl * 1000:.2f} mm")
    print(f"  Seccion rectangular equiv.   a = {a_rect * 1000:.2f} mm (perimetro = pi*D)")
    print(f"  D equivalente (correlaciones)  = {D * 1000:.2f} mm")
    print(f"  Longitud del tubo L_tube       = {L_tube * 100:.1f} cm")
    print(f"  --- Aletas (4 por seccion) ---")
    print(f"  Distancia desde centro         = {fin_from_center * 1000:.1f} mm")
    print(f"  Altura neta de aleta           = {fin_height * 1000:.2f} mm")
    print(f"  Espesor de aleta               = {fin_thickness * 1000:.1f} mm")
    print(f"  Separacion entre planos        = {fin_gap * 1000:.1f} mm")
    print(f"  Zona sin aletas inferior       = {bare_bottom * 1000:.0f} mm")
    print(f"  Zona sin aletas superior       = {bare_top * 1000:.0f} mm")
    print(f"  Longitud aletada               = {L_finned * 1000:.0f} mm")
    print(f"  Planos de aletas por tubo      = {N_fin_planes}")
    print(f"  Total aletas por tubo          = {N_fins_total_per_tube}")
    print(f"  k_fin                          = {k_fin} W/(m*K)")
    print(f"  --- Operacion ---")
    print(f"  Resistencias totales           = {N_resistances}")
    print(f"  Potencia total Q_target        = {Q_total:.0f} W")
    print(f"  T_entrada aire                 = {T_inf:.0f} C")
    print(f"  T_superficie                   = {T_s:.0f} C")
    print(f"  S_T (fijo)                     = {ST_fixed * 100:.0f} cm")

    # Configuraciones
    configs = [
        {'name': 'Config A: 2 res/fila', 'NL': 3, 'NT': 4},
        {'name': 'Config B: 1 res/fila', 'NL': 6, 'NT': 2},
    ]

    # S_L variable desde 4 cm
    SL_values_cm = [4, 5, 6, 7, 8, 9, 10]
    ST = ST_fixed

    results = {c['name']: [] for c in configs}

    for sl_cm in SL_values_cm:
        SL = sl_cm / 100.0

        print(f"\n{'=' * 85}")
        print(f"  S_L = {sl_cm} cm  (S_T fijo = {ST*100:.0f} cm)")
        print(f"{'=' * 85}")

        for cfg in configs:
            NL, NT = cfg['NL'], cfg['NT']
            name = cfg['name']

            V_req = find_velocity(NL, NT, ST, SL, Q_total)

            if V_req is None:
                print(f"\n  {name} (NL={NL}, NT={NT})")
                print(f"    [!] No se encontro velocidad viable en el rango")
                results[name].append(None)
                continue

            r = analyze(NL, NT, ST, SL, V_req)
            results[name].append(r)

            print(f"\n  {name} (NL={NL}, NT={NT})")
            print(f"    V (aproximacion)    = {r['V']:.3f} m/s")
            print(f"    V_max               = {r['Vmax']:.3f} m/s")
            print(f"    Re_D                = {r['Re_D']:.0f}")
            print(f"    Nu_D                = {r['Nu_D']:.2f}")
            print(f"    h                   = {r['h']:.2f} W/(m2*K)")
            print(f"    n_aleta             = {r['eta_f']:.4f}")
            print(f"    A_eff total         = {r['A_eff']:.4f} m2")
            print(f"    T_salida aire       = {r['T_e']:.1f} C")
            print(f"    dT_lm              = {r['dT_lm']:.1f} C")
            print(f"    Q disipado          = {r['Q']:.0f} W")
            print(f"    dP                  = {r['dP']:.2f} Pa")
            print(f"    Caudal              = {r['V_dot']:.1f} m3/h")
            print(f"    Potencia ventilador = {r['W_pump']:.4f} W")

    # ==========================================================
    # TABLA RESUMEN
    # ==========================================================
    print(f"\n\n{'=' * 85}")
    print("  TABLA RESUMEN COMPARATIVA (S_T fijo = 3 cm, S_L variable)")
    print(f"{'=' * 85}")

    hdr = (f"{'S_L[cm]':>7} | {'Config':>16} | {'V[m/s]':>7} | "
           f"{'Re_D':>7} | {'h[W/m2K]':>9} | {'Q[W]':>7} | "
           f"{'dP[Pa]':>7} | {'W_fan[W]':>9}")
    print(hdr)
    print("-" * len(hdr))

    for j, sl_cm in enumerate(SL_values_cm):
        for cfg in configs:
            r = results[cfg['name']][j]
            if r is None:
                continue
            label = "2 res/fila" if cfg['NL'] == 3 else "1 res/fila"
            print(f"{sl_cm:>5} cm | {label:>16} | {r['V']:>7.3f} | "
                  f"{r['Re_D']:>7.0f} | {r['h']:>9.2f} | {r['Q']:>7.0f} | "
                  f"{r['dP']:>7.2f} | {r['W_pump']:>9.4f}")

    # ==========================================================
    # RECOMENDACION
    # ==========================================================
    print(f"\n\n{'=' * 85}")
    print("  RECOMENDACION")
    print(f"{'=' * 85}")

    best = None
    best_name = ""
    best_sl = 0
    for cfg in configs:
        for j, r in enumerate(results[cfg['name']]):
            if r is None:
                continue
            if best is None or r['W_pump'] < best['W_pump']:
                best = r
                best_name = cfg['name']
                best_sl = SL_values_cm[j]

    if best:
        print(f"\n  >> Configuracion optima (min. potencia ventilador):")
        print(f"    {best_name}")
        print(f"    S_T (fijo)               = {ST_fixed*100:.0f} cm")
        print(f"    S_L optimo               = {best_sl} cm")
        print(f"    Velocidad requerida V    = {best['V']:.3f} m/s")
        print(f"    Calor disipado Q         = {best['Q']:.0f} W")
        print(f"    Caida de presion dP      = {best['dP']:.2f} Pa")
        print(f"    Potencia ventilador      = {best['W_pump']:.4f} W")
        print(f"    Caudal volumetrico       = {best['V_dot']:.1f} m3/h")
        print(f"    Temp. salida aire        = {best['T_e']:.1f} C")
        print(f"    Eficiencia de aleta      = {best['eta_f']:.4f}")

    print(f"\n{'=' * 85}")
    print("  SUPUESTOS PRINCIPALES")
    print(f"{'=' * 85}")
    print(f"  1. Aire a 1 atm, entrada a {T_inf:.0f} C")
    print(f"  2. Temperatura superficial T_s = {T_s:.0f} C")
    print(f"  3. Seccion rectangular equivalente (mismo perimetro que cilindro)")
    print(f"  4. 4 aletas rectangulares por seccion, 15mm desde el centro")
    print(f"  5. Eficiencia de aleta: tanh(mLc)/mLc (Tabla 3-3, Ec. 3-77)")
    print(f"  6. Correlaciones de Zukauskas (Tabla 7-2) para NuD escalonado")
    print(f"  7. Factor correccion F (Tabla 7-3) para NL < 16")
    print(f"  8. Factor friccion aprox. de Fig. 7-27b")
    print(f"  9. Propiedades evaluadas a T_m = (T_i + T_e)/2 con CoolProp")
    print()

    # ==========================================================
    # GRAFICO DE LA MEJOR DISTRIBUCION
    # ==========================================================
    if best:
        print("\n  Generando grafico de la distribucion optima...")
        plot_best_distribution(best, best_name, best_sl)


if __name__ == '__main__':
    main()
