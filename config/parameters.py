# config/parameters.py

import numpy as np

# ============================================================
# 1. PARAMETER ROBOT (Differential Drive)
# ============================================================

# Jari-jari roda (meter)
WHEEL_RADIUS = 0.05

# Jarak antar roda (wheelbase) (meter)
WHEEL_BASE = 0.30

# Kecepatan linear maksimum (m/s)
MAX_LINEAR_VEL = 0.8

# Kecepatan sudut maksimum (rad/s)
MAX_ANGULAR_VEL = 1.5


# ============================================================
# 2. PARAMETER SIMULASI
# ============================================================

# Time step simulasi (detik)
DT = 0.1

# Waktu simulasi maksimum (detik)
MAX_SIM_TIME = 300.0

# Jarak toleransi ke goal (meter)
GOAL_TOLERANCE = 0.5


# ============================================================
# 3. PARAMETER PURE PURSUIT CONTROLLER
# ============================================================

# Look-ahead distance (meter)
LOOKAHEAD_DISTANCE = 1.1
# Catatan:
# - kecil  → akurat tapi osilasi
# - besar → halus tapi potong tikungan

# Kecepatan linear konstan (m/s)
PP_LINEAR_VEL = 0.6


# ============================================================
# 4. PARAMETER STANLEY CONTROLLER
# ============================================================

# Gain cross-track error
STANLEY_K = 1.2

# Gain heading error
STANLEY_K_HEADING = 1.0

# Kecepatan referensi Stanley (m/s)
STANLEY_LINEAR_VEL = 0.6


# ============================================================
# 5. PARAMETER ERROR & LOGGING
# ============================================================

# Apakah error disimpan untuk plotting
SAVE_ERROR_HISTORY = True


# ============================================================
# 6. PARAMETER VISUALISASI
# ============================================================

# Interval animasi (ms)
ANIMATION_INTERVAL = 100

# Ukuran robot saat divisualisasikan
ROBOT_RADIUS = 0.3

# Tampilkan look-ahead point (Pure Pursuit)
SHOW_LOOKAHEAD = True


# ============================================================
# 7. WARNA VISUALISASI
# ============================================================

COLOR_REFERENCE_PATH = "blue"
COLOR_ROBOT_PATH = "red"
COLOR_ROBOT = "black"
COLOR_LOOKAHEAD = "green"

# ============================================================
# 8. PARAMETER TERPADU (UNTUK SIMULATOR)
# ============================================================

SIMULATION_PARAMS = {
    "dt": DT,
    "max_time": MAX_SIM_TIME,
    "target_speed": PP_LINEAR_VEL,   # default speed
    "goal_tolerance": GOAL_TOLERANCE
}

# ============================================================
# 9. PARAMETER TERPADU UNTUK MAIN & TRACKING
# ============================================================

TRACKING_PARAMS = {
    # Controller yang dipakai: "pure_pursuit" atau "stanley"
    "controller": "pure_pursuit",

    "pure_pursuit": {
        "lookahead_distance": LOOKAHEAD_DISTANCE,
        "linear_velocity": PP_LINEAR_VEL
    },

    "stanley": {
        "k": STANLEY_K,
        "k_heading": STANLEY_K_HEADING,
        "linear_velocity": STANLEY_LINEAR_VEL
    }
}

