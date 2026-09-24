"""Bridge configuration.

EXPORT_NAMES must list the CarSim export variables in EXACTLY the order they
are defined in the .sim run (Import/Export tab). The bridge looks them up by
name, so extra variables (lateral error, station, ...) can sit anywhere.

Required : Xo Yo Zo Yaw Pitch Roll Steer_L1 Steer_R1
Recommended (better sync):
           Vx Vy AVx AVy AVz      -> exact velocity / IMU instead of finite differences
           AVy_L1 AVy_R1 AVy_L2 AVy_R2 -> true wheel spin (slip visible) instead of Vx/R
           Steer_L2 Steer_R2      -> rear-wheel steer (4WS)
           Steer_SW Throttle GearStat -> reported through vehicle.get_control()
           Jnc_L1 Jnc_R1 Jnc_L2 Jnc_R2 -> suspension travel (wheel up/down vs body)
See docs/CarSim_export_variables.md for units and meaning.
"""

# Order must match the .sim file. Edit this to your run.
EXPORT_NAMES = [
    "Xo", "Yo", "Zo", "Yaw", "Pitch", "Roll",
    "Vx", "Vy", "AVx", "AVy", "AVz",
    "Steer_SW", "Steer_L1", "Steer_R1", "Steer_L2", "Steer_R2",
    "AVy_L1", "AVy_R1", "AVy_L2", "AVy_R2",
    "Throttle", "GearStat",
    "Jnc_L1", "Jnc_R1", "Jnc_L2", "Jnc_R2",
]

# Units the VS solver uses for the exports above (CarSim "user units").
# Change to "rad" / "m/s" / "rad/s" if your run exports internal SI units.
UNITS = {
    "angle": "deg",        # Yaw, Pitch, Roll, Steer_*
    "speed": "km/h",       # Vx, Vy
    "rate": "deg/s",       # AVx, AVy, AVz (body angular rates)
    "wheel_spin": "rpm",   # AVy_L1 ...
    "jounce": "mm",        # Jnc_L1 ... (+ = compression)
}

# Where the CarSim reference point (Xo, Yo, Zo) sits on the CARLA vehicle,
# in CARLA vehicle-local meters (x fwd, y right, z up; origin = actor origin).
#   "front_axle" : front axle center at ground level (CarSim sprung-mass origin)
#   (x, y, z)    : explicit offset
CARSIM_REFERENCE_POINT = "front_axle"

# Vertical placement:
#   "carsim" : use Zo from CarSim (CarSim road must match CARLA terrain)
#   "ground" : keep CarSim x/y/attitude but snap height to CARLA road surface
Z_MODE = "carsim"

# Unreal wheel pitch sign for forward rolling. tests/check_wheels.py on 0.9.16:
# set_wheel_pitch_angle(+30) moves the top of the wheel BACKWARDS, so forward
# rolling needs a negative angle.
WHEEL_SPIN_SIGN = -1.0

# Steering wheel angle that maps to steer = +-1.0 in get_control().
STEERING_WHEEL_MAX_DEG = 540.0

# --driver pid: reuse SimplePathFollower from python_carsim_env. Name the
# export variables it reads (your .sim must export a lateral path error).
PID_LATERAL_ERROR = "LatErr"   # rename to your lateral-error export
PID_SPEED = "Vx"               # km/h
PID_TARGET_SPEED = 50.0        # km/h
