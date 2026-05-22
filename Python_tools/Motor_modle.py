import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
from typing import  Tuple

# ============================================================
# Datasheet inputs for 17HE15-1504S
# ============================================================
I_rated  = 1.5         # A
R        = 0.5         # ohm
L        = 1.5e-3      # H
J        = 0.08275     # kg·m²
Kt       = 1.5         # V/Rad
b        = 0.001       # NM/Rad  
V_supply = 12          #V

s = ctrl.TransferFunction.s

# ============================================================
# Plant: Voltage → Speed → Position
# ============================================================

# Electrical dynamics (voltage → current)
G_elec = 1 / (L*s + R)

# Torque constant
G_torque = Kt

# Mechanical dynamics (torque → speed)
G_mech = 1 / (J*s + b)

# Full motor (voltage → speed)
G_motor = G_elec * G_torque * G_mech

print(f"Motor Transfer Function (Voltage -> Speed):\n{G_motor}\n\n")

# ============================================================
# HELPER: Make PID with filtered derivative
# ============================================================
def make_pid(Kp, Ki, Kd, fc_d, zeta=0.707):
    wn = 2 * np.pi * fc_d

    # 2nd-order LPF:
    #        wn^2
    # ---------------------
    # s^2 + 2*zeta*wn*s + wn^2
    d_lpf = (wn**2) / (s**2 + 2*zeta*wn*s + wn**2)

    return Kp + Ki/s + Kd * s * d_lpf

def make_PIFD_zeros(a, b, K, fc_d):
    """
    PID from desired zero locations.

    Zeros:
        s = -a
        s = -b

    Gain:
        K = derivative gain before filtering
    """

    Kd = K
    Kp = K * (a + b)
    Ki = K * a * b

    print(f"Calculated PID gains: Kp={Kp:.4f}, Ki={Ki:.4f}, Kd={Kd:.4f}")
    return make_pid(Kp, Ki, Kd, fc_d)

# =========================================================
# DISCRETIZATION
# =========================================================
def discretize_system(Gv_s:ctrl.TransferFunction, 
                      Gp_s:ctrl.TransferFunction, 
                      Cs:ctrl.TransferFunction, 
                      Ts: float) -> Tuple[ctrl.TransferFunction]:
    """ The 

    Args:
        Gv_s (ctrl.TransferFunction): Plant
        Gp_s (ctrl.TransferFunction): _description_
        Cs (ctrl.TransferFunction): _description_
        Ts (float): _description_

    Returns:
        Tuple[ctrl.TransferFunction]: _description_
    """

    Gv_z = ctrl.c2d(Gv_s, Ts, method='zoh')       # plant (correct)
    Gp_z = ctrl.c2d(Gp_s, Ts, method='zoh')       # optional
    Cz   = ctrl.c2d(Cs,   Ts, method='tustin')    # controller

    return Gv_z, Gp_z, Cz

# =========================================================
# CLOSED LOOP
# =========================================================
def closed_loop(Cz:ctrl.TransferFunction, Gv_z:ctrl.TransferFunction)->ctrl.TransferFunction:
    """Position loop: PID(angle error) -> voltage -> velocity -> integrated externally

    Args:
        Cz (ctrl.TransferFunction): _description_
        Gv_z (ctrl.TransferFunction): _description_

    Returns:
        ctrl.TransferFunction: _description_
    """

    return ctrl.feedback(Cz * Gv_z, 1)

# =========================================================
# CLOSED LOOP SIGNALS
# =========================================================
def closed_loop_signals(Cz: ctrl.TransferFunction,
                        Gp_z: ctrl.TransferFunction) -> tuple[
                            ctrl.TransferFunction,
                            ctrl.TransferFunction,
                            ctrl.TransferFunction
                        ]:
    """ returns transfer functions for internal signals in closed-loop system

    Args:
        Cz (ctrl.TransferFunction): controller transfer function
        Gp_z (ctrl.TransferFunction): plant transfer function

    Returns:
        tuple[ ctrl.TransferFunction, ctrl.TransferFunction, ctrl.TransferFunction ]: These are the transfer functions for:
            - E_over_R: error transfer function
            - U_over_R: controller output transfer function
            - Y_over_R: plant output transfer function
    """

    L = ctrl.minreal(Cz * Gp_z, verbose=False)

    one = ctrl.TransferFunction([1.0], [1.0], Cz.dt)

    E_over_R = ctrl.minreal(ctrl.feedback(one, L), verbose=False)
    U_over_R = ctrl.minreal(Cz * E_over_R, verbose=False)
    Y_over_R = ctrl.minreal(Gp_z * U_over_R, verbose=False)

    return E_over_R, U_over_R, Y_over_R

# ============================================================
# HELPER: Print Step Response and Stability Metrics
# ============================================================
def print_loop_info(loop_name, controller: ctrl.TransferFunction, plant: ctrl.TransferFunction, system: ctrl.TransferFunction, Ts: float, sim_time=1.0, setpoint=1.0, max_cmd = 1):
    """ Prints

    Args:
        loop_name (str): controller name for printing
        controller (ctrl.TransferFunction): controller transfer function
        plant (ctrl.TransferFunction): plant transfer function
        system (ctrl.TransferFunction): open-loop transfer function (controller * plant)
        Ts (float): sampling time for simulation
        sim_time (float, optional): simulation time. Defaults to 1.0.
        setpoint (float, optional): setpoint value. Defaults to 1.0.
    """
    gm, pm, wg, wp = ctrl.margin(system)

    print(f"\n================ {loop_name} =================\n")
    print(f"Gain Margin:        {gm}")
    print(f"Gain Margin @:      {wg/(2*np.pi):.2f} Hz")
    print(f"Phase Margin:       {pm:.1f} deg")
    print(f"Phase Margin @:     {wp/(2*np.pi):.2f} Hz")

    t = np.arange(0, sim_time, Ts)
    r = np.ones_like(t) * setpoint

    E_over_R, U_over_R, Y_over_R = closed_loop_signals(controller, plant)

    # Simulate internal signals
    _, e = ctrl.forced_response(E_over_R, T=t, U=r)
    _, u = ctrl.forced_response(U_over_R, T=t, U=r)
    _, y = ctrl.forced_response(Y_over_R, T=t, U=r)

    L = ctrl.minreal(controller * plant, verbose=False)
    CL = ctrl.minreal(ctrl.feedback(L, 1), verbose=False)

    poles = ctrl.poles(CL)

    print("\nClosed-loop Poles:", poles)

    pole_mags = np.abs(poles)
    print("Pole magnitudes:", pole_mags)

    tol = 1e-6

    if np.any(pole_mags > 1.0 + tol):
        print("Warning: Unstable discrete poles detected!")
    elif np.any(np.isclose(pole_mags, 1.0, atol=tol)):
        print("Warning: Marginal pole near unit circle detected.")
    else:
        print("Discrete closed-loop is stable.")


    try:
        info = ctrl.step_info(
            setpoint * Y_over_R,
            T=t,
            yfinal=setpoint
        )

        print("\nPlant Output Step Info:")
        for k, v in info.items():
            if isinstance(v, (float, int, np.floating)):
                print(f"{k}: {v:.6f}")
            else:
                print(f"{k}: {v}")

    except Exception as err:
        print(f"\nStep info failed: {err}")

    # One window, multiple views
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    axs[0].plot(t, r, label="setpoint r")
    axs[0].plot(t, y, label="plant output y")
    axs[0].set_ylabel("Y(t) output")
    axs[0].set_xlabel("Time (s)")
    axs[0].grid(True)
    axs[0].legend()

    # Voltage plot
    axs[1].plot(t, u, label="controller output u")

    axs[1].plot(t,  np.ones_like(t) * max_cmd,
                label="+ limit", linestyle="--")

    axs[1].plot(t, -np.ones_like(t) * max_cmd,
                label="- limit", linestyle="--")

    # Regions where output exceeds supply limits
    over_pos = u > max_cmd
    over_neg = u < -max_cmd

    # Mark actual points that exceed limits
    axs[1].scatter(t[over_pos], u[over_pos], s=12, label="positive saturation violation")
    axs[1].scatter(t[over_neg], u[over_neg], s=12, label="negative saturation violation")

    # Print max violation info
    u_max = np.max(u)
    u_min = np.min(u)

    axs[1].set_ylabel("U(t) cmd")
    axs[1].set_xlabel("Time (s)")
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(t, e, label="error e")
    axs[2].set_ylabel("E(t) Error")
    axs[2].set_xlabel("Time (s)")
    axs[2].grid(True)
    axs[2].legend()

    fig.suptitle(loop_name)
    plt.tight_layout()
    plt.show()
    

# ============================================================
# LOOP 1: VELOCITY LOOP  (inner loop)
# ============================================================
def design_velocity_loop():
    """ Designs the velocity loop controller and returns the controller, closed-loop transfer function, and open-loop transfer function.

    Returns:
        _type_: tuple[ ctrl.TransferFunction, ctrl.TransferFunction, ctrl.TransferFunction ]: These are the controller transfer function, closed-loop transfer function, and open-loop transfer function for the velocity loop.
    """
    G_velocity = G_motor   # plant: voltage → speed
    
    Kp = 1.0751
    Ki = 0.0125
    Kd = 0.0025
    fc_d = 50
    zeta = 1
    # ------------------

    C_vel = make_pid(Kp, Ki, Kd, fc_d, zeta)

    L_vel = C_vel * G_velocity
    CL_vel = ctrl.feedback(L_vel, 1)

    return G_velocity, C_vel, CL_vel

# ============================================================
# LOOP 2: POSITION LOOP (outer loop)
# ============================================================
def design_position_loop(inner_CL):
    """ Designs the position loop controller based on the inner velocity loop closed-loop transfer function.

    Args:
        inner_CL (_type_): 

    Returns:
        _type_: _description_
    """
    G_position = inner_CL * (1/s)   # speed → position

    # --- tune these ---
    Kp = 1.8
    Ki = 0.0
    Kd = 0.002
    fc_d = 10
    zeta = 1
    # ------------------

    C_pos = make_pid(Kp, Ki, Kd, fc_d, zeta)

    L_pos = C_pos * G_position
    CL_pos = ctrl.feedback(L_pos, 1)

    return G_position, C_pos, CL_pos



# =========================================================
# EXPORT (C)
# =========================================================
def export_tf(sys: ctrl.TransferFunction, name: str = "TF") -> None:
    """
    Export a discrete transfer function as normalized C coefficient arrays.

    Difference equation form:

        y[k] = b0*x[k] + b1*x[k-1] + ...
             - a1*y[k-1] - a2*y[k-2] - ...

    den[0] is normalized to 1.0.
    """

    num = np.asarray(sys.num[0][0], dtype=float)
    den = np.asarray(sys.den[0][0], dtype=float)

    if den[0] == 0.0:
        raise ValueError(f"{name}: denominator leading coefficient is zero")

    num = num / den[0]
    den = den / den[0]

    def fmt(arr):
        return "{ " + ", ".join([f"{x:.8f}f" for x in arr]) + " }"

    safe_name = (
        name.upper()
            .replace(" ", "_")
            .replace("-", "_")
            .replace("(", "")
            .replace(")", "")
    )

    print(f"\n// ===== {name} =====")
    print(f"#define {safe_name}_NUM_LEN {len(num)}")
    print(f"#define {safe_name}_DEN_LEN {len(den)}")
    print(f"static const float {safe_name}_NUM[{len(num)}] = {fmt(num)};")
    print(f"static const float {safe_name}_DEN[{len(den)}] = {fmt(den)};")



# ============================================================
# DISCRETIZE BOTH CONTROLLERS
# ============================================================
vel_Ts = 1/1000  # 1000 Hz control loop
pos_Ts = 1/100  # 100 Hz control loop

# ============================================================
# CALL BOTH LOOPS
# ============================================================
G_velocity, C_vel, CL_vel  = design_velocity_loop()

G_position, C_pos, CL_pos = design_position_loop(CL_vel)

poles: np.ndarray = ctrl.poles(G_position)
zeros: np.ndarray = ctrl.zeros(G_position)

print("G_position poles:", poles)
print("G_position zeros:", zeros)


G_velocity_z = ctrl.c2d(G_velocity, vel_Ts, method='zoh') # plant 
C_vel_z  = ctrl.c2d(C_vel, vel_Ts, method='tustin')       # controller

G_pos_z = ctrl.c2d(G_position, pos_Ts, method='zoh')     # plant 
C_pos_z  = ctrl.c2d(C_pos, pos_Ts, method='tustin')       # controller

CL_vel_z = closed_loop(C_vel_z, G_velocity_z)
CL_pos_z = closed_loop(C_pos_z, G_pos_z)



export_tf(C_vel_z,      "VELOCITY controller")
export_tf(G_velocity_z, "VELOCITY plant")

export_tf(C_pos_z,      "POSITION controller")
export_tf(G_pos_z,      "POSITION plant")

# ============================================================
# step response of discretized position loop
# ============================================================

#print_loop_info("DISCRETIZED VELCITY LOOP", C_vel_z, G_velocity_z, C_vel_z*G_velocity_z, vel_Ts, sim_time=1.0, setpoint=2*np.pi, max_cmd=V_supply)

# print_loop_info("DISCRETIZED POSITION LOOP", C_pos_z, G_pos_z, C_pos_z*G_pos_z, pos_Ts, sim_time=5.0, setpoint=np.pi, max_cmd = 2*np.pi)





