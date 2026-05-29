import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
import math
from typing import  Tuple


# ============================================================
# Datasheet inputs for motor
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

s = ctrl.TransferFunction.s

def make_PIFFD_gains(Kp, Ki, Kd, fc_d, zeta=0.707):
    """ Generate a 

    Args:
        Kp (_type_): _description_
        Ki (_type_): _description_
        Kd (_type_): _description_
        fc_d (_type_): _description_
        zeta (float, optional): _description_. Defaults to 0.707.

    Returns:
        _type_: _description_
    """
    wn = 2 * np.pi * fc_d

    # 2nd-order LPF:
    #        wn^2
    # ---------------------
    # s^2 + 2*zeta*wn*s + wn^2
    d_lpf = (wn**2) / (s**2 + 2*zeta*wn*s + wn**2)

    return Kp + Ki/s + Kd * s * d_lpf

def make_PIFD_gains(Kp, Ki, Kd, fc_d): 
    """_summary_

    Args:
        Kp (_type_): _description_
        Ki (_type_): _description_
        Kd (_type_): _description_
        fc_d (_type_): _description_

    Returns:
        _type_: _description_
    """
    tau = 1/(2*np.pi*fc_d) 
    return Kp + Ki/s + Kd*(s/(tau*s+1))

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
    return make_PIFD_gains(Kp, Ki, Kd, fc_d), Kp, Ki, Kd


# ============================================================
# ROOT LOCUS PLOTTING HELPERS
# ============================================================

def plot_OL_RL(open_loop_sys, title="Open-Loop Root Locus"):
    """
    Plot root locus of an already-built open-loop system.

    Example:
        L = C * G
        plot_OL_RL(L)
    """

    plt.figure(figsize=(8, 6))
    gains = np.linspace(0, 100, 1000)

    ctrl.root_locus_plot(
        open_loop_sys,
        grid=True
    )

    plt.title(title)
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.axvline(0, color="k", linestyle="--", linewidth=1)
    plt.show()


def plot_controller_plant_RL(controller, plant, title="Controller * Plant Root Locus"):
    """
    Build open-loop system from controller and plant, then plot root locus.

    L(s) = C(s)G(s)

    Example:
        C = make_pid_from_zeros(a=4, b=20, K=0.1, fc_d=100)
        G = G_motor
        plot_controller_plant_RL(C, G)
    """

    open_loop_sys = controller * plant

    plt.figure(figsize=(8, 6))

    ctrl.root_locus_plot(
        open_loop_sys,
        grid=True
    )

    plt.title(title)
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.axvline(0, color="k", linestyle="--", linewidth=1)
    plt.show()

    return open_loop_sys

def plot_poles_zeros(sys, title="Pole-Zero Map"):
    """
    Plot the poles and zeros of an input transfer function.

    Poles:  x markers
    Zeros:  o markers
    """

    poles = ctrl.poles(sys)
    zeros = ctrl.zeros(sys)

    plt.figure(figsize=(8, 6))

    # Plot zeros
    if len(zeros) > 0:
        plt.scatter(
            np.real(zeros),
            np.imag(zeros),
            marker="o",
            s=100,
            facecolors="none",
            edgecolors="blue",
            label="Zeros"
        )

    # Plot poles
    if len(poles) > 0:
        plt.scatter(
            np.real(poles),
            np.imag(poles),
            marker="x",
            s=100,
            color="red",
            label="Poles"
        )

    plt.axhline(0, color="k", linewidth=1)
    plt.axvline(0, color="k", linestyle="--", linewidth=1)

    plt.title(title)
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")

    plt.show()

    return poles, zeros

def plot_bode(sys, title="Bode Plot", f_min=0.1, f_max=1000, points=1000):
    """
    Plot Bode magnitude and phase for a continuous or discrete control system.

    Args:
        sys: python-control TransferFunction or StateSpace system
        title: plot title
        f_min: minimum frequency in Hz
        f_max: maximum frequency in Hz
        points: number of frequency points
    """

    GM, PM, Wcg, Wcp = ctrl.margin(sys)
    print(f"Gain Margin: {GM:.2f} dB at {Wcg/(2*np.pi):.2f} Hz")
    print(f"Phase Margin: {PM:.1f} deg at {Wcp/(2*np.pi):.2f} Hz")
    print(f"Gain Crossover Frequency: {Wcg/(2*np.pi):.2f} Hz")
    print(f"Phase Crossover Frequency: {Wcp/(2*np.pi):.2f} Hz")

    # Frequency range in rad/s
    freq_hz = np.logspace(np.log10(f_min), np.log10(f_max), points)
    omega = 2.0 * np.pi * freq_hz

    # Get frequency response
    mag, phase, omega = ctrl.frequency_response(sys, omega)

    mag = np.squeeze(mag)
    phase = np.squeeze(phase)

    mag_db = 20.0 * np.log10(np.maximum(mag, 1e-12))
    phase_deg = np.unwrap(phase) * 180.0 / np.pi

    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axs[0].semilogx(freq_hz, mag_db)
    axs[0].set_ylabel("Magnitude [dB]")
    axs[0].grid(True, which="both")
    axs[0].set_title(title)

    axs[1].semilogx(freq_hz, phase_deg)
    axs[1].set_ylabel("Phase [deg]")
    axs[1].set_xlabel("Frequency [Hz]")
    axs[1].grid(True, which="both")

    plt.tight_layout()
    plt.show()

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
def print_loop_info(loop_name, controller: ctrl.TransferFunction, plant: ctrl.TransferFunction, Ts: float, sim_time=1.0, setpoint=1.0, max_cmd = 1):
    """ Prints

    Args:
        loop_name (str): controller name for printing
        controller (ctrl.TransferFunction): controller transfer function
        plant (ctrl.TransferFunction): plant transfer function
        Ts (float): sampling time for simulation
        sim_time (float, optional): simulation time. Defaults to 1.0.
        setpoint (float, optional): setpoint value. Defaults to 1.0.
    """
    system = controller * plant

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
# DISCRETIZE BOTH CONTROLLERS
# ============================================================
vel_Ts = 1/1000  # 1000 Hz control loop
pos_Ts = 1/100  # 100 Hz control loop

G = G_motor * 1/s

#plot_poles_zeros(G, title="Plant Pole-Zero Map")


poles: np.ndarray = ctrl.poles(G)
zeros: np.ndarray = ctrl.zeros(G)

print("Plant poles:", poles)
print("Plant zeros:", zeros)

a = 1.20845921e-02
b = 0.0
k = 0.0453
print(f"Desired zeros at s={-a:.6f} and s={-b:.6f} with gain K={k:.6f}\n")

controller, Kp, Ki, Kd = make_PIFD_zeros(a=a, b=b, K=k, fc_d=10)

#plot_OL_RL(G*controller)

# Increase Kp
# raises the whole magnitude plot
# increases crossover frequency
# makes response faster
# reduces phase margin
# can increase overshoot
Kp -= 0.00015


# Increase Ki
# raises low-frequency gain
# removes steady-state error
# adds phase lag
# can make the system oscillate
Ki += 0.0001

# Increase Kd
# adds phase lead
# improves damping
# can increase phase margin
# helps reduce overshoot
# amplifies noise if not filtered

Kd += 0.000

# Lower derivative filter cutoff
# less noise
# less phase boost
# safer actuator output

#Higher derivative filter cutoff
# more phase boost
# more noise sensitivity
# more voltage spikes

fc_d = 10

print(f"Final PID gains: Kp={Kp:.4f}, Ki={Ki:.4f}, Kd={Kd:.4f}")
controller = make_PIFD_gains(Kp, Ki, Kd, fc_d = fc_d)

# plot_bode(controller * G, title="Velocity Loop Open-Loop Bode")


# ============================================================
# Closed-loop analysis
# ============================================================

#print_loop_info("DISCRETIZED VELCITY LOOP", controller, G, vel_Ts, sim_time=1.0, setpoint=2*np.pi, max_cmd=12)
print_loop_info("DISCRETIZED POSION LOOP", controller, G, pos_Ts, sim_time=5.0, setpoint=np.pi, max_cmd=2*np.pi)

