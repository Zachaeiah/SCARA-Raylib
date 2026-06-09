"""
Clean PID + setpoint feedforward design script.

Control structure:

    e    = r - y
    u_fb = C(z) * e
    u_ff = F(z) * r
    u    = u_fb + u_ff
    y    = G(z) * u

Feedback controller C(z): discrete PID with filtered derivative.
Feedforward F(z): direct setpoint feedforward gain.

Notes:
    - The feedback loop poles are set by C(z) * G(z).
    - Feedforward changes the command/output response, but it does not change
      the feedback loop stability by itself.
    - Kff = 0.0 gives the original feedback-only PID behavior.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import control as ctrl
import matplotlib.pyplot as plt


# =============================================================================
# Motor / plant parameters
# =============================================================================


@dataclass(frozen=True)
class MotorParams:
    rated_current_A: float = 1.5
    resistance_ohm: float = 0.5
    inductance_H: float = 1.5e-3
    inertia_kg_m2: float = 0.08275
    torque_constant_Nm_per_A: float = 1.5
    viscous_friction_Nm_per_rad_s: float = 0.001
    supply_voltage_V: float = 12.0


@dataclass(frozen=True)
class SimConfig:
    sample_time_s: float = 1.0 / 100.0
    sim_time_s: float = 5.0
    setpoint_rad: float = np.pi
    max_command: float = 2.0 * np.pi


@dataclass(frozen=True)
class PidZeroDesign:
    zero_a: float = 0.0
    zero_b: float = 0.0
    derivative_gain: float = 1.0
    derivative_filter_hz: float = 10.0


@dataclass(frozen=True)
class ManualGainTrim:
    kp_trim: float = 0.0
    ki_trim: float = 0.0
    kd_trim: float = 0.0


s = ctrl.TransferFunction.s


# =============================================================================
# Plant construction
# =============================================================================


def make_motor_speed_plant(params: MotorParams) -> ctrl.TransferFunction:
    """
    Build the continuous motor plant from voltage to angular velocity.

        voltage -> current -> torque -> speed
    """
    G_elec = 1.0 / (params.inductance_H * s + params.resistance_ohm)
    G_torque = params.torque_constant_Nm_per_A
    G_mech = 1.0 / (
        params.inertia_kg_m2 * s + params.viscous_friction_Nm_per_rad_s
    )

    return ctrl.minreal(G_elec * G_torque * G_mech, verbose=False)


def make_motor_position_plant(params: MotorParams) -> ctrl.TransferFunction:
    """
    Build the continuous motor plant from voltage to angular position.
    """
    G_speed = make_motor_speed_plant(params)
    G_position = G_speed * (1.0 / s)

    return ctrl.minreal(G_position, verbose=False)


# =============================================================================
# Controller construction
# =============================================================================


def make_pifd_controller(
    Kp: float,
    Ki: float,
    Kd: float,
    derivative_filter_hz: float,
) -> ctrl.TransferFunction:
    """
    Continuous PID-style controller with a first-order filtered derivative.

        C(s) = Kp + Ki/s + Kd * s/(tau*s + 1)

    The derivative filter keeps the derivative term from becoming an ideal
    high-frequency gain block.
    """
    tau = 1.0 / (2.0 * np.pi * derivative_filter_hz)
    C = Kp + Ki / s + Kd * (s / (tau * s + 1.0))

    return ctrl.minreal(C, verbose=False)


def pid_gains_from_zeros(
    zero_a: float,
    zero_b: float,
    derivative_gain: float,
) -> tuple[float, float, float]:
    """
    Convert desired PID numerator zeros into PID gains.

    Desired zeros:
        s = -zero_a
        s = -zero_b

    With:
        C(s) = Kd*s + Kp + Ki/s
             = (Kd*s^2 + Kp*s + Ki) / s

    Then:
        Kp = Kd * (zero_a + zero_b)
        Ki = Kd * zero_a * zero_b
    """
    Kd = derivative_gain
    Kp = derivative_gain * (zero_a + zero_b)
    Ki = derivative_gain * zero_a * zero_b

    return Kp, Ki, Kd


def make_pifd_from_zero_design(
    design: PidZeroDesign,
    trim: ManualGainTrim | None = None,
) -> tuple[ctrl.TransferFunction, float, float, float]:
    """
    Build a continuous filtered PID controller from zero placement, then apply
    optional manual gain trims.
    """
    Kp, Ki, Kd = pid_gains_from_zeros(
        zero_a=design.zero_a,
        zero_b=design.zero_b,
        derivative_gain=design.derivative_gain,
    )

    if trim is not None:
        Kp += trim.kp_trim
        Ki += trim.ki_trim
        Kd += trim.kd_trim

    C = make_pifd_controller(
        Kp=Kp,
        Ki=Ki,
        Kd=Kd,
        derivative_filter_hz=design.derivative_filter_hz,
    )

    return C, Kp, Ki, Kd


def discretize_controller(
    controller_s: ctrl.TransferFunction,
    sample_time_s: float,
) -> ctrl.TransferFunction:
    """
    Discretize the controller using Tustin/bilinear conversion.
    """
    return ctrl.c2d(controller_s, sample_time_s, method="tustin")


def discretize_plant(
    plant_s: ctrl.TransferFunction,
    sample_time_s: float,
) -> ctrl.TransferFunction:
    """
    Discretize the plant using zero-order hold.
    """
    return ctrl.c2d(plant_s, sample_time_s, method="zoh")


# =============================================================================
# Setpoint feedforward
# =============================================================================


def make_setpoint_feedforward(
    Kff: float,
    sample_time_s: float,
) -> ctrl.TransferFunction:
    """
    Direct setpoint feedforward block.

        u_ff = Kff * r

    For this script:
        r    is angular position setpoint [rad]
        u_ff is command added to the feedback command

    Kff = 0.0 gives feedback-only PID.
    """
    return ctrl.TransferFunction([float(Kff)], [1.0], sample_time_s)


def closed_loop_signals_with_feedforward(
    controller_z: ctrl.TransferFunction,
    feedforward_z: ctrl.TransferFunction,
    plant_z: ctrl.TransferFunction,
) -> tuple[
    ctrl.TransferFunction,
    ctrl.TransferFunction,
    ctrl.TransferFunction,
    ctrl.TransferFunction,
    ctrl.TransferFunction,
]:
    """
    Return transfer functions from setpoint r to internal loop signals.

    Control law:
        e    = r - y
        u_fb = C * e
        u_ff = F * r
        u    = u_fb + u_ff
        y    = G * u

    Algebra:
        y/r    = G*(C + F) / (1 + G*C)
        e/r    = (1 - G*F) / (1 + G*C)
        u_fb/r = C * e/r
        u_ff/r = F
        u/r    = u_fb/r + u_ff/r
    """
    C = controller_z
    F = feedforward_z
    G = plant_z

    one = ctrl.TransferFunction([1.0], [1.0], plant_z.dt)
    loop_gain = ctrl.minreal(C * G, verbose=False)
    sensitivity = ctrl.minreal(ctrl.feedback(one, loop_gain), verbose=False)

    E_over_R = ctrl.minreal((one - G * F) * sensitivity, verbose=False)
    Ufb_over_R = ctrl.minreal(C * E_over_R, verbose=False)
    Uff_over_R = F
    U_over_R = ctrl.minreal(Ufb_over_R + Uff_over_R, verbose=False)
    Y_over_R = ctrl.minreal(G * U_over_R, verbose=False)

    return E_over_R, Ufb_over_R, Uff_over_R, U_over_R, Y_over_R


# =============================================================================
# Analysis helpers
# =============================================================================


def format_frequency_hz(omega_rad_s: float) -> str:
    if omega_rad_s is None or not np.isfinite(omega_rad_s):
        return "n/a"

    return f"{omega_rad_s / (2.0 * np.pi):.3f} Hz"


def format_gain_margin(gm: float) -> str:
    if gm is None or np.isnan(gm):
        return "n/a"

    if np.isinf(gm):
        return "inf"

    if gm <= 0.0:
        return f"{gm:.6g} ratio"

    gm_db = 20.0 * np.log10(gm)
    return f"{gm:.6g} ratio / {gm_db:.2f} dB"


def print_feedback_stability(
    controller_z: ctrl.TransferFunction,
    plant_z: ctrl.TransferFunction,
) -> None:
    """
    Print feedback loop margins and closed-loop pole stability.
    """
    open_loop = ctrl.minreal(controller_z * plant_z, verbose=False)
    gm, pm, wg, wp = ctrl.margin(open_loop)

    print("\nFeedback loop margins:")
    print(f"  Gain margin:  {format_gain_margin(gm)}")
    print(f"  GM freq:      {format_frequency_hz(wg)}")
    print(f"  Phase margin: {pm:.3f} deg" if np.isfinite(pm) else "  Phase margin: n/a")
    print(f"  PM freq:      {format_frequency_hz(wp)}")

    closed_loop = ctrl.minreal(ctrl.feedback(open_loop, 1.0), verbose=False)
    poles = ctrl.poles(closed_loop)
    pole_magnitudes = np.abs(poles)

    print("\nFeedback closed-loop poles:")
    print(poles)
    print("\nFeedback closed-loop pole magnitudes:")
    print(pole_magnitudes)

    tolerance = 1e-6
    if np.any(pole_magnitudes > 1.0 + tolerance):
        print("\nWARNING: unstable discrete feedback poles detected.")
    elif np.any(np.isclose(pole_magnitudes, 1.0, atol=tolerance)):
        print("\nWARNING: marginal discrete feedback pole near unit circle detected.")
    else:
        print("\nDiscrete feedback loop is stable.")


def simulate_feedforward_loop(
    controller_z: ctrl.TransferFunction,
    feedforward_z: ctrl.TransferFunction,
    plant_z: ctrl.TransferFunction,
    config: SimConfig,
) -> dict[str, np.ndarray]:
    """
    Simulate the setpoint feedforward loop and return all useful signals.
    """
    t = np.arange(0.0, config.sim_time_s, config.sample_time_s)
    r = np.full_like(t, config.setpoint_rad, dtype=float)

    E_over_R, Ufb_over_R, Uff_over_R, U_over_R, Y_over_R = (
        closed_loop_signals_with_feedforward(
            controller_z=controller_z,
            feedforward_z=feedforward_z,
            plant_z=plant_z,
        )
    )

    _, e = ctrl.forced_response(E_over_R, T=t, U=r)
    _, u_fb = ctrl.forced_response(Ufb_over_R, T=t, U=r)
    _, u_ff = ctrl.forced_response(Uff_over_R, T=t, U=r)
    _, u = ctrl.forced_response(U_over_R, T=t, U=r)
    _, y = ctrl.forced_response(Y_over_R, T=t, U=r)

    return {
        "t": np.asarray(t),
        "r": np.asarray(r),
        "y": np.asarray(y),
        "e": np.asarray(e),
        "u_fb": np.asarray(u_fb),
        "u_ff": np.asarray(u_ff),
        "u": np.asarray(u),
    }


def print_response_info(
    signals: dict[str, np.ndarray],
    max_command: float,
) -> None:
    """
    Print command saturation and basic response information.
    """
    t = signals["t"]
    y = signals["y"]
    u = signals["u"]
    r = signals["r"]

    setpoint = float(r[-1])
    u_max = float(np.max(u))
    u_min = float(np.min(u))
    final_output = float(y[-1])
    final_error = setpoint - final_output

    print("\nCommand limits:")
    print(f"  Max command: {u_max:.6f}")
    print(f"  Min command: {u_min:.6f}")
    print(f"  Limit:       +/-{max_command:.6f}")

    if u_max > max_command or u_min < -max_command:
        print("  WARNING: total command exceeds the actuator limit.")
    else:
        print("  Total command stays inside the actuator limit.")

    print("\nFinal values:")
    print(f"  Setpoint:     {setpoint:.6f}")
    print(f"  Final output: {final_output:.6f}")
    print(f"  Final error:  {final_error:.6f}")

    try:
        # This uses the simulated output directly, so it avoids ambiguity around
        # continuous vs discrete model representation.
        info = ctrl.step_info(y, T=t, yfinal=setpoint)
        print("\nStep information:")
        for key, value in info.items():
            if isinstance(value, (float, int, np.floating)):
                print(f"  {key}: {value:.6f}")
            else:
                print(f"  {key}: {value}")
    except Exception as err:
        print(f"\nStep information failed: {err}")


# =============================================================================
# Plotting helpers
# =============================================================================


def plot_poles_zeros(
    system: ctrl.TransferFunction,
    title: str = "Pole-Zero Map",
) -> tuple[np.ndarray, np.ndarray]:
    poles = ctrl.poles(system)
    zeros = ctrl.zeros(system)

    plt.figure(figsize=(8, 6))

    if len(zeros) > 0:
        plt.scatter(
            np.real(zeros),
            np.imag(zeros),
            marker="o",
            s=100,
            facecolors="none",
            edgecolors="blue",
            label="Zeros",
        )

    if len(poles) > 0:
        plt.scatter(
            np.real(poles),
            np.imag(poles),
            marker="x",
            s=100,
            color="red",
            label="Poles",
        )

    plt.axhline(0.0, color="black", linewidth=1)
    plt.axvline(0.0, color="black", linestyle="--", linewidth=1)
    plt.title(title)
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.tight_layout()
    plt.show()

    return poles, zeros


def plot_root_locus(
    open_loop_system: ctrl.TransferFunction,
    title: str = "Open-Loop Root Locus",
) -> None:
    plt.figure(figsize=(8, 6))
    ctrl.root_locus_plot(open_loop_system, grid=True)
    plt.title(title)
    plt.xlabel("Real Axis")
    plt.ylabel("Imaginary Axis")
    plt.axvline(0.0, color="black", linestyle="--", linewidth=1)
    plt.tight_layout()
    plt.show()


def plot_bode(
    system: ctrl.TransferFunction,
    title: str = "Bode Plot",
    f_min_hz: float = 0.1,
    f_max_hz: float = 500.0,
    point_count: int = 1000,
) -> None:
    gm, pm, wg, wp = ctrl.margin(system)

    print("\nBode margin summary:")
    print(f"  Gain margin:  {format_gain_margin(gm)}")
    print(f"  GM freq:      {format_frequency_hz(wg)}")
    print(f"  Phase margin: {pm:.3f} deg" if np.isfinite(pm) else "  Phase margin: n/a")
    print(f"  PM freq:      {format_frequency_hz(wp)}")

    freq_hz = np.logspace(np.log10(f_min_hz), np.log10(f_max_hz), point_count)
    omega_rad_s = 2.0 * np.pi * freq_hz

    mag, phase, _ = ctrl.frequency_response(system, omega_rad_s)
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


def plot_feedforward_response(
    signals: dict[str, np.ndarray],
    max_command: float,
    title: str = "PID + Setpoint Feedforward Response",
) -> None:
    t = signals["t"]
    r = signals["r"]
    y = signals["y"]
    e = signals["e"]
    u_fb = signals["u_fb"]
    u_ff = signals["u_ff"]
    u = signals["u"]

    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 10))

    axs[0].plot(t, r, label="setpoint r")
    axs[0].plot(t, y, label="plant output y")
    axs[0].set_ylabel("Position [rad]")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(t, u_fb, label="feedback command u_fb")
    axs[1].plot(t, u_ff, label="feedforward command u_ff")
    axs[1].plot(t, u, label="total command u")
    axs[1].plot(t, np.full_like(t, max_command), linestyle="--", label="+ limit")
    axs[1].plot(t, np.full_like(t, -max_command), linestyle="--", label="- limit")

    over_pos = u > max_command
    over_neg = u < -max_command

    if np.any(over_pos):
        axs[1].scatter(t[over_pos], u[over_pos], s=12, label="positive limit violation")

    if np.any(over_neg):
        axs[1].scatter(t[over_neg], u[over_neg], s=12, label="negative limit violation")

    axs[1].set_ylabel("Command")
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(t, e, label="error e")
    axs[2].set_ylabel("Error [rad]")
    axs[2].grid(True)
    axs[2].legend()

    axs[3].plot(t, r - y, label="r - y check")
    axs[3].set_ylabel("Tracking error [rad]")
    axs[3].set_xlabel("Time [s]")
    axs[3].grid(True)
    axs[3].legend()

    fig.suptitle(title)
    plt.tight_layout()
    plt.show()


# =============================================================================
# Main design run
# =============================================================================


def main() -> None:
    motor = MotorParams()
    sim = SimConfig(
        sample_time_s=1.0 / 1000.0,
        sim_time_s=0.5,
        setpoint_rad=np.pi,
        max_command=12,
    )

    pid_design = PidZeroDesign(
        zero_a=1.20845921e-02,
        zero_b=0.0,
        derivative_gain=0.005,
        derivative_filter_hz=10.0,
    )

    gain_trim = ManualGainTrim(
        kp_trim=0.8,
        ki_trim=0.015,
        kd_trim=0.00,
    )

    # Direct setpoint feedforward gain.
    # Use Kff = 0.0 to compare against feedback-only PID.
    Kff = 0.00

    plant_s = make_motor_speed_plant(motor)
    controller_s, Kp, Ki, Kd = make_pifd_from_zero_design(pid_design, gain_trim)

    plant_z = discretize_plant(plant_s, sim.sample_time_s)
    controller_z = discretize_controller(controller_s, sim.sample_time_s)
    feedforward_z = make_setpoint_feedforward(Kff, sim.sample_time_s)

    # plot_root_locus(controller_z * plant_z)
    # plot_poles_zeros(controller_z * plant_z)

    print("\n================ PID + SETPOINT FEEDFORWARD DESIGN ================")
    print(f"Sample time: {sim.sample_time_s:.6f} s")
    print(f"Setpoint:    {sim.setpoint_rad:.6f} rad/s")
    print(f"Kff:         {Kff:.6f}")

    print("\nContinuous PID gains before discretization:")
    print(f"  Kp = {Kp:.8f}")
    print(f"  Ki = {Ki:.8f}")
    print(f"  Kd = {Kd:.8f}")

    plant_poles = ctrl.poles(plant_s)
    plant_zeros = ctrl.zeros(plant_s)
    print("\nContinuous plant poles:")
    print(plant_poles)
    print("\nContinuous plant zeros:")
    print(plant_zeros)

    print_feedback_stability(controller_z, plant_z)

    signals = simulate_feedforward_loop(
        controller_z=controller_z,
        feedforward_z=feedforward_z,
        plant_z=plant_z,
        config=sim,
    )

    print_response_info(signals, max_command=sim.max_command)

    # Uncomment whichever plots you want while tuning.
    plot_poles_zeros(plant_s, title="Continuous Position Plant Pole-Zero Map")
    plot_root_locus(controller_s * plant_s, title="Continuous Controller * Plant Root Locus")
    plot_bode(controller_z * plant_z, title="Discrete Open-Loop Bode")

    plot_feedforward_response(
        signals,
        max_command=sim.max_command,
        title="PID + Setpoint Feedforward Position Loop",
    )


if __name__ == "__main__":
    main()
