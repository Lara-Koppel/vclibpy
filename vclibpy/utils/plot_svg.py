import logging

from vclibpy.media.ref_prop import RefProp
from vclibpy.media.cool_prop import CoolProp
import matplotlib.pyplot as plt
import numpy as np
import logging

try:
    plt.style.use('D:/kbr-fme/ebc.paper.mplstyle')
except OSError:
    logging.warning("Could not load the custom matplotlib style file. Using default style.")


def plot_log_p_h_diagram(savepath: str,
                         p_bounds: tuple,
                         h_bounds: tuple,
                         fluid: str,
                         grid_size: int = 400,
                         isentropes=True,
                         isothermes=True):
    """
    Create a log(p)-h diagram and save it as an SVG file.

    Parameters:
    savepath (str): Path to save the SVG file.
    p_bounds (tuple): Tuple containing the minimum and maximum pressure (in Pa).
    h_bounds (tuple): Tuple containing the minimum and maximum enthalpy (in J/kg).
    fluid (str): Name of the fluid.
    """

    # Initialize the MedProp class
    med_prop = RefProp(fluid)

    # Generate pressure and enthalpy values
    p_values = np.logspace(np.log10(p_bounds[0]), np.log10(p_bounds[1]), grid_size)
    h_values = np.linspace(h_bounds[0], h_bounds[1], grid_size)

    # Create a meshgrid for plotting
    P, H = np.meshgrid(p_values, h_values)
    T = np.zeros_like(P)
    S = np.zeros_like(P)

    # Calculate temperature for each (p, h) pair
    for i in range(P.shape[0]):
        for j in range(P.shape[1]):
            try:
                state = med_prop.calc_state("PH", P[i, j], H[i, j])
                T[i, j] = state.T
                S[i, j] = state.s
            except ValueError as e:
                T[i, j] = np.nan
                S[i, j] = np.nan
                logging.warning(f"Failed to calculate temperature for (P, H) = ({P[i, j]}, {H[i, j]}): {e}")
        print(f"Progress: {i + 1}/{P.shape[0]}")

    # Plot the log(p)-h diagram
    plt.figure(figsize=(8, 6))
    if isothermes:
        plt.contour(H/1e3, P/1e6, T, levels=np.arange(33.15, 463.15, 20), colors='grey', linewidths=0.8)
    if isentropes:
        plt.contour(H/1e3, P/1e6, S/1e3, levels=np.arange(0, 5, 0.2), colors='grey', linewidths=0.8)

    phase_p = med_prop.get_two_phase_limits("p")
    phase_h = med_prop.get_two_phase_limits("h")
    phase = [[], []]
    for i in range(len(phase_p) - 1):
        if p_bounds[0] < phase_p[i] < p_bounds[1] and h_bounds[0] < phase_h[i] < h_bounds[1]:
            phase[0].append(phase_h[i]/1e3)
            phase[1].append(phase_p[i]/1e6)
    plt.plot(
        phase[0], phase[1], color='black'
    )

    plt.xlabel('h in kJ/kg')
    plt.ylabel('log(p) in MPa')
    plt.yscale('log')
    plt.title(f'log(p)-h Diagram for {fluid}')
    plt.grid(True)

    # Save the plot as an SVG file
    plt.savefig(savepath, format='svg')
    plt.close()


if __name__ == "__main__":
    plot_log_p_h_diagram('D:/kbr-fme/log_p_h_diagram.svg', (1e6, 25e6), (120e3, 520e3), 'CarbonDioxide')