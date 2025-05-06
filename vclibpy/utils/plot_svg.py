import logging

from vclibpy.media.ref_prop import RefProp
from vclibpy.media.cool_prop import CoolProp
import matplotlib.pyplot as plt
import numpy as np
import logging
import progressbar

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

    bar = progressbar.ProgressBar(max_value=P.shape[0])

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
        bar.update(i+1)

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


def plot_T_h_diagram(savepath: str,
                     T_bounds: tuple,
                     h_bounds: tuple,
                     fluid: str,
                     grid_size: int = 400,
                     isentropes=False,
                     plot_states=False):
    """
    Creates a T-h diagram and saves it as an SVG file.

    Parameters:
    savepath (str): Path to save the SVG file.
    T_bounds (tuple): Range of temperature (in K).
    h_bounds (tuple): Range of enthalpy (in J/kg).
    fluid (str): Name of the fluid.
    grid_size (int): Resolution of the diagram (default: 400).
    isentropes (bool): Whether to draw isentropes (lines of constant entropy).
    plot_states (bool): Whether to plot specific states.
    """

    # Initialize the MedProp class with the specified fluid
    med_prop = RefProp(fluid_name=fluid, use_error_check=False)

    # Generate temperature and enthalpy values
    T_values = np.linspace(T_bounds[0], T_bounds[1], grid_size)
    h_values = np.linspace(h_bounds[0], h_bounds[1], grid_size)

    # Create a meshgrid for the diagram data
    T, H = np.meshgrid(T_values, h_values)
    S = np.zeros_like(T)  # Initialize an array for entropy values

    if isentropes:
        # Progress bar for entropy calculation
        bar = progressbar.ProgressBar(max_value=T.shape[0])
        for i in range(T.shape[0]):
            for j in range(T.shape[1]):
                try:
                    # Calculate the state based on temperature and enthalpy
                    state = med_prop.calc_state("TH", T[i, j], H[i, j])
                    S[i, j] = state.s  # Store the entropy
                except ValueError as e:
                    # Handle errors for invalid states
                    S[i, j] = np.nan
                    logging.warning(f"Error calculating for (T, H) = ({T[i, j]}, {H[i, j]}): {e}")
            bar.update(i + 1)

    # Create the diagram
    fig, ax = plt.subplots(figsize=(4, 2.67))

    if isentropes:
        # Draw isentropes (lines of constant entropy)
        ax.contour(H / 1e3, T - 273.15, S / 1e3, levels=np.arange(0, 5, 0.2), colors='grey', linewidths=0.8)

    # Get the phase boundaries (e.g., saturation line) for the fluid
    phase_T = med_prop.get_two_phase_limits("T")
    phase_h = med_prop.get_two_phase_limits("h")
    phase = [[], []]
    for i in range(len(phase_T) - 1):
        if T_bounds[0] < phase_T[i] < T_bounds[1] and h_bounds[0] < phase_h[i] < h_bounds[1]:
            phase[0].append(phase_h[i] / 1e3)  # Enthalpy in kJ/kg
            phase[1].append(phase_T[i] - 273.15)  # Temperature in °C
    # Draw the phase boundary
    ax.plot(phase[0], phase[1], color='black')

    if plot_states:
        # Plot specific states and transitions
        states: List[ThermodynamicState] = []
        states.append(med_prop.calc_state("PT", 10e6, 310))  # State 1
        states.append(med_prop.calc_state("PH", 3e6, states[0].h))  # State 2
        states.append(med_prop.calc_state("PQ", 3e6, 1))  # State 3
        states.append(med_prop.calc_state("PS", 10e6, states[2].s))  # State 4
        h = [state.h * 1e3 for state in states]
        T = [state.T - 273.15 for state in states]
        ax.plot(h, T, marker="o", color="r", zorder=10)  # States as points

        # Draw a cooling curve between two states
        states_cooler: List[ThermodynamicState] = []
        for i in range(100):
            states_cooler.append(
                med_prop.calc_state("PH", 10e6, states[3].h - (states[3].h - states[0].h) * i / 100))
        h_cooler = [state.h * 1e3 for state in states_cooler]
        T_cooler = [state.T - 273.15 for state in states_cooler]
        ax.plot(h_cooler, T_cooler, color="r", zorder=10)

    # Axis labels and limits
    ax.set_xlabel('h in kJ/kg')
    ax.set_ylabel('T in °C')
    ax.set_xlim(h_bounds[0] / 1e3, h_bounds[1] / 1e3)
    ax.set_ylim(T_bounds[0] - 273.15, T_bounds[1] - 273.15)

    # Enable grid lines
    ax.grid(visible=True, which='both', zorder=0)

    # Save the diagram as an SVG file
    fig.savefig(savepath, format='svg')
    plt.show()
    plt.close()

if __name__ == "__main__":
    plot_log_p_h_diagram('D:/kbr-fme/log_p_h_diagram_ejector.svg', (2.5e6, 12e6), (170e3, 510e3), 'CarbonDioxide', plot_states=True, isentropes=True)
    #plot_T_h_diagram('D:/kbr-fme/T_h_diagram_trans.svg', (-30+273.15, 110+273.15), (170e3, 510e3), 'CarbonDioxide',plot_states=True, isentropes=False)
