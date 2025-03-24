# # Example for standard ejector cycle
import logging

import vclibpy.components.heat_exchangers
from vclibpy.media import MedProp
import numpy
from vclibpy.components.expansion_valves.ejector import Ejector
from vclibpy.media import CoolProp, RefProp
from vclibpy.utils.plotting import plot_cycle
from vclibpy.flowsheets.standard_ejector_cycle import StandardEjectorCycle
import matplotlib.pyplot as plt
import CoolProp.CoolProp as CP
import plotly.graph_objects as go
import math
import pandas as pd
import openpyxl

try:
    plt.style.use('D:/kbr-fme/ebc.paper.mplstyle')
except OSError:
    logging.warning("Could not load the custom matplotlib style file. Using default style.")


med_prop = CoolProp(fluid_name="CarbonDioxide")
ejector = Ejector(1, 2.6, use_quick_solver=True)
ejector.med_prop = med_prop


def main(use_condenser_inlet: bool = True):
    #m_flow_calculation('J:/Massenstromberechnungen/Daten_Ejektor_2.xlsx')
    #error_calculation_p_throat()
    #p_throat_iteration()
    calc_single_ejector_state()


def test_standard_ejector_cycle():
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
    )
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=15,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    from vclibpy.components.expansion_valves import Bernoulli
    expansion_valve = Bernoulli(A=0.01)
    from vclibpy.components.compressors import RotaryCompressor
    compressor = RotaryCompressor(
        N_max=125,
        V_h=19e-6
    )

    flowsheet = StandardEjectorCycle(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        compressor=compressor,
        expansion_valve=ejector,
        metering_valve=expansion_valve
    )


def calc_single_ejector_state():
    ejector.state_primary = med_prop.calc_state("PT", 7e6, 20 + 273.15)
    ejector.state_secondary = med_prop.calc_state("PT", 2e6, -15 + 273.15)
    print(ejector.calc_m_flow(p_3=2.5e6, correlation=False) * 3600)

    states = [ejector.state_primary,
              ejector.state_throat,
              ejector.state_primary_mixing,
              ejector.state_mixing,
              ejector.state_outlet,
              ejector.state_mixing,
              ejector.state_secondary]
    print(states)
    for state in states:
        print(state)
    plot_cycle(med_prop, states, show=True)


def m_flow_calculation(excel_path: str):
    # Read the Excel file
    df = pd.read_excel(excel_path)

    # Drop rows with NaN values
    df = df.dropna(subset=['P_p', 'T_p', 'P_s', 'T_s', 'P_b', 'T_b'])

    pp_list = df['P_p'].tolist()
    Tp_list = df['T_p'].tolist()
    ps_list = df['P_s'].tolist()
    Ts_list = df['T_s'].tolist()
    pb_list = df['P_b'].tolist()
    Tb_list = df['T_b'].tolist()
    m_flow_list = [[0]*2 for i in range(len(pp_list))]

    for i in range(len(pp_list)):
        print(f"Calculating state for pressure: {pp_list[i]} MPa")
        ejector.state_primary = med_prop.calc_state("PT", pp_list[i] * 1e6, Tp_list[i] + 273.15)
        ejector.state_secondary = med_prop.calc_state("PT", ps_list[i] * 1e6, Ts_list[i] + 273.15)
        ejector.calc_m_flow(pb_list[i] * 1e6, correlation=False)
        m_flow_list[i][0] = ejector.m_flow_primary * 3600
        m_flow_list[i][1] = ejector.m_flow_secondary * 3600

    # Add the m_flow_list values to a new column in the DataFrame
    df['m_p_iteration_QNE_2500'] = [m[0] for m in m_flow_list]
    df['m_s_iteration_QNE_2500'] = [m[1] for m in m_flow_list]

    with pd.ExcelWriter(excel_path, mode='a', if_sheet_exists='overlay') as writer:
        df.to_excel(writer, index=False, sheet_name='Tabelle1')

    # plt.figure(figsize=(10, 6))
    # plt.plot(pp_list, [m[0] for m in m_flow_list], label='Primary Mass Flow')
    # plt.plot(pp_list, [m[1] for m in m_flow_list], label='Secondary Mass Flow')
    # plt.xlabel('Primary Pressure [MPa]')
    # plt.ylabel('Mass Flow [kg/h]')
    # plt.title('Primary and Secondary Mass Flow vs Primary Pressure')
    # plt.legend()
    # plt.show()


def qne_plot():
    x = numpy.arange(0.2, 0.8, 0.01)
    Q_NE = 8629 * math.e ** (-10 * (1 - x))
    plt.plot(x, Q_NE)
    plt.show()


def test_partial_derivation():

    def calculate_derivative_T_p_s(fluid, pressure, quality):
        """Calculate (dT/dp)_s using CoolProp."""

        # Use CoolProp to find temperature at given pressure and entropy
        temperature = CP.PropsSI('T', 'P', pressure, 'T', quality, fluid_name)

        # Calculate the derivative (dT/dp)_s using CoolProp's built-in function
        dT_dp_s_value = CP.PropsSI('d(T)/d(P)|S', 'P', pressure, 'Q', quality, fluid)

        return dT_dp_s_value

    # Example usage:
    fluid_name = "CarbonDioxide"  # Example fluid
    pressure_example = 5000000.0  # Pa (example pressure)
    entropy_example = 1417.3  # J/kg.K (example entropy)

    result_derivative_T_p_s = calculate_derivative_T_p_s(fluid_name, pressure_example, 1)

    print(f"The derivative (dT/dp)_s is: {result_derivative_T_p_s}")


def sound_speed_calculation():

    p_throat = 4500000
    q_list = numpy.linspace(0.0001, 0.9999, 100)
    c = []
    c_2 = []
    c_3 = []
    c_4 = []
    phi = []

    state_throat_vapor = med_prop.calc_state("PQ", p_throat, 1)
    state_throat_liquid = med_prop.calc_state("PQ", p_throat, 0)
    for q in q_list:
        state_throat = med_prop.calc_state("PQ", p_throat, q)

        # Volume fractions inside throat
        phi_throat_vapor = ((state_throat.q / state_throat_vapor.d) /
                            ((state_throat.q / state_throat_vapor.d) +
                             ((1-state_throat.q) / state_throat_liquid.d)))
        phi.append(phi_throat_vapor)
        phi_throat_liquid = 1 - phi_throat_vapor
        # Speed of sound for separate phases
        a_liquid = med_prop.get_saturated_speed_of_sound(p_throat, False)
        a_vapor = med_prop.get_saturated_speed_of_sound(p_throat, True)
        # Specific heat capacities
        c_p_liquid = med_prop.calc_transport_properties(state_throat_liquid).cp
        c_p_vapor = med_prop.calc_transport_properties(state_throat_vapor).cp
        # Extensive heat capacities
        C_p_liquid = state_throat_liquid.d * phi_throat_liquid * c_p_liquid
        C_p_vapor = state_throat_vapor.d * phi_throat_vapor * c_p_vapor
        # Thermal expansion coefficients
        beta_liquid = med_prop.calc_transport_properties(state_throat_liquid).beta
        beta_vapor = med_prop.calc_transport_properties(state_throat_vapor).beta
        zeta_liquid = state_throat.T * beta_liquid / (state_throat_liquid.d * c_p_liquid)
        zeta_vapor = state_throat.T * beta_vapor / (state_throat_vapor.d * c_p_vapor)
        x_1 = (phi_throat_vapor / (state_throat_vapor.d * a_vapor ** 2) +
               phi_throat_liquid / (state_throat_liquid.d * a_liquid ** 2))
        x_2 = C_p_vapor * C_p_liquid * (zeta_liquid - zeta_vapor) ** 2 / (C_p_vapor + C_p_liquid)
        # Speed of sound for two phase flow
        c_throat = (state_throat.d * x_1 + (state_throat.d / state_throat.T) * x_2) ** -0.5  #
        c_throat_2 = (state_throat.d * x_1) ** -0.5
        c.append(c_throat)
        c_2.append(c_throat_2)

        temperature = CP.PropsSI('T', 'P', p_throat, 'T', q, "CarbonDioxide")
        dT_dp_s_value_l = CP.PropsSI('d(T)/d(P)|S', 'P', p_throat, 'Q', 0, "CarbonDioxide")
        dT_dp_s_value_g = CP.PropsSI('d(T)/d(P)|S', 'P', p_throat, 'Q', 1, "CarbonDioxide")
        c_throat_3 = (state_throat.d * x_1 + (state_throat.d / state_throat.T) * C_p_vapor*C_p_liquid*(dT_dp_s_value_l-dT_dp_s_value_g)**2/(C_p_vapor+C_p_liquid)) ** -0.5
        print(
            f"q={q}: c_throat_3 = ({state_throat.d} * {x_1} + ({state_throat.d} / {state_throat.T}) * {C_p_vapor} * {C_p_liquid} * ({dT_dp_s_value_l} - {dT_dp_s_value_g}) ** 2 / ({C_p_vapor} + {C_p_liquid})) ** -0.5")
        c_3.append(c_throat_3)
        c_4.append(state_throat.q/state_throat.T*x_2)

    print(med_prop.get_saturated_speed_of_sound(p_throat, False))
    print(med_prop.get_saturated_speed_of_sound(p_throat, True))
    print(med_prop.calc_transport_properties(state_throat_liquid).cp)
    print(med_prop.calc_transport_properties(state_throat_vapor).cp)
    print(state_throat_liquid.d)
    print(state_throat_vapor.d)

    plt.figure(figsize=(10, 8))

    # First subplot
    plt.subplot(2, 1, 1)
    plt.plot(phi, c, label="long")
    plt.plot(phi, c_2, label="short")
    plt.plot(phi, c_3, label="CoolProp")
    plt.legend()
    plt.xlabel('Quality')
    plt.ylabel('Speed of sound')
    plt.title('Speed of sound comparison')

    # Second subplot
    plt.subplot(2, 1, 2)
    plt.plot(phi, c_4, label="difference")
    plt.legend()
    plt.xlabel('Quality')
    plt.ylabel('Speed of sound difference')
    plt.title('Speed of sound difference')

    plt.tight_layout()
    plt.show()


def error_calculation_p_throat():
    """When running: change in ejector.py: 1. check nozzle quality to return -1; 2. return error h_throat after calculation"""

    # Define the range of throat pressures and primary pressures
    p_throat = numpy.arange(1000000, 8000000, 10000)
    p_primary = numpy.arange(3000000, 12000000, 10000)

    # Initialize the error list to store errors for each combination of throat and primary pressures
    error_list = [[0] * len(p_throat) for _ in range(len(p_primary))]
    print(error_list.__len__())
    print(error_list[29].__len__())

    ejector.use_quick_solver = True

    # Iterate over each primary pressure
    i, j = 0, 0
    for p_p in p_primary:
        # Set the primary state for the ejector
        ejector.state_primary = med_prop.calc_state("PT", p_p, 35.2 + 273.15)

        # Iterate over each throat pressure
        for p_t in p_throat:
            # Calculate the error for the current throat pressure and store it in the error list
            error_list[i][j] = ejector.iterate_throat_pressure(p_t)
            j += 1
        i += 1
        j = 0
        print(f"Progress: {i}/{len(p_primary)}")

    # Convert the error list to a numpy array for plotting
    data = numpy.array(error_list).T

    # Create a meshgrid for plotting
    x = p_primary
    y = p_throat
    x, y = numpy.meshgrid(x, y)

    # Plot the error surface
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, data, cmap='viridis')

    # Mark positions where z is about 0
    z_threshold = 0.01  # Define a threshold for z values close to 0
    mask = numpy.abs(data) < z_threshold
    ax.scatter(x[mask], y[mask], data[mask], color='red', s=10, label='z ≈ 0', zorder=10)

    ax.set_xlabel('Primary pressure')
    ax.set_ylabel('Throat pressure')
    ax.set_zlabel('Error')
    plt.show()

    # Create a 3D surface plot
    fig = go.Figure(data=[go.Surface(z=data, x=x, y=y)])

    # Add labels
    fig.update_layout(scene=dict(
        xaxis_title='Primary pressure',
        yaxis_title='Throat pressure',
        zaxis_title='Error'
    ))

    # Mark positions where z is about 0
    z_threshold = 0.01  # Define a threshold for z values close to 0
    mask = numpy.abs(data) < z_threshold
    fig.add_trace(go.Scatter3d(
        x=x[mask],
        y=y[mask],
        z=data[mask],
        mode='markers',
        marker=dict(color='green', size=5),
        name='z ≈ 0'
    ))

    # Save the plot as an HTML file
    fig.write_html('J:/error_surface.html')

    # To view the plot, open the HTML file in a web browser


def p_throat_iteration():

    pp_list = [8.1, 8.15, 8.39, 8.43, 8.44, 8.66, 8.71, 8.72, 9.04, 9.05, 9.19, 9.22, 9.26, 9.3, 9.33, 9.34, 9.45, 9.57,
               9.67, 9.71]
    p_list_2 = numpy.arange(8.0, 10, 0.01)
    m_flow_list = []
    q_list = []
    pt_list = []
    ejector.state_secondary = med_prop.calc_state("PT", 3.6 * 1e6, 22.5 + 273.15)
    for i in range(len(pp_list)):
        print(f"Calculating state for pressure: {pp_list[i]} MPa")
        ejector.state_primary = med_prop.calc_state("PT", pp_list[i] * 1e6, 35.2 + 273.15)
        ejector.calc_m_flow(4.2 * 1e6, correlation=False)
        m_flow_list.append(ejector.m_flow_primary * 3600)
        q_list.append(ejector.state_throat.q)
        pt_list.append(ejector.state_throat.p / 1e6)

    plt.subplot(3, 1, 1)
    plt.plot(pp_list, m_flow_list)
    plt.xlabel('Primary pressure [MPa]')
    plt.ylabel('Primary mass flow [kg/h]')
    plt.title('p_throat iteration - Q_NE factor = 2500')
    plt.subplot(3, 1, 2)
    plt.plot(pp_list, q_list)
    plt.xlabel('Primary pressure [MPa]')
    plt.ylabel('Quality [-]')
    plt.subplot(3, 1, 3)
    plt.plot(pp_list, pt_list)
    plt.xlabel('Primary pressure [MPa]')
    plt.ylabel('Throat pressure [MPa]')
    plt.show()


if __name__ == "__main__":
    main(use_condenser_inlet=False)

