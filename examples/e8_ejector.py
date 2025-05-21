# # Example for standard ejector cycle
import logging
import numpy
import pathlib
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd

from vclibpy.components.expansion_valves.ejector import Ejector
from vclibpy.media import CoolProp, RefProp
from vclibpy.flowsheets.standard_ejector_cycle import StandardEjectorCycle
from vclibpy.utils.plotting import plot_cycle

try:
    plt.style.use('D:/kbr-fme/ebc.paper.mplstyle')
except OSError:
    logging.warning("Could not load the custom matplotlib style file. Using default style.")
    print("Could not load the custom matplotlib style file. Using default style.")

mpl.rcParams['svg.fonttype'] = 'none'  # make pyplot save text as text and not paths, so it is editable in Inkscape

med_prop = RefProp(fluid_name="CarbonDioxide")
ejector = Ejector(0.8, 2, use_quick_solver=True)  # Ejector 1
ejector_2 = Ejector(1, 2.6, use_quick_solver=True)  # Ejector 2
ejector.med_prop = med_prop
ejector_2.med_prop = med_prop


def main(use_condenser_inlet: bool = True):
    # calc_single_ejector_state()
    test_standard_ejector_cycle()

def test_standard_ejector_cycle():
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer
    from vclibpy.components.expansion_valves import Bernoulli
    from vclibpy.components.compressors import RotaryCompressor
    from vclibpy.components.phase_separator import PhaseSeparator
    from vclibpy.datamodels import Inputs, FlowsheetState, RelativeCompressorSpeedControl, HeatExchangerInputs
    from vclibpy.algorithms.iteration import Iteration

    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=30,
        secondary_medium="air",
        flow_type="cross",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=30,
        secondary_medium="air",
        flow_type="cross",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    expansion_valve = Bernoulli(A=0.01)
    compressor = RotaryCompressor(
        N_max=125,
        V_h=19e-6
    )
    phase_seperator = PhaseSeparator()

    flowsheet = StandardEjectorCycle(
        evaporator=evaporator,
        condenser=condenser,
        fluid="CarbonDioxide",
        compressor=compressor,
        ejector=ejector_2,
        metering_valve=expansion_valve,
        phase_seperator=phase_seperator
    )
    save_path = pathlib.Path(r"D:\00_temp\standard_ejector_cycle")
    algorithm = Iteration(raise_errors=True, save_path_plots=save_path, show_iteration=True)
    speed_control = RelativeCompressorSpeedControl(0.2, 0.0, 5.0)
    eva_inputs = HeatExchangerInputs(T_in=-5 + 273.15, m_flow=1.00842)
    con_inputs = HeatExchangerInputs(T_in=15 + 273.15, m_flow=1.00842)
    inputs = Inputs(control=speed_control, evaporator=eva_inputs, condenser=con_inputs)
    p_1_start, p_2_start, p_max, fs_state = algorithm.initial_setup(flowsheet, fluid=None, inputs=inputs)

    for i in range(10):
        speed_control = RelativeCompressorSpeedControl(0.2, 0.0, dT_con_subcooling=i)
        inputs = Inputs(control=speed_control, evaporator=eva_inputs, condenser=con_inputs)
        fs_state = algorithm.calc_steady_state(flowsheet, inputs, "CarbonDioxide", )
    # plot_cycle(flowsheet.med_prop, flowsheet.get_states_in_order_for_plotting(), show=True)
        print(f"Subcooling: {i} K")
        print(f"Compressor:")
        print(f"m_flow = {flowsheet.compressor.m_flow * 3600} kg/h")
        print(f"state_inlet = {flowsheet.compressor.state_inlet}")
        print(f"state_outlet = {flowsheet.compressor.state_outlet}")
        print(f"Ejector:")
        print(f"m_flow_outlet = {flowsheet.ejector.m_flow_outlet * 3600} kg/h, m_flow_primary = {flowsheet.ejector.m_flow_primary * 3600} kg/h, m_flow_secondary = {flowsheet.ejector.m_flow_secondary * 3600} kg/h")
        print(f"state_outlet = {flowsheet.ejector.state_outlet}")
        print(f"state_primary = {flowsheet.ejector.state_primary}")
        print(f"Condenser:")
        print(f"m_flow = {flowsheet.condenser.m_flow * 3600} kg/h")
        print(f"state_inlet = {flowsheet.condenser.state_inlet}")
        print(f"state_outlet = {flowsheet.condenser.state_outlet}")
        print(f"Evaporator:")
        print(f"m_flow = {flowsheet.evaporator.m_flow * 3600} kg/h")
        print(f"state_inlet = {flowsheet.evaporator.state_inlet}")
        print(f"state_outlet = {flowsheet.evaporator.state_outlet}")

        # Get all possible values:
        variables_to_excel = [{
            **fs_state.convert_to_str_value_format(with_unit_and_description=False),
        }]

        # Save to excel
        save_path_csv = save_path.joinpath(f"{flowsheet.flowsheet_name}_{flowsheet.fluid}_{i}.csv")
        pd.DataFrame(variables_to_excel).to_csv(
            save_path_csv, sep=","
        )


def calc_single_ejector_state():
    ejector_2.state_primary = med_prop.calc_state("PT", 9e6, 35 + 273.15)
    ejector_2.state_secondary = med_prop.calc_state("PQ", 3e6, 1)

    p3_list = numpy.arange(3.45, 3.8, 0.01)
    m_flow_p_list = []
    m_flow_s_list = []
    m_flow_outlet_list = []
    m_flow_outlet_vapor_list = []
    m_flow_outlet_liquid_list = []

    for p_3 in p3_list:
        ejector_2.calc_m_flow(p_3 * 1e6, correlation=True)
        m_flow_p_list.append(ejector_2.m_flow_primary * 3600)
        m_flow_s_list.append(ejector_2.m_flow_secondary * 3600)
        m_flow_outlet_list.append(ejector_2.m_flow_outlet * 3600)
        m_flow_outlet_vapor_list.append(ejector_2.m_flow_outlet * ejector_2.state_outlet.q * 3600)
        m_flow_outlet_liquid_list.append(ejector_2.m_flow_outlet * (1-ejector_2.state_outlet.q) * 3600)
        print(f"p_3 = {p_3} MPa")
        print(f"mdot_p = {ejector_2.m_flow_primary * 3600} kg/h, mdot_s = {ejector_2.m_flow_secondary * 3600} kg/h, mdot_outlet = {ejector_2.m_flow_outlet * 3600} kg/h")
        print(f"mdot_outlet_vapor = {ejector_2.m_flow_outlet * ejector_2.state_outlet.q * 3600} kg/h, mdot_outlet_liquid = {ejector_2.m_flow_outlet * (1-ejector_2.state_outlet.q) * 3600} kg/h")

    plt.figure(figsize=(6.125, 6.125/4*2))
    plt.subplot(1, 1, 1)
    colorcycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
    plt.plot(p3_list, m_flow_p_list, label='mdot_p', zorder=10, color=colorcycle[0])
    plt.plot(p3_list, m_flow_s_list, label='mdot_s', zorder=10, color=colorcycle[13])
    plt.plot(p3_list, m_flow_outlet_vapor_list, label='mdot_outlet_vapor', zorder=10, color=colorcycle[0], linestyle='--')
    plt.plot(p3_list, m_flow_outlet_liquid_list, label='mdot_outlet_liquid', zorder=10, color=colorcycle[13], linestyle='--')
    plt.xlabel('p_3 in MPa')
    plt.ylabel('Mass flow in kg/h')
    plt.grid(visible=True, which='both', zorder=0)
    plt.legend(ncol=2)
    plt.show()

    states = [ejector_2.state_primary,
              ejector_2.state_throat,
              ejector_2.state_primary_mixing,
              ejector_2.state_mixing,
              ejector_2.state_outlet,
              ejector_2.state_mixing,
              ejector_2.state_secondary]
    print(states)
    for state in states:
        print(state)
    plot_cycle(med_prop, states, show=True)

if __name__ == "__main__":
    main(use_condenser_inlet=False)

