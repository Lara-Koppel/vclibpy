# # Example for a heat pump with vapor injection using a phase separator

def main():
    # Let's use a flowsheet which is more complex, e.g. the vapor injection
    # with a phase seperator.
    # We can import this flowsheet and how to use it like this:
    from vclibpy.flowsheets import BasePhaseSeparator
    #help(VaporInjectionPhaseSeparator)

    # As it needs the same heat exchanger model as a standard heat pump,
    # we will just use the ones from the standard cycle. Also, as
    # the expansion valve model does not influence the results for
    # the current algorithm, we will just use the same expansion-valve
    # twice. Note, that you could size the two expansion valves
    # using vclibpy, including off-design, but this is one for another
    # example.
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=100,
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
        A=100,
        secondary_medium="air",
        flow_type="cross",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    from vclibpy.components.expansion_valves import Bernoulli
    high_pressure_valve = Bernoulli(A=0.1)
    low_pressure_valve = Bernoulli(A=0.1)
    mid_pressure_valve = Bernoulli(A=0.1)
    # For the compressors, we need to specify low- and high-pressure
    # compressors. To achieve a somewhat similar heat pump as the
    # one in the standard-cycle example, we will  assume that we
    # use two smaller compressors instead of one larger one:
    from vclibpy.components.compressors import RotaryCompressor
    compressor = RotaryCompressor(
        N_max=125,
        V_h=19e-6
    )


    # Now, we can plug everything into the flowsheet:
    flowsheet = BasePhaseSeparator(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        compressor=compressor,
        high_pressure_valve=high_pressure_valve,
        low_pressure_valve=low_pressure_valve,
        mid_pressure_valve=mid_pressure_valve,
    )
    # As in the other example, we can specify save-paths,
    # solver settings and inputs to vary:
    save_path = r"C:\Users\Lara\PycharmProjects\vclibpy\test10"
    T_eva_in = [26 + 273.15]
    T_con = [28 + 273.15]
    n = [0.7, 1]

    # Now, we can generate the full-factorial performance map
    # using all inputs. The results will be stored under the
    # save-path. To see some logs, we can import the logging module
    # and get, for example, all messages equal or above the INFO-level
    import logging
    logging.basicConfig(level="INFO")

    from vclibpy import utils
    utils.full_factorial_map_generation(
        flowsheet=flowsheet,
        save_path=save_path,
        T_con=T_con,
        T_eva_in=T_eva_in,
        n=n,
        use_condenser_inlet=True,
        use_multiprocessing=False,
        save_plots=True,
        m_flow_con=[0.2],
        m_flow_eva=[0.9],
        dT_eva_superheating=[5],
        dT_con_subcooling=[0],
    )


if __name__ == "__main__":
    main()
