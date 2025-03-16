# # Example for standard ejector cycle
from vclibpy.media import MedProp


def main(use_condenser_inlet: bool = True):
    import numpy
    from vclibpy.components.expansion_valves.ejector import Ejector
    from vclibpy.media import CoolProp, RefProp
    from vclibpy.utils.plotting import plot_cycle
    import matplotlib.pyplot as plt
    import CoolProp.CoolProp as CP
    med_prop = CoolProp(fluid_name="CarbonDioxide")
    # p_throat = 4500000
    # q_list = numpy.linspace(0.0001, 0.9999, 100)
    # c = []
    # c_2 = []
    # c_3 = []
    # phi = []
    # for q in q_list:
    #     state_throat = med_prop.calc_state("PQ", p_throat, q)
    #
    #     state_throat_vapor = med_prop.calc_state("PQ", p_throat, 1)
    #     state_throat_liquid = med_prop.calc_state("PQ", p_throat, 0)
    #     # Volume fractions inside throat
    #     phi_throat_vapor = ((state_throat.q / state_throat_vapor.d) /
    #                         ((state_throat.q / state_throat_vapor.d) +
    #                          ((1-state_throat.q) / state_throat_liquid.d)))
    #     phi.append(phi_throat_vapor)
    #     phi_throat_liquid = 1 - phi_throat_vapor
    #     # Speed of sound for separate phases
    #     a_liquid = med_prop.get_saturated_speed_of_sound(p_throat, False)
    #     a_vapor = med_prop.get_saturated_speed_of_sound(p_throat, True)
    #     # Specific heat capacities
    #     c_p_liquid = med_prop.calc_transport_properties(state_throat_liquid).cp
    #     c_p_vapor = med_prop.calc_transport_properties(state_throat_vapor).cp
    #     # Extensive heat capacities
    #     C_p_liquid = state_throat_liquid.d * phi_throat_liquid * c_p_liquid
    #     C_p_vapor = state_throat_vapor.d * phi_throat_vapor * c_p_vapor
    #     # Thermal expansion coefficients
    #     beta_liquid = med_prop.calc_transport_properties(state_throat_liquid).beta
    #     beta_vapor = med_prop.calc_transport_properties(state_throat_vapor).beta
    #     zeta_liquid = state_throat.T * beta_liquid / (state_throat_liquid.d * c_p_liquid)
    #     zeta_vapor = state_throat.T * beta_vapor / (state_throat_vapor.d * c_p_vapor)
    #     x_1 = (phi_throat_vapor / (state_throat_vapor.d * a_vapor ** 2) +
    #            phi_throat_liquid / (state_throat_liquid.d * a_liquid ** 2))
    #     x_2 = C_p_vapor * C_p_liquid * (zeta_liquid - zeta_vapor) ** 2 / (C_p_vapor + C_p_liquid)
    #     # Speed of sound for two phase flow
    #     c_throat = (state_throat.d * x_1 + (state_throat.d / state_throat.T) * x_2) ** -0.5  #
    #     c_throat_2 = (state_throat.d * x_1) ** -0.5
    #     c.append(c_throat)
    #     c_2.append(c_throat_2)
    #
    #     temperature = CP.PropsSI('T', 'P', p_throat, 'T', q, "CarbonDioxide")
    #     dT_dp_s_value_l = CP.PropsSI('d(T)/d(P)|S', 'P', p_throat, 'Q', 0, "CarbonDioxide")
    #     dT_dp_s_value_g = CP.PropsSI('d(T)/d(P)|S', 'P', p_throat, 'Q', 1, "CarbonDioxide")
    #     c_throat_3 = (state_throat.d * x_1 + (state_throat.d / state_throat.T) * C_p_vapor*C_p_liquid*(dT_dp_s_value_l-dT_dp_s_value_g)**2/(C_p_vapor+C_p_liquid)) ** -0.5
    #     c_3.append(c_throat_3)
    #
    # print(med_prop.get_saturated_speed_of_sound(p_throat, False))
    # print(med_prop.get_saturated_speed_of_sound(p_throat, True))
    #
    # plt.figure()
    # plt.plot(phi, c, label="long")
    # plt.plot(phi, c_2, label="short")
    # plt.plot(phi, c_3, label="CoolProp")
    # plt.legend()
    # plt.xlabel('Quality')
    # plt.ylabel('Speed of sound')
    # plt.show()

    #
    p_list = [8.1, 8.15, 8.39, 8.43, 8.44, 8.66, 8.71, 8.72, 9.04, 9.05, 9.19, 9.22, 9.26, 9.3, 9.33, 9.34, 9.45, 9.57, 9.67, 9.71]
    ejector = Ejector(0.8, 2)
    ejector.med_prop = med_prop
    ejector.state_secondary = med_prop.calc_state("PT", 3.60e6, 21.2+273.15)
    # for p in p_list:
    #     print(f"Calculating state for pressure: {p} MPa")
    #     ejector.state_primary = med_prop.calc_state("PT", p*1e6, 35.2+273.15)
    #     ejector.calc_m_flow(4.30e6)
    ejector.state_primary = med_prop.calc_state("PT", 8.72e6, 35.2 + 273.15)
    print(ejector.calc_m_flow(4.30e6))

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


if __name__ == "__main__":
    main(use_condenser_inlet=False)

# import CoolProp.CoolProp as CP
#
#
# def calculate_derivative_T_p_s(fluid_name, pressure, quality):
#     """Calculate (dT/dp)_s using CoolProp."""
#
#     # Use CoolProp to find temperature at given pressure and entropy
#     temperature = CP.PropsSI('T', 'P', pressure, 'T', quality, fluid_name)
#
#     # Calculate the derivative (dT/dp)_s using CoolProp's built-in function
#     dT_dp_s_value = CP.PropsSI('d(T)/d(P)|S', 'P', pressure, 'Q', quality, fluid_name)
#
#     return dT_dp_s_value
#
#
# # Example usage:
# fluid_name = "CarbonDioxide"  # Example fluid
# pressure_example = 5000000.0  # Pa (example pressure)
# entropy_example = 1417.3  # J/kg.K (example entropy)
#
# result_derivative_T_p_s = calculate_derivative_T_p_s(fluid_name, pressure_example, 1)
#
# print(f"The derivative (dT/dp)_s is: {result_derivative_T_p_s}")