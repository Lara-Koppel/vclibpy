# # Example for standard ejector cycle
def main(use_condenser_inlet: bool = True):
    from vclibpy.components.expansion_valves.ejector import Ejector
    from vclibpy.media import RefProp
    med_prop = RefProp(fluid_name="CarbonDioxide")

    ejector = Ejector(0.8, 2)
    ejector.med_prop = med_prop
    ejector.state_primary = med_prop.calc_state("PT", 8.25e6, 35.5+273.15)
    ejector.state_secondary = med_prop.calc_state("PT", 4.15e6, 21.9+273.15)
    print(ejector.calc_m_flow(4.62e6))


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