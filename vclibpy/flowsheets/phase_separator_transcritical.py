from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.phase_separator import PhaseSeparator
from vclibpy.media import ThermodynamicState
import numpy
import abc
from copy import deepcopy
import logging
from scipy.optimize import fsolve

logger = logging.getLogger(__name__)

class PhaseSeparatorTranscritical(BaseCycle, abc.ABC):
    """
    Class for a standard cycle with four components.

    For the standard cycle, we have 4 possible states:

    1. Before compressor, after evaporator
    2. Before condenser, after compressor
    3. Before EV, after condenser
    4. Before Evaporator, after EV
    """

    flowsheet_name = "PhaseSeparatorTranscritical"

    #We first define the constructor for the StandardCycleTranscritical class, which expects a Compressor and an Expansion valve
    #We also pass any additional keyword arguments to the parent class constructor. That is done throuhg the **kwargs, which stands for keyword arguments.
    #The "self, compressor: Compressor, expansion_valve: ExpansionValve, **kwargs" in the brackets means that the constructor expects
    #a Compressor object and an ExpansionValve object, along with any other keyword arguments that might be passed.
    #Afterwards we call the parent class constructor with super().__init__(**kwargs). So every argument in the base class will also be
    #passed to the StandardCycleTranscritical class. See in BaseCycle: there we define the fluid as a string, the evaporator and condenser.
    #self.compressor and self.expansion_valve make sure, we can set the compressor and expansion valve as attributes of the StandardCycleTranscritical class.
    #so that we can code sth. like flowsheet = StandardCycleTranscritical(compressor=my_compressor, expansion_valve=my_expansion_valve)
    #otherwise we would not be able to access the compressor and expansion valve in the flowsheet object. Same for the parent class constructor

    def __init__(
            self,
            compressor: Compressor,
            high_pressure_valve: ExpansionValve,
            low_pressure_valve: ExpansionValve,
            mid_pressure_valve: ExpansionValve,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.high_pressure_valve = high_pressure_valve
        self.low_pressure_valve = low_pressure_valve
        self.mid_pressure_valve = mid_pressure_valve
        self.phase_separator = PhaseSeparator()
        # Avoid nasty bugs for setting states
        if id(low_pressure_valve) == id(high_pressure_valve):
            self.high_pressure_valve = deepcopy(low_pressure_valve)
        if id(low_pressure_valve) == id(mid_pressure_valve):
            self.mid_pressure_valve = deepcopy(low_pressure_valve)

    # Nice to know: get_all_components is a called a method, because it is a function that is defined inside a class
    # get_all_components also exists in the BaseCycle class. Therefore we call the parent function "get_all_components" with super().get_all_components()
    # After that we add the compressor and expansion valve to the list of components.
    # At the end, we should have a list of all components (excluding the fluid for example),
    # and it should look like this: [my_condenser, my_evaporator, my_compressor, my_expansion_valve]

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.high_pressure_valve,
            self.low_pressure_valve,
            self.mid_pressure_valve,
            self.phase_separator,
        ]

    # In this function, all the states of the cycle are defined. Compared to the old subcritical cycle
    # we don't have a constant temperature in the two-phase region. Before it was only a return function
    # now it is a given state list.
    def get_states_in_order_for_plotting(self):
        states = [
            self.low_pressure_valve.state_inlet,
            self.low_pressure_valve.state_outlet,
            self.evaporator.state_inlet,
            self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1),
            self.evaporator.state_outlet,
            self.compressor.state_inlet,
            self.compressor.state_outlet,
        ]

        # Compared to the subcritical flowsheet, we cannot calculate the inlet and outlet state of the condenser/gas cooler
        # through the quality of the vapor, due to the supercritical state inside the gas cooler
        # Therefore the gas cooler is split into 20 segments and
        # Interpolate the states between the condenser inlet and outlet
        p = self.condenser.state_inlet.p
        h_in = self.condenser.state_inlet.h
        h_out = self.condenser.state_outlet.h
        h_steps = numpy.linspace(h_in, h_out, 50)

        for h_val in h_steps:
            inter_state = self.med_prop.calc_state("PH", p, h_val)
            states.append(inter_state)

        states.append(self.high_pressure_valve.state_inlet)
        states.append(self.high_pressure_valve.state_outlet)
        states.append(self.phase_separator.state_inlet)
        states.append(self.phase_separator.state_outlet_vapor)
        states.append(self.mid_pressure_valve.state_inlet)
        states.append(self.mid_pressure_valve.state_outlet)
        states.append(self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1))
        states.append(self.compressor.state_inlet)
        # Go back to separator for clear lines
        states.append(self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1))
        states.append(self.mid_pressure_valve.state_outlet)
        states.append(self.mid_pressure_valve.state_inlet)
        states.append(self.phase_separator.state_outlet_vapor)
        states.append(self.phase_separator.state_outlet_liquid)

        return states

    def set_condenser_outlet_based_on_q(self, p_con: float, inputs: Inputs, q_4, p_eva: float):
        h_4 = self.med_prop.calc_state("PQ", p_eva, q_4).h
        self.condenser.state_outlet = self.med_prop.calc_state("PH", p_con, h_4)
        #print(self.condenser.state_outlet)

    def set_condenser_outlet_based_on_pinch_point(self, p_2, inputs, pinch_point=3):
        """
        Set the condenser outlet based on the pinch point.
        For maximal efficiency, the pinch point in a gas cooler should be at the gas cooler outlet.

        Args:
            p_2 (float): Gas cooler pressure in Pa.
            inputs (Inputs): Inputs object containing the condenser information.
            pinch_point (float): Pinch point in K. Default is 2 K.
        """
        if inputs.condenser.uses_inlet:
            T_in = inputs.condenser.T_in
        else:
            raise NotImplementedError("Secondary condenser outlet temperature calculation not implemented yet. "
                                      "Condenser inlet temperature for secondary side needs to be provided.")
        return self.med_prop.calc_state("PT", p_2, T_in + pinch_point)

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState, **kwargs):
        """
        Final Corrected Version. This method now performs the full cycle calculation
        and returns the evaporator enthalpy error directly. This is the function
        that the brentq solver will find the root of.
        """
        p_mid = kwargs.get("p_mid")
        if p_mid is None:
            raise TypeError("PhaseSeparatorTranscritical.calc_states() is missing 'p_mid'")

        # 1. DEFINE COMPRESSOR INLET & CALCULATE MASS FLOW
        T_sat_p1 = self.med_prop.calc_state("PQ", p_1, 1).T
        T_1 = T_sat_p1 + inputs.control.dT_eva_superheating
        self.compressor.state_inlet = self.med_prop.calc_state("PT", p_1, T_1)
        h_1_required = self.compressor.state_inlet.h
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        m_flow_total = self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)

        if m_flow_total <= 1e-9: raise ValueError("Mass flow is zero.")
        self.compressor.m_flow = self.condenser.m_flow = self.high_pressure_valve.m_flow = m_flow_total

        # 2. CALCULATE HIGH-PRESSURE SIDE
        self.condenser.state_inlet = self.compressor.state_outlet

        # Condenser outlet
        def get_condenser_error(T_3_guess_array):
            T_3_guess = T_3_guess_array[0]
            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_guess)
            error, _ = self.condenser.calc(inputs=inputs, fs_state=fs_state)
            return error

        if inputs.condenser.uses_inlet:
            T_con_sec_in = inputs.condenser.T_in
            T_3_initial_guess = T_con_sec_in + 5.0
        else:
            T_con_sec_out = inputs.condenser.T_out
            T_3_initial_guess = T_con_sec_out - 5.0

        try:
            T_3_solution_array, _, ier, _ = fsolve(get_condenser_error, x0=[T_3_initial_guess], xtol=0.01,
                                                   full_output=True)

            if ier != 1:
                raise ValueError("fsolve_condenser_did_not_converge")

            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_solution_array[0])

        except Exception as e:
            raise ValueError("fsolve_condenser_did_not_converge") from e

        # T_2 = self.compressor.state_outlet.T
        # T_con_sec_in = inputs.condenser.T_in
        # if T_2 <= T_con_sec_in:
        #     raise ValueError(f"T_2 ({T_2 - 273.15:.1f}C) is not > T_con_in ({T_con_sec_in - 273.15:.1f}C)")
        #
        # try:  # Your fsolve condenser model
        #     def get_condenser_error(T_3_guess_array):
        #         self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_guess_array[0])
        #         error, _ = self.condenser.calc(inputs=inputs, fs_state=fs_state)
        #         return error
        #
        #     T_3_solution_array, _, ier, _ = fsolve(get_condenser_error, x0=[T_con_sec_in + 5.0], xtol=0.01,
        #                                            full_output=True)
        #     if ier != 1: raise ValueError("fsolve_condenser failed")
        #     self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_solution_array[0])
        # except Exception as e:
        #     raise ValueError(f"Condenser calculation failed: {e}") from e

        # 3. CALCULATE SEPARATOR AND VALVE STATES
        self.high_pressure_valve.state_inlet = self.condenser.state_outlet
        self.high_pressure_valve.calc_outlet(p_outlet=p_mid)

        self.phase_separator.state_inlet = self.high_pressure_valve.state_outlet
        x_phase_separator = self.phase_separator.state_inlet.q
        if x_phase_separator > 0.0 and x_phase_separator < 1.0:
            state_liquid_out = self.phase_separator.state_outlet_liquid
            state_vapor_out = self.phase_separator.state_outlet_vapor

            self.mid_pressure_valve.state_inlet = state_vapor_out
            self.mid_pressure_valve.calc_outlet(p_outlet=p_1)
            h_flash_gas_throttled = self.mid_pressure_valve.state_outlet.h

            self.low_pressure_valve.state_inlet = state_liquid_out
            self.low_pressure_valve.calc_outlet(p_outlet=p_1)

            self.evaporator.state_inlet = self.low_pressure_valve.state_outlet
            self.evaporator.m_flow = m_flow_total * (1 - x_phase_separator)
            h_5_evap = (h_1_required - x_phase_separator * h_flash_gas_throttled) / (1 - x_phase_separator)

            self.evaporator.state_outlet = self.med_prop.calc_state("PH", p_1, h_5_evap)

        elif x_phase_separator >= 1.0:
            raise ValueError("No liquid for evaporator.")
        elif x_phase_separator <= 0.0:
            self.mid_pressure_valve.state_inlet = None
            self.mid_pressure_valve.state_outlet = None

            self.low_pressure_valve.state_inlet = self.phase_separator.state_inlet
            self.low_pressure_valve.calc_outlet(p_outlet=p_1)

            self.evaporator.state_inlet = self.low_pressure_valve.state_outlet
            self.evaporator.m_flow = m_flow_total
            self.evaporator.state_outlet = self.med_prop.calc_state("PH", p_1, h_1_required)



        # fs_state.set(
        #     name="p_mid", value=p_mid,
        #     unit="Pa", description="Intermediate pressure"
        # )
        #
        # fs_state.set(
        #     name="T_1", value=self.evaporator.state_outlet.T,
        #     unit="K", description="Refrigerant temperature at evaporator outlet"
        # )
        # fs_state.set(
        #     name="T_2", value=self.compressor.state_outlet.T,
        #     unit="K", description="Compressor outlet temperature"
        # )
        # fs_state.set(
        #     name="T_3", value=self.condenser.state_outlet.T, unit="K",
        #     description="Refrigerant temperature at condenser outlet"
        # )
        # fs_state.set(
        #     name="T_4", value=self.evaporator.state_inlet.T,
        #     unit="K", description="Refrigerant temperature at evaporator inlet"
        # )
        # fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation pressure")
        # fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation pressure")
        # # print("converged")


    # def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
    #     """
    #     This function calculates the states of a standard heat pump under
    #     specific conditions while adhering to several general assumptions.
    #
    #     General Assumptions:
    #     ---------------------
    #     - Isenthalpic expansion valves:
    #       The enthalpy at the inlet equals the enthalpy at the outlet.
    #     - Input to the evaporator is always in the two-phase region.
    #     - Output of the evaporator and output of the condenser maintain
    #       a constant overheating or subcooling (can be set in Inputs).
    #     """
    #
    #     # last_cop = 1
    #     # q_4_step = 0.1
    #     # q_4 = 0.15
    #     #
    #     # while q_4_step > 0.0001:
    #     #     self.set_condenser_outlet_based_on_q(p_con=p_2, inputs=inputs, q_4=q_4, p_eva=p_1)
    #     #     self.expansion_valve.state_inlet = self.condenser.state_outlet
    #     #     self.expansion_valve.calc_outlet(p_outlet=p_1)
    #     #     self.evaporator.state_inlet = self.expansion_valve.state_outlet
    #     #     self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
    #     #     self.compressor.state_inlet = self.evaporator.state_outlet
    #     #     self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
    #     #     self.condenser.state_inlet = self.compressor.state_outlet
    #     #     # Mass flow rate:
    #     #     self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
    #     #     self.condenser.m_flow = self.compressor.m_flow
    #     #     self.evaporator.m_flow = self.compressor.m_flow
    #     #     self.expansion_valve.m_flow = self.compressor.m_flow
    #     #     Q_con = self.condenser.calc_Q_flow()
    #     #     P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)
    #     #     current_cop = Q_con / P_el
    #     #     print(f"COP: {current_cop}; q_4: {q_4}")
    #     #     if current_cop < last_cop:
    #     #         q_4 += q_4_step
    #     #         q_4_step /= 10
    #     #         q_4 -= q_4_step
    #     #         if 0 > q_4 or q_4 > 1:
    #     #             q_4 += q_4_step
    #     #             q_4_step /= 10
    #     #     else:
    #     #         q_4 -= q_4_step
    #     #         if 0 > q_4 or q_4 > 1:
    #     #             q_4 += q_4_step
    #     #             q_4_step /= 10
    #     #     #print("q_4: ", q_4)
    #     #     last_cop = current_cop
    #
    #     # k_phase_separator = inputs.control.get("k_phase_separator", default=1)
    #
    #     p_phase_separator = 5500000
    #
    #     # Calculate low compressor stage to already have access to the mass flow rates.
    #     self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
    #
    #     def get_compressor_input_error(h_1_guess_array):
    #         h_1_guess = h_1_guess_array[0]
    #
    #         # Condenser outlet
    #         self.compressor.state_inlet = self.med_prop.calc_state("PH", p_1, h_1_guess)
    #         self.compressor.calc_state_outlet(
    #             p_outlet=p_2, inputs=inputs, fs_state=fs_state
    #         )
    #
    #
    #         m_flow_high = self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
    #
    #         # Check m_flow of both compressor stages to check if
    #         # there would be an asymmetry of how much refrigerant is transported
    #         # m_flow_high = self.high_pressure_compressor.calc_m_flow(
    #         #    inputs=inputs, fs_state=fs_state
    #         # )
    #         # m_flow_low_should = m_flow_high * (1-x_vapor_injection)
    #         # percent_deviation = (m_flow_low - m_flow_low_should) / m_flow_low_should * 100
    #         # logger.debug("Deviation of mass flow rates is %s percent", percent_deviation)
    #
    #         # Set states
    #         self.condenser.m_flow = self.compressor.m_flow
    #         self.condenser.state_inlet = self.compressor.state_outlet
    #         # Condenser outlet
    #         def get_condenser_error(T_3_guess_array):
    #             T_3_guess = T_3_guess_array[0]
    #             self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_guess)
    #             error, _ = self.condenser.calc(inputs=inputs, fs_state=fs_state)
    #             return error
    #
    #         if inputs.condenser.uses_inlet:
    #             T_con_sec_in = inputs.condenser.T_in
    #             T_3_initial_guess = T_con_sec_in + 3.0
    #         else:
    #             T_con_sec_out = inputs.condenser.T_out
    #             T_3_initial_guess = T_con_sec_out - 3.0
    #
    #         try:
    #             T_3_solution_array, _, ier, _ = fsolve(get_condenser_error, x0=[T_3_initial_guess], xtol=0.01,
    #                                                    full_output=True)
    #
    #             if ier != 1:
    #                 raise ValueError("fsolve_condenser_did_not_converge")
    #
    #             self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_solution_array[0])
    #
    #         except Exception as e:
    #             raise ValueError("fsolve_condenser_did_not_converge") from e
    #
    #         # High pressure EV
    #         self.high_pressure_valve.state_inlet = self.condenser.state_outlet
    #
    #         self.high_pressure_valve.calc_outlet(p_outlet=p_phase_separator)
    #
    #         # Phase Separator component:
    #         x_phase_separator, h_vapor_phase_separator, state_low_ev_outlet, state_mid_ps_outlet = self.calc_separation()
    #         # Low pressure EV
    #         self.low_pressure_valve.state_inlet = state_low_ev_outlet
    #         self.low_pressure_valve.calc_outlet(p_outlet=p_1)
    #         # Mid pressure EV
    #         self.mid_pressure_valve.state_inlet = state_mid_ps_outlet
    #         self.mid_pressure_valve.calc_outlet(p_outlet=p_1)
    #         # Evaporator
    #         self.evaporator.state_inlet = self.low_pressure_valve.state_outlet
    #
    #         # Ideal Mixing of state_5 and state_1_VI:
    #         h_1_VI_mixed = (
    #                 (1 - x_phase_separator) * self.evaporator.state_outlet.h +
    #                 x_phase_separator * self.mid_pressure_valve.state_outlet.h
    #         )
    #
    #         self.evaporator.m_flow = self.compressor.m_flow * (1 - x_phase_separator)
    #
    #         error = (h_1_guess - h_1_VI_mixed)/ h_1_VI_mixed
    #         error = numpy.array([error])
    #         return error
    #
    #     h_compressor_initial_guess = self.evaporator.state_outlet.h
    #
    #     try:
    #         h_1_solution_array, _, ier, _ = fsolve(get_compressor_input_error, x0=[h_compressor_initial_guess], xtol=0.01,
    #                                                full_output=True)
    #
    #         if ier != 1:
    #             raise ValueError("fsolve_compressor_did_not_converge")
    #
    #         self.compressor.state_inlet = self.med_prop.calc_state("PH", p_1, h_1_solution_array[0])
    #
    #     except Exception as e:
    #         raise ValueError("fsolve_compressor_did_not_converge") from e
    #
    #     self.compressor.state_inlet = self.med_prop.calc_state("PH", p_1, h_1_solution_array[0])
    #     self.compressor.calc_state_outlet(
    #         p_outlet=p_2, inputs=inputs, fs_state=fs_state
    #     )
    #
    #     m_flow_high = self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
    #
    #     # Check m_flow of both compressor stages to check if
    #     # there would be an asymmetry of how much refrigerant is transported
    #     # m_flow_high = self.high_pressure_compressor.calc_m_flow(
    #     #    inputs=inputs, fs_state=fs_state
    #     # )
    #     # m_flow_low_should = m_flow_high * (1-x_vapor_injection)
    #     # percent_deviation = (m_flow_low - m_flow_low_should) / m_flow_low_should * 100
    #     # logger.debug("Deviation of mass flow rates is %s percent", percent_deviation)
    #
    #     # Set states
    #     self.condenser.m_flow = self.compressor.m_flow
    #     self.condenser.state_inlet = self.compressor.state_outlet
    #
    #     # Condenser outlet
    #     def get_condenser_error(T_3_guess_array):
    #         T_3_guess = T_3_guess_array[0]
    #         self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_guess)
    #         error, _ = self.condenser.calc(inputs=inputs, fs_state=fs_state)
    #         return error
    #
    #     if inputs.condenser.uses_inlet:
    #         T_con_sec_in = inputs.condenser.T_in
    #         T_3_initial_guess = T_con_sec_in + 3.0
    #     else:
    #         T_con_sec_out = inputs.condenser.T_out
    #         T_3_initial_guess = T_con_sec_out - 3.0
    #
    #     try:
    #         T_3_solution_array, _, ier, _ = fsolve(get_condenser_error, x0=[T_3_initial_guess], xtol=0.01,
    #                                                full_output=True)
    #
    #         if ier != 1:
    #             raise ValueError("fsolve_condenser_did_not_converge")
    #
    #         self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_solution_array[0])
    #
    #     except Exception as e:
    #         raise ValueError("fsolve_condenser_did_not_converge") from e
    #
    #     # High pressure EV
    #     self.high_pressure_valve.state_inlet = self.condenser.state_outlet
    #     self.high_pressure_valve.calc_outlet(p_outlet=p_phase_separator)
    #
    #     # Phase Separator component:
    #     x_phase_separator, h_vapor_phase_separator, state_low_ev_outlet, state_mid_ps_outlet = self.calc_separation()
    #     # Low pressure EV
    #     self.low_pressure_valve.state_inlet = state_low_ev_outlet
    #     self.low_pressure_valve.calc_outlet(p_outlet=p_1)
    #     # Mid pressure EV
    #     self.mid_pressure_valve.state_inlet = state_mid_ps_outlet
    #     self.mid_pressure_valve.calc_outlet(p_outlet=p_1)
    #     # Evaporator
    #     self.evaporator.state_inlet = self.low_pressure_valve.state_outlet
    #
    #     # Ideal Mixing of state_5 and state_1_VI:
    #     h_1_VI_mixed = (
    #             (1 - x_phase_separator) * self.evaporator.state_outlet.h +
    #             x_phase_separator * self.mid_pressure_valve.state_outlet.h
    #     )
    #
    #     self.evaporator.m_flow = self.compressor.m_flow * (1 - x_phase_separator)
    #
    #     #fs_state.set(
    #      #   name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
    #       #  unit="-", description="Expansion valve opening"
    #     #)
    #     fs_state.set(
    #         name="T_1", value=self.evaporator.state_outlet.T,
    #         unit="K", description="Refrigerant temperature at evaporator outlet"
    #     )
    #     fs_state.set(
    #         name="T_2", value=self.compressor.state_outlet.T,
    #         unit="K", description="Compressor outlet temperature"
    #     )
    #     fs_state.set(
    #         name="T_3", value=self.condenser.state_outlet.T, unit="K",
    #         description="Refrigerant temperature at condenser outlet"
    #     )
    #     fs_state.set(
    #         name="T_4", value=self.evaporator.state_inlet.T,
    #         unit="K", description="Refrigerant temperature at evaporator inlet"
    #     )
    #     fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation pressure")
    #     fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation pressure")
        #print("converged")



    def calc_separation(self) -> (float, float, ThermodynamicState, ThermodynamicState):
        # Phase separator
        self.phase_separator.state_inlet = self.high_pressure_valve.state_outlet
        x_phase_separator = self.phase_separator.state_inlet.q
        h_vapor_phase_separator = self.phase_separator.state_outlet_vapor.h
        return x_phase_separator, h_vapor_phase_separator, self.phase_separator.state_outlet_liquid, self.phase_separator.state_outlet_vapor

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
