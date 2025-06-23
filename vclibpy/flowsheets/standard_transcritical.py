from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
import numpy
import logging
from scipy.optimize import fsolve


class StandardCycleTranscritical(BaseCycle):
    """
    Class for a standard cycle with four components.

    For the standard cycle, we have 4 possible states:

    1. Before compressor, after evaporator
    2. Before condenser, after compressor
    3. Before EV, after condenser
    4. Before Evaporator, after EV
    """

    flowsheet_name = "StandardTranscritical"

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
            expansion_valve: ExpansionValve,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.expansion_valve = expansion_valve

    # Nice to know: get_all_components is a called a method, because it is a function that is defined inside a class
    # get_all_components also exists in the BaseCycle class. Therefore we call the parent function "get_all_components" with super().get_all_components()
    # After that we add the compressor and expansion valve to the list of components.
    # At the end, we should have a list of all components (excluding the fluid for example),
    # and it should look like this: [my_condenser, my_evaporator, my_compressor, my_expansion_valve]

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.expansion_valve
        ]

    # In this function, all the states of the cycle are defined. Compared to the old subcritical cycle
    # we don't have a constant temperature in the two-phase region. Before it was only a return function
    # now it is a given state list.
    def get_states_in_order_for_plotting(self):
        states = [
            self.evaporator.state_inlet,
            self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1),
            self.compressor.state_inlet,
        ]

        # Compared to the subcritical flowsheet, we cannot calculate the inlet and outlet state of the condenser/gas cooler
        # through the quality of the vapor, due to the supercritical state inside the gas cooler
        # Therefore the gas cooler is split into 20 segments and 
        # Interpolate the states between the condenser inlet and outlet
        p = self.condenser.state_inlet.p
        h_in = self.condenser.state_inlet.h
        h_out = self.condenser.state_outlet.h
        h_steps = numpy.linspace(h_in, h_out, 20)

        for h_val in h_steps:
            inter_state = self.med_prop.calc_state("PH", p, h_val)
            states.append(inter_state)

        states.append(self.expansion_valve.state_inlet)
        states.append(self.expansion_valve.state_outlet)
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

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        """
        This function calculates the states of a standard heat pump under
        specific conditions while adhering to several general assumptions.

        General Assumptions:
        ---------------------
        - Isenthalpic expansion valves:
          The enthalpy at the inlet equals the enthalpy at the outlet.
        - Input to the evaporator is always in the two-phase region.
        - Output of the evaporator and output of the condenser maintain
          a constant overheating or subcooling (can be set in Inputs).
        """

        # last_cop = 1
        # q_4_step = 0.1
        # q_4 = 0.15
        #
        # while q_4_step > 0.0001:
        #     self.set_condenser_outlet_based_on_q(p_con=p_2, inputs=inputs, q_4=q_4, p_eva=p_1)
        #     self.expansion_valve.state_inlet = self.condenser.state_outlet
        #     self.expansion_valve.calc_outlet(p_outlet=p_1)
        #     self.evaporator.state_inlet = self.expansion_valve.state_outlet
        #     self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        #     self.compressor.state_inlet = self.evaporator.state_outlet
        #     self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        #     self.condenser.state_inlet = self.compressor.state_outlet
        #     # Mass flow rate:
        #     self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        #     self.condenser.m_flow = self.compressor.m_flow
        #     self.evaporator.m_flow = self.compressor.m_flow
        #     self.expansion_valve.m_flow = self.compressor.m_flow
        #     Q_con = self.condenser.calc_Q_flow()
        #     P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)
        #     current_cop = Q_con / P_el
        #     print(f"COP: {current_cop}; q_4: {q_4}")
        #     if current_cop < last_cop:
        #         q_4 += q_4_step
        #         q_4_step /= 10
        #         q_4 -= q_4_step
        #         if 0 > q_4 or q_4 > 1:
        #             q_4 += q_4_step
        #             q_4_step /= 10
        #     else:
        #         q_4 -= q_4_step
        #         if 0 > q_4 or q_4 > 1:
        #             q_4 += q_4_step
        #             q_4_step /= 10
        #     #print("q_4: ", q_4)
        #     last_cop = current_cop

        # Calling the function from base.py to set the evaporator outlet based on superheating
        # When superheating > 0, the outlet state is calculated based on "PT" so given pressure and outlet temperature.
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor.state_inlet = self.evaporator.state_outlet  # Setting the compressor inlet state to the evaporator outlet state, assuming no losses

        # Calling the function from compressor.py to calculate the compressor outlet state
        # Isentropic state is calculated based on p_2, entropy of inlet state (see self.compressor.state_inlet = self.evaporator.state_outlet)
        # The isentropic efficiency is calculated based on the regression of Mirko Engelpracht.
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet

        # Mass flow rates:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        # print(f"DEBUG (StandardCycleTranscritical): For Inputs '{inputs.get_name()}'") #NEWLY ADDED
        # print(f"DEBUG: Calculated refrigerant mass flow rate: {self.compressor.m_flow} kg/s") #NEWLY ADDED

        # The mass flow in every component is the same, as we assume a closed cycle
        self.condenser.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.expansion_valve.m_flow = self.compressor.m_flow

        # Wir definieren eine "Zielfunktion", die den Kondensatorfehler zurückgibt.
        # Der Solver wird den Input dieser Funktion (T_3_guess) so lange ändern,
        # bis der Output (error) null ist.
        def get_condenser_error(T_3_guess):
            # Setze den angenommenen Austrittszustand am Kondensator
            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_guess)
            # Berechne den resultierenden Fehler mit dem Wärmetauschermodell
            error, _ = self.condenser.calc(inputs=inputs, fs_state=fs_state)
            return error

        # Definiere einen sinnvollen Startwert für die Suche nach T_3
        T_con_sec_in = inputs.condenser.T_in
        T_3_initial_guess = T_con_sec_in + 3.0  # Unsere bewährte 3-K-Annahme als Startpunkt

        try:
            # Führe die Nullstellensuche mit fsolve durch.
            # fsolve gibt ein Array zurück, wir brauchen nur das erste Element.
            T_3_solution, = fsolve(get_condenser_error, x0=T_3_initial_guess, xtol=0.01)

            # Setze den finalen, korrekten Austrittszustand
            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, T_3_solution)
        except Exception as e:
            # Falls fsolve aus anderen Gründen fehlschlägt (z.B. keine Konvergenz)
            logging.error(
                f"Condenser solver 'fsolve' failed to find a solution starting from T_3={T_3_initial_guess:.2f} K. Error: {e}")
            raise

        '''
        # iterate the condenser outlet temperature based on energy balance
        max_err_q = 0.5
        error_history = []
        step_pinch_point = 1
        min_step_pinch_point = 0.001
        pinch_point = 3    # Starting pinch point in K
        self.condenser.state_outlet = self.set_condenser_outlet_based_on_pinch_point(p_2=p_2, inputs=inputs, pinch_point=pinch_point)
        # First iteration outside while loop to get the first error
        error, dT_min = self.condenser.calc(inputs=inputs, fs_state=fs_state)
        error_history.append(error)
        # print(f"Error: {error}, T_con_out: {self.condenser.state_outlet.T}")
        if error > 0:
            pinch_point -= step_pinch_point
        else:
            pinch_point += step_pinch_point
        self.condenser.state_outlet = self.set_condenser_outlet_based_on_pinch_point(p_2=p_2, inputs=inputs,
                                                                                     pinch_point=pinch_point)
        #print(f"Error: {error}, T_con_out: {self.condenser.state_outlet.T}")
        while True:
            error, dT_min = self.condenser.calc(inputs=inputs, fs_state=fs_state)
            error_history.append(error)
            # print(f"Error: {error}, T_con_out: {self.condenser.state_outlet.T}")

            if numpy.sign(error_history[-1]) != numpy.sign(error_history[-2]):
                step_pinch_point /= 10

            if abs(error) > max_err_q or step_pinch_point > min_step_pinch_point:
                if error > 0:
                    pinch_point -= step_pinch_point
                else:
                    pinch_point += step_pinch_point
                self.condenser.state_outlet = self.set_condenser_outlet_based_on_pinch_point(p_2=p_2, inputs=inputs, pinch_point=pinch_point)
            else:
                break
                '''

        '''
        # This is an alternative to the above iteration, which sets a fixed gas cooler outlet temperature
        fixed_gas_cooler_outlet_temp = 35 + 273.15

        try:
            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, fixed_gas_cooler_outlet_temp)
            logging.info(f"Set gas cooler outlet temperature to {fixed_gas_cooler_outlet_temp} K at p={p_2/1e5:.2f} bar.")
        except Exception as e_gc_fixed:
            logging.error(f"Failed to set fixed gas cooler outlet temperature to: {fixed_gas_cooler_outlet_temp} K")
            raise e_gc_fixed

        try:
            error, dT_min = self.condenser.calc(inputs=inputs, fs_state=fs_state)
            logging.info(f"Calculated condenser error: {error}, dT_min: {dT_min}")
        except Exception as e_gc_calc:
            logging.error(f"Failed to calculate condenser state with fixed outlet temperature: {fixed_gas_cooler_outlet_temp} K")
            raise e_gc_calc
        '''

        '''
        user_defined_T_gc_outlet_K = None  # Initialize
        if hasattr(self, 'user_defined_fixed_T_gc_outlet_K') and self.user_defined_fixed_T_gc_outlet_K is not None:
            user_defined_T_gc_outlet_K = self.user_defined_fixed_T_gc_outlet_K
            logging.info(
                f"Using user-defined fixed GC outlet temperature from example script: "
                f"{user_defined_T_gc_outlet_K - 273.15:.2f}°C at p={p_2 / 1e5:.2f}bar"
            )
        else:
            # If no temperature was set in the example script, use a hardcoded default here
            # OR raise an error to remind the user to set it in the example script.
            # For now, using a default and issuing a warning.
            user_defined_T_gc_outlet_K = 40 + 273.15  # Default to 40°C, for example
            logging.warning(
                f"No user-defined fixed GC outlet temperature set in example script! "
                f"Using internal default: {user_defined_T_gc_outlet_K - 273.15:.2f}°C"
            )

        try:
            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_2, user_defined_T_gc_outlet_K)
        except Exception as e_gc_fixed:
            logging.error(
                f"Error setting fixed GC outlet to "
                f"{user_defined_T_gc_outlet_K - 273.15:.2f}°C: {e_gc_fixed}"
            )
            raise  # Re-raise the exception

        try:
            # Important: Call self.condenser.calc() once to update Q_con, secondary side temps, etc.
            # The returned 'error' and 'dT_min' are not used for iteration here.
            error, dT_min = self.condenser.calc(inputs=inputs, fs_state=fs_state)
            logging.info(
                f"GC calculation with fixed outlet: error={error:.2f}%, dT_min={dT_min:.2f}K"
            )
        except Exception as e_gc_calc:
            logging.error(
                f"Error during self.condenser.calc() with fixed outlet: {e_gc_calc}"
            )
            raise  # Re-raise the exception
        '''

        self.expansion_valve.state_inlet = self.condenser.state_outlet
        self.expansion_valve.calc_outlet(p_outlet=p_1)
        self.evaporator.state_inlet = self.expansion_valve.state_outlet
        # print(self.condenser.state_outlet)
        # print(error)

        fs_state.set(
            name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
            unit="-", description="Expansion valve opening"
        )
        fs_state.set(
            name="T_1", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet"
        )
        fs_state.set(
            name="T_2", value=self.compressor.state_outlet.T,
            unit="K", description="Compressor outlet temperature"
        )
        fs_state.set(
            name="T_3", value=self.condenser.state_outlet.T, unit="K",
            description="Refrigerant temperature at condenser outlet"
        )
        fs_state.set(
            name="T_4", value=self.evaporator.state_inlet.T,
            unit="K", description="Refrigerant temperature at evaporator inlet"
        )
        fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation pressure")
        #print("converged")

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
