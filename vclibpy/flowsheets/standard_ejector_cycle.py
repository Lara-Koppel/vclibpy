import math
import logging
from abc import ABC

import numpy as np

from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.phase_separator import PhaseSeparator
from vclibpy.flowsheets import BaseEjectorCycle
from vclibpy.media import ThermodynamicState
from vclibpy import Inputs, FlowsheetState


logger = logging.getLogger(__name__)


class StandardEjectorCycle(BaseEjectorCycle, ABC):
    """
    Class for the standard ejector cycle

    For this cycle, we have 7 relevant states:

    - 1: Before compressor, after phase separator
    - 2: Before condenser, after compressor
    - 3: Before ejector, after condenser
    - 4: Before ejector, after evaporator
    - 5: Before phase separator, after ejector
    - 6: Before metering valve, after phase separator
    - 7: Before evaporator, after metering valve

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    def __init__(self, phase_seperator: PhaseSeparator, **kwargs):
        """Initialize class with kwargs"""
        self.max_err = kwargs.pop("max_err", 0.5)
        self.min_iteration_step = kwargs.pop("min_iteration_step", 1)
        self.show_iteration = kwargs.get("show_iteration", False)
        self.use_quick_solver = kwargs.pop("use_quick_solver", True)
        self.max_num_iterations = kwargs.pop("max_num_iterations", int(1e5))
        self.step_max = kwargs.pop("step_max", 10000)
        self.phase_seperator = phase_seperator
        super().__init__(**kwargs)

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        # Condenser outlet
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)  # ToDO: Implement transcritical cycle

        # Ejector
        self.ejector.state_primary = self.condenser.state_outlet
        # in the standard cycle the secondary inlet is the vapour from the evaporator outlet
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.ejector.state_secondary = self.evaporator.state_outlet

        p_3 = self.iterate_p3(p_1, p_2, inputs, fs_state)
        if p_3 == -1:
            return
        # Phase Separator
        self.phase_seperator.state_inlet = self.ejector.state_outlet

        # # Compressor
        # self.compressor.state_inlet = self.phase_seperator.state_outlet_vapor
        # self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)

        # Condenser inlet
        self.condenser.state_inlet = self.compressor.state_outlet

        # Metering Valve
        self.metering_valve.state_inlet = self.phase_seperator.state_outlet_liquid
        self.metering_valve.calc_outlet(p_outlet=p_1)

        # Evaporator
        self.evaporator.state_inlet = self.metering_valve.state_outlet

        # Mass flow rates:
        self.condenser.m_flow = self.ejector.m_flow_outlet * self.ejector.state_outlet.q
        self.metering_valve.m_flow = self.ejector.m_flow_outlet * (1 - self.ejector.state_outlet.q)
        self.evaporator.m_flow = self.metering_valve.m_flow

        fs_state.set(
            name="T_1", value=self.compressor.state_inlet.T,
            unit="K", description="Refrigerant temperature at compressor inlet"
        )
        fs_state.set(
            name="T_2", value=self.compressor.state_outlet.T,
            unit="K", description="Refrigerant temperature at compressor outlet"
        )
        fs_state.set(
            name="T_3", value=self.condenser.state_outlet.T,
            unit="K", description="Refrigerant temperature at condenser outlet"
        )
        fs_state.set(
            name="T_4", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet"
        )
        fs_state.set(
            name="h_mixing", value=self.ejector.state_mixing.h,
            unit="J/kgK", description="Specific enthalpy at mixing chamber outlet"
        )
        fs_state.set(
            name="h_ejector", value=self.ejector.state_outlet.h,
            unit="J/kgK", description="Specific enthalpy at ejector outlet"
        )
        fs_state.set(
            name="p_con", value=p_2,
            unit="Pa", description="Condensation pressure"
        )
        fs_state.set(
            name="p_eva", value=p_1,
            unit="Pa", description="Evaporation pressure"
        )
        fs_state.set(
            name="p_middle", value=p_3,
            unit="Pa", description="Pressure between ejector and compressor"
        )

    def iterate_p3(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        """
        Iterate p_3 to get matching m_flow for ejector inlets and outlet
        """
        # Settings
        if self.use_quick_solver:
            step_p3 = self.step_max
        else:
            step_p3 = self.min_iteration_step

        p_3_history = []
        error_m_flow_history = []
        error_m_flow = np.nan
        num_iterations = 0

        # iterate p_ejector_outlet = p_3 to get matching m_flow from ejector and compressor
        p_3 = p_1 + 1e5  # ToDO: How to gues p_3?
        while True:
            if isinstance(self.max_num_iterations, (int, float)):
                if num_iterations > self.max_num_iterations:
                    logger.warning("Maximum number of iterations for p_3 %s exceeded. Stopping.",
                                   self.max_num_iterations)
                    return

                if (num_iterations + 1) % (0.1 * self.max_num_iterations) == 0:
                    logger.info("Info: %s percent of max_num_iterations for p_3 %s used",
                                100 * (num_iterations + 1) / self.max_num_iterations, self.max_num_iterations)

            if p_3 < p_1:
                logger.error("p_3 is smaller than p_1 - Configuration is infeasible. Stopping.")
                return -1

            # increase counter
            num_iterations += 1

            m_flow_ejector = self.ejector.calc_m_flow(p_3)
            m_flow_ejector_vapor = m_flow_ejector * self.ejector.state_outlet.q
            error_m_flow = m_flow_ejector_vapor - self.ejector.m_flow_primary
            error_m_flow_history.append(error_m_flow)
            p_3_history.append(p_3 / 1e5)

            # print(f"p_3 Iteration {num_iterations}: p_3 = {p_3}, error_m_flow = {error_m_flow}")
            # print(p_1, p_2)

            if abs(error_m_flow) > self.max_err or step_p3 > self.min_iteration_step:
                if num_iterations > 2 and np.sign(error_m_flow_history[-1]) != np.sign(error_m_flow_history[-2]):
                    step_p3 /= 10
                    # print(f"Last errors: {error_m_flow_history[-1]}, {error_m_flow_history[-2]}; decreasing step to {step_p3}")
                if error_m_flow < 0:
                    p_3 -= step_p3
                    continue
                else:
                    p_3 += step_p3
                    continue


            logger.info("Breaking: p_3 Converged")
            break
        print(f"p_3 converged after {num_iterations} iterations. p_3 = {p_3}")
        self.compressor.state_inlet = self.med_prop.calc_state("PQ", p_3, 1)
        self.compressor.calc_state_outlet(p_2, inputs, fs_state)
        self.compressor.m_flow = self.ejector.m_flow_outlet * self.ejector.state_outlet.q  #ToDO: Calculate corresponding n, as it is fixed for stationary operation
        return p_3

    def iterate_p3_old(self, p_1, p_2, inputs, fs_state):
        """Iterate p_3 to get matching m_flow from ejector and compressor - does not work"""
        # Settings
        if self.use_quick_solver:
            step_p3 = self.step_max
        else:
            step_p3 = self.min_iteration_step

        p_3_history = []
        error_m_flow_history = []
        error_m_flow = np.nan
        num_iterations = 0

        # iterate p_ejector_outlet = p_3 to get matching m_flow from ejector and compressor
        p_3 = p_1+1e5  #  ToDO: How to gues p_3?
        while True:
            if isinstance(self.max_num_iterations, (int, float)):
                if num_iterations > self.max_num_iterations:
                    logger.warning("Maximum number of iterations for p_3 %s exceeded. Stopping.",
                                   self.max_num_iterations)
                    return

                if (num_iterations + 1) % (0.1 * self.max_num_iterations) == 0:
                    logger.info("Info: %s percent of max_num_iterations for p_3 %s used",
                                100 * (num_iterations + 1) / self.max_num_iterations, self.max_num_iterations)

            if p_3 < p_1:
                logger.error("p_3 is smaller than p_1 - Configuration is infeasible. Stopping.")
                return -1

            # increase counter
            num_iterations += 1

            m_flow_ejector = self.ejector.calc_m_flow(p_3)
            m_flow_ejector_vapor = m_flow_ejector * self.ejector.state_outlet.q
            self.compressor.state_inlet = self.med_prop.calc_state("PQ", p_3, 1)
            self.compressor.calc_state_outlet(p_2, inputs, fs_state)
            m_flow_compressor = self.compressor.calc_m_flow(inputs, fs_state)
            error_m_flow = m_flow_ejector_vapor - m_flow_compressor
            error_m_flow_history.append(error_m_flow)
            p_3_history.append(p_3 / 1e5)

            #print(f"Iteration {num_iterations}: p_3 = {p_3}, error_m_flow = {error_m_flow}")
            #print(p_1, p_2)

            if abs(error_m_flow) > self.max_err or step_p3 > self.min_iteration_step:
                if num_iterations > 2 and np.sign(error_m_flow_history[-1]) != np.sign(error_m_flow_history[-2]):
                    step_p3 /= 10
                    #print(f"Last errors: {error_m_flow_history[-1]}, {error_m_flow_history[-2]}; decreasing step to {step_p3}")
                if error_m_flow < 0:
                    p_3 -= step_p3
                    continue
                else:
                    p_3 += step_p3
                    continue

            # If still here, and the values are equal, we may break.
            #if p_3 == p_3_next:
                # Check if solution was too far away. If so, jump back
                # And decrease the iteration step by factor 10.
                #if step_p3 > self.min_iteration_step:
                    #p_3_next = p_3 + step_p3
                    #step_p3 /= 10
                    #continue
            logger.info("Breaking: p_3 Converged")
            break
            # Check if values are not converging at all:
            #p_3_unique = set(p_3_history[-10:])
            #if len(p_3_unique) == 2 and step_p3 == self.min_iteration_step:
            #    logger.critical("Breaking: p_3 not converging at all")
            #    break
        #print(f"p_3 converged after {num_iterations} iterations. p_3 = {p_3}")
        return p_3

    def get_all_components(self):
        return super().get_all_components() + [
            self.phase_seperator
        ]

    def get_states_in_order_for_plotting(self):
        return [
            self.ejector.state_outlet,
            self.compressor.state_inlet,
            self.compressor.state_outlet,
            self.ejector.state_primary,
            self.ejector.state_throat,
            self.ejector.state_primary_mixing,
            self.ejector.state_mixing,
            self.ejector.state_outlet,
            self.metering_valve.state_inlet,
            self.metering_valve.state_outlet,
            self.evaporator.state_outlet,
            self.ejector.state_secondary,
            self.ejector.state_mixing
        ]

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
