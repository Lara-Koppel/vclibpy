import abc
import logging
from copy import deepcopy
import numpy as np

#from docs.jupyter_notebooks.e7_vapor_injection import high_pressure_valve
from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import Inputs, FlowsheetState
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.phase_separator import PhaseSeparator
from vclibpy.media import ThermodynamicState

logger = logging.getLogger(__name__)


class BasePhaseSeparator(BaseCycle, abc.ABC):
    """
    Cycle using an adiabatic ideal phase seperator.

    For this cycle, we have 9 relevant states:

    - 1: Before compressor, after evaporator
    - 2: Before condenser, after compressor
    - 3: Before first EV, after condenser
    - 4: Before Evaporator, after second EV
    - 5_ps: Before PS, after first EV
    - 6_ps: Before mid EV, after PS
    - 7_ps: Before second EV, after PS
    - 8_ps: Before Mixing with 1, after mid EV
    - 1_PS_mixed. Before compressor, after mixing with 8_vips

    Additional Assumptions:
    -----------------------
    - Ideal mixing in compressor of state 5 and state 4

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "PhaseSeparator"

    def __init__(
            self,
            compressor: Compressor,
            high_pressure_valve: ExpansionValve,
            low_pressure_valve: ExpansionValve,
            mid_pressure_valve: ExpansionValve,
            **kwargs):
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

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.high_pressure_valve,
            self.low_pressure_valve,
            self.mid_pressure_valve,
            self.phase_separator,
        ]

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState, **kwargs):
        k_phase_separator = inputs.control.get("k_phase_separator", default=1)
        # Default according to Xu, 2019

        p_phase_separator = 0.7 * p_2

        # Condenser outlet
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        # High pressure EV
        self.high_pressure_valve.state_inlet = self.condenser.state_outlet
        self.high_pressure_valve.calc_outlet(p_outlet=p_phase_separator)


        # Calculate low compressor stage to already have access to the mass flow rates.
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)

        # Phase Separator component:
        x_phase_separator, h_vapor_phase_separator, state_low_ev_inlet, state_mid_ps_outlet = self.calc_separation()
        # Low pressure EV
        self.low_pressure_valve.state_inlet = state_low_ev_inlet
        self.low_pressure_valve.calc_outlet(p_outlet=p_1)
        # Mid pressure EV
        self.mid_pressure_valve.state_inlet = state_mid_ps_outlet
        self.mid_pressure_valve.calc_outlet(p_outlet=p_1)
        # Evaporator
        self.evaporator.state_inlet = self.low_pressure_valve.state_outlet

        # Ideal Mixing of state_5 and state_1_VI:
        h_1_VI_mixed = (
                (1-x_phase_separator) * self.evaporator.state_outlet.h +
                x_phase_separator * self.mid_pressure_valve.state_outlet.h
        )
        self.compressor.state_inlet = self.med_prop.calc_state(
            "PH", p_1, h_1_VI_mixed
        )
        self.compressor.calc_state_outlet(
            p_outlet=p_2, inputs=inputs, fs_state=fs_state
        )
        # Set the state of the compressor inlet based on the phase separator outlet

        m_flow_high = self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)

        self.evaporator.m_flow = self.compressor.m_flow * (1 - x_phase_separator)


        # Check m_flow of both compressor stages to check if
        # there would be an asymmetry of how much refrigerant is transported
        #m_flow_high = self.high_pressure_compressor.calc_m_flow(
        #    inputs=inputs, fs_state=fs_state
        #)
        #m_flow_low_should = m_flow_high * (1-x_vapor_injection)
        #percent_deviation = (m_flow_low - m_flow_low_should) / m_flow_low_should * 100
        #logger.debug("Deviation of mass flow rates is %s percent", percent_deviation)

        # Set states
        self.condenser.m_flow = self.compressor.m_flow
        self.condenser.state_inlet = self.compressor.state_outlet

        fs_state.set(
            name="T_1", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet"
        )
        fs_state.set(
            name="T_2", value=self.compressor.state_outlet.T,
            unit="K", description="Compressor outlet temperature"
        )
        fs_state.set(
            name="T_3", value=self.condenser.state_outlet.T,
            unit="K", description="Refrigerant temperature at condenser outlet"
        )
        fs_state.set(
            name="T_4", value=self.evaporator.state_inlet.T,
            unit="K", description="Refrigerant temperature at evaporator inlet"
        )
        fs_state.set(
            name="p_con", value=p_2,
            unit="Pa", description="Condensation pressure"
        )
        fs_state.set(
            name="p_eva", value=p_1,
            unit="Pa", description="Evaporation pressure"
        )

    def calc_separation(self) -> (float, float, ThermodynamicState, ThermodynamicState):
        # Phase separator
        self.phase_separator.state_inlet = self.high_pressure_valve.state_outlet
        x_phase_separator = self.phase_separator.state_inlet.q
        h_vapor_phase_separator = self.phase_separator.state_outlet_vapor.h
        return x_phase_separator, h_vapor_phase_separator, self.phase_separator.state_outlet_liquid, self.phase_separator.state_outlet_vapor

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        P_el = self.compressor.calc_electrical_power(
            inputs=inputs, fs_state=fs_state
        )

        fs_state.set(
            name="P_el",
            value=P_el,
            unit="W",
            description="Electrical power consumption of compressor"
        )

        return P_el

    def get_states_in_order_for_plotting(self):
        """
        List with all relevant states of two-stage cycle
        except the intermediate component, e.g. phase separator
        or heat exchanger.
        """
        return [
            self.low_pressure_valve.state_inlet,
            self.low_pressure_valve.state_outlet,
            self.evaporator.state_inlet,
            self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1),
            self.evaporator.state_outlet,
            self.compressor.state_inlet,
            self.compressor.state_outlet,
            self.condenser.state_inlet,
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 1),
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 0),
            self.condenser.state_outlet,
            self.high_pressure_valve.state_inlet,
            self.high_pressure_valve.state_outlet,
            self.phase_separator.state_inlet,
            self.phase_separator.state_outlet_vapor,
            self.mid_pressure_valve.state_inlet,
            self.mid_pressure_valve.state_outlet,
            self.compressor.state_inlet,
            # Go back to separator for clear lines
            self.mid_pressure_valve.state_outlet,
            self.mid_pressure_valve.state_inlet,
            self.phase_separator.state_outlet_vapor,
            self.phase_separator.state_outlet_liquid
        ]