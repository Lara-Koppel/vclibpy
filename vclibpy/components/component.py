from abc import ABC
from vclibpy.media import ThermodynamicState, MedProp


class BaseComponent(ABC):
    """
    Abstract base class for defining interfaces of components in the vapor compression cycle
    with no inlets or outlets.

    Properties:
        med_prop (MedProp):
            Property for accessing and setting the property wrapper for the working fluid.
    """

    def __init__(self):
        """
        Initialize the BaseComponent.
        """
        super(BaseComponent, self).__init__()
        self._med_prop: MedProp = None

    @property
    def med_prop(self) -> MedProp:
        """
        Get or set the property wrapper for the working fluid.

        Returns:
            MedProp: Property wrapper for the working fluid.
        """
        return self._med_prop

    @med_prop.setter
    def med_prop(self, med_prop: MedProp):
        """
        Set the property wrapper for the working fluid.

        Args:
            med_prop (MedProp): Property wrapper to set.
        """
        self._med_prop = med_prop


class TwoPortComponent(BaseComponent):
    """
     Abstract base class for defining interfaces of components in the vapor compression cycle
     with only one input and one output.

     Properties:
         state_inlet (ThermodynamicState):
             Property for accessing and setting the inlet state of the component.
         state_outlet (ThermodynamicState):
             Property for accessing and setting the outlet state of the component.
         m_flow (float):
             Property for accessing and setting the mass flow rate through the component.
         med_prop (MedProp):
             Property for accessing and setting the property wrapper for the working fluid.
     """

    def __init__(self):
        """
        Initialize the TwoPortComponent.
        """
        super(TwoPortComponent, self).__init__()
        self._state_inlet: ThermodynamicState = None
        self._state_outlet: ThermodynamicState = None
        self._m_flow: float = None

    @property
    def state_inlet(self) -> ThermodynamicState:
        """
        Get or set the inlet state of the component.

        Returns:
            ThermodynamicState: Inlet state of the component.
        """
        return self._state_inlet

    @state_inlet.setter
    def state_inlet(self, state_inlet: ThermodynamicState):
        """
        Set the inlet state of the component.

        Args:
            state_inlet (ThermodynamicState): Inlet state to set.
        """
        self._state_inlet = state_inlet

    @property
    def state_outlet(self) -> ThermodynamicState:
        """
        Get or set the outlet state of the component.

        Returns:
            ThermodynamicState: Outlet state of the component.
        """
        return self._state_outlet

    @state_outlet.setter
    def state_outlet(self, state_outlet: ThermodynamicState):
        """
        Set the outlet state of the component.

        Args:
            state_outlet (ThermodynamicState): Outlet state to set.
        """
        self._state_outlet = state_outlet

    @property
    def m_flow(self) -> float:
        """
        Get or set the mass flow rate through the component.

        Returns:
            float: Mass flow rate through the component.
        """
        return self._m_flow

    @m_flow.setter
    def m_flow(self, m_flow: float):
        """
        Set the mass flow rate through the component.

        Args:
            m_flow (float): Mass flow rate to set.
        """
        self._m_flow = m_flow


class ThreePortComponent(BaseComponent):
    """
    Component with three ports, typically two inputs and one output

    Properties:
         state_primary (ThermodynamicState):
             Property for accessing and setting the primary inlet state of the component.
         state_secondary (ThermodynamicState):
             Property for accessing and setting the secondary inlet state of the component.
         state_output (ThermodynamicState):
             Property for accessing and setting the output state of the component.
         m_flow (float):
             Property for accessing and setting the mass flow rate through the component.
         med_prop (MedProp):
             Property for accessing and setting the property wrapper for the working fluid.k
    """

    def __init__(self):
        """
        initialize the ThreePortComponent
        """
        super(ThreePortComponent, self).__init__()
        self._state_primary: ThermodynamicState = None
        self._state_secondary: ThermodynamicState = None
        self._state_outlet: ThermodynamicState = None
        self._m_flow_primary: float = None
        self._m_flow_secondary: float = None
        self._m_flow_outlet: float = None

    @property
    def state_primary(self) -> ThermodynamicState:
        """
        Get or set the state of the component
        on the primary inlet.

        Returns:
            ThermodynamicState: Primary inlet state of the component.
        """
        return self._state_primary

    @state_primary.setter
    def state_primary(self, state_primary: ThermodynamicState):
        """
        Set the state of the component
        on the primary inlet side.

        Args:
            state_primary (ThermodynamicState): Primary inlet state to set.
        """
        self._state_primary = state_primary

    @property
    def state_secondary(self) -> ThermodynamicState:
        """
        Get or set the state of the component
        on the secondary inlet.

        Returns:
            ThermodynamicState: Secondary inlet state of the component.
        """
        return self._state_secondary

    @state_secondary.setter
    def state_secondary(self, state_secondary: ThermodynamicState):
        """
        Set the state of the component
        on the secondary inlet side.

        Args:
            state_secondary (ThermodynamicState): Secondary inlet state to set.
        """
        self._state_secondary = state_secondary

    @property
    def state_outlet(self) -> ThermodynamicState:
        """
        Get or set the state of the component
        on the outlet.

        Returns:
            ThermodynamicState: Outlet state of the component.
        """
        return self._state_outlet

    @state_outlet.setter
    def state_outlet(self, state_outlet: ThermodynamicState):
        """
        Set the state of the component
        on the outlet side.

        Args:
            state_outlet (ThermodynamicState): Outlet state to set.
        """
        self._state_outlet = state_outlet

    @property
    def m_flow_primary(self) -> float:
        """
        Get or set the mass flow rate through the component
        on the primary inlet side.

        Returns:
            float: Mass flow rate through the primary inlet.
        """
        return self._m_flow_primary

    @m_flow_primary.setter
    def m_flow_primary(self, m_flow_primary: float):
        """
        Set the mass flow rate through the component
        on the primary inlet side.

        Args:
            m_flow_primary (float): Mass flow rate to set.
        """
        self._m_flow_primary = m_flow_primary

    @property
    def m_flow_secondary(self) -> float:
        """
        Get or set the mass flow rate through the component
        on the secondary inlet side.

        Returns:
            float: Mass flow rate through the secondary inlet.
        """
        return self._m_flow_secondary

    @m_flow_secondary.setter
    def m_flow_secondary(self, m_flow_secondary: float):
        """
        Set the mass flow rate through the component
        on the secondary inlet side.

        Args:
            m_flow_secondary (float): Mass flow rate to set.
        """
        self._m_flow_secondary = m_flow_secondary

    @property
    def m_flow_outlet(self) -> float:
        """
        Get or set the mass flow rate through the component
        on the outlet side.

        Returns:
            float: Mass flow rate through the component.
        """
        return self._m_flow_outlet

    @m_flow_outlet.setter
    def m_flow_outlet(self, m_flow_outlet: float):
        """
        Set the mass flow rate through the component
        on the outlet side.

        Args:
            m_flow_outlet (float): Mass flow rate to set.
        """
        self._m_flow_outlet = m_flow_outlet


class FourPortComponent(BaseComponent):
    """
    Component with four ports, typically with a
    high and a low pressure / temperature side.
    """

    def __init__(self):
        """
        Initialize the FourPortComponent.
        """
        super(FourPortComponent, self).__init__()
        self._state_inlet_low: ThermodynamicState = None
        self._state_outlet_low: ThermodynamicState = None
        self._state_inlet_high: ThermodynamicState = None
        self._state_outlet_high: ThermodynamicState = None
        self._m_flow_high: float = None
        self._m_flow_low: float = None

    @property
    def state_inlet_high(self) -> ThermodynamicState:
        """
        Get or set the inlet state of the component
        on the high pressure / temperature side.

        Returns:
            ThermodynamicState: Inlet state of the component.
        """
        return self._state_inlet_high

    @state_inlet_high.setter
    def state_inlet_high(self, state_inlet_high: ThermodynamicState):
        """
        Set the inlet state of the component
        on the high pressure / temperature side.

        Args:
            state_inlet_high (ThermodynamicState): Inlet state to set.
        """
        self._state_inlet_high = state_inlet_high

    @property
    def state_outlet_high(self) -> ThermodynamicState:
        """
        Get or set the outlet state of the component
        on the high pressure / temperature side.

        Returns:
            ThermodynamicState: Outlet state of the component.
        """
        return self._state_outlet_high

    @state_outlet_high.setter
    def state_outlet_high(self, state_outlet_high: ThermodynamicState):
        """
        Set the outlet state of the component
        on the high pressure / temperature side.

        Args:
            state_outlet (ThermodynamicState): Outlet state to set.
        """
        self._state_outlet_high = state_outlet_high

    @property
    def m_flow_high(self) -> float:
        """
        Get or set the mass flow rate through the component
        on the high pressure / temperature side.

        Returns:
            float: Mass flow rate through the component.
        """
        return self._m_flow_high

    @m_flow_high.setter
    def m_flow_high(self, m_flow_high: float):
        """
        Set the mass flow rate through the component
        on the high pressure / temperature side.

        Args:
            m_flow_high (float): Mass flow rate to set.
        """
        self._m_flow_high = m_flow_high

    @property
    def state_inlet_low(self) -> ThermodynamicState:
        """
        Get or set the inlet state of the component
        on the low pressure / temperature side.

        Returns:
            ThermodynamicState: Inlet state of the component.
        """
        return self._state_inlet_low

    @state_inlet_low.setter
    def state_inlet_low(self, state_inlet_low: ThermodynamicState):
        """
        Set the inlet state of the component
        on the low pressure / temperature side.

        Args:
            state_inlet_low (ThermodynamicState): Inlet state to set.
        """
        self._state_inlet_low = state_inlet_low

    @property
    def state_outlet_low(self) -> ThermodynamicState:
        """
        Get or set the outlet state of the component.

        Returns:
            ThermodynamicState: Outlet state of the component.
        """
        return self._state_outlet_low

    @state_outlet_low.setter
    def state_outlet_low(self, state_outlet_low: ThermodynamicState):
        """
        Set the outlet state of the component.

        Args:
            state_outlet_low (ThermodynamicState): Outlet state to set.
        """
        self._state_outlet_low = state_outlet_low

    @property
    def m_flow_low(self) -> float:
        """
        Get or set the mass flow rate through the component
        on the low pressure / temperature side.

        Returns:
            float: Mass flow rate through the component.
        """
        return self._m_flow_low

    @m_flow_low.setter
    def m_flow_low(self, m_flow_low: float):
        """
        Set the mass flow rate through the component
        on the low pressure / temperature side.

        Args:
            m_flow_low (float): Mass flow rate to set.
        """
        self._m_flow_low = m_flow_low
