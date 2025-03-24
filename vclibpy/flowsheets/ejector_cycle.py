import abc

from vclibpy.flowsheets import BaseCycle
from vclibpy.components.expansion_valves.ejector import Ejector
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve


class BaseEjectorCycle(BaseCycle, abc.ABC):
    """
    Class for cycles using an ejector
    
    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "Base Class of all ejector cycles - not for use of HP generation"

    def __init__(self,
                 ejector: Ejector,
                 compressor: Compressor,
                 metering_valve: ExpansionValve,
                 **kwargs):
        super().__init__(**kwargs)
        self.ejector = ejector
        self.compressor = compressor
        self.metering_valve = metering_valve
