import abc

from vclibpy import Inputs, FlowsheetState
from vclibpy.flowsheets import BaseCycle

class BaseEjectorCycle(BaseCycle, abc.ABC)
    """
    Class for cycles using an ejector
    
    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

