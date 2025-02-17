from abc import ABC

from vclibpy.flowsheets.ejector_cycle import BaseEjectorCycle
from vclibpy.media import ThermodynamicState


class StandardEjectorCycle(BaseEjectorCycle, ABC):

    def __init__(self):
        super().__init__()
