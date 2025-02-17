"""
Module with semi-physical model of the transcritical CO2 ejector
"""

from vclibpy.components.component import ThreePortComponent

class Ejector(ThreePortComponent):
    """
    Ejector model according to Zhu 2017:
    'Theoretical model of a transcritical CO2 ejector
    with non-equilibrium phase change correlation'

    Assumptions:

    - flow inside ejector is steady and one-dimensional
    - ejector walls are adiabatic
    - inlet-flow velocity is neglected in the energy-conservation equation
    - isentropic equations are used for the flow except in the mixing process
    - mixing occurs at constant pressure
    - the two-phase flow in the suction chamber, mixing chamber
      and diffuser is homogeneous and is in thermodynamic equilibrium
    - primary nozzle flow becomes sonic in nozzle throat

    For more information on the model refer to the paper
    """

    def __init__(self,
                 d_throat: float,
                 d_mixing: float,):
        super().__init__()
        self.d_throat = d_throat
        self.d_mixing = d_mixing

