import logging
from typing import Union

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

try:
    import CoolProp.CoolProp as CP
except ImportError:
    CP = None

from vclibpy import Inputs, FlowsheetState
from vclibpy.algorithms.base import Algorithm
from vclibpy.flowsheets import BaseCycle

logger = logging.getLogger(__name__)


class Iteration_TC(Algorithm):
    """
    Algorithm to calculate steady states with an iteration based approach.
    (Docstring bleibt unverändert)
    """

    def __init__(self, **kwargs):
        """Initialize class with kwargs"""
        self.min_iteration_step = kwargs.pop("min_iteration_step", 10)  # Min step in Pa
        self.show_iteration = kwargs.get("show_iteration", False)
        self.use_quick_solver = kwargs.pop("use_quick_solver", True)
        self.max_err_dT_min = kwargs.pop("min_allowed_dT_min", 0.01)
        self.max_num_iterations = kwargs.pop("max_num_iterations", int(1e5))
        self.step_max = kwargs.pop("step_max", 10000)
        self.return_least_error_if_max_reached = kwargs.pop("return_least_error_if_max_reached", False)
        super().__init__(**kwargs)

    def _initial_setup(self, flowsheet, inputs, fluid):
        """Helper to avoid code duplication."""
        p_1_next, p_2_next, _p_max, fs_state = self.initial_setup(
            flowsheet=flowsheet, inputs=inputs, fluid=fluid
        )
        try:
            self._p_min = flowsheet.compressor.med_prop.p_min
        except AttributeError:
            self._p_min = 1e3
        return p_1_next, p_2_next, _p_max, fs_state

    def _handle_subcritical(self, flowsheet, inputs, p_1_start, p_2_start, p_max, fs_state):
        """
        Handles the iteration for the subcritical case.
        """
        p_1, p_2 = p_1_start, p_2_start
        step_p1 = self.step_max if self.use_quick_solver else self.min_iteration_step
        step_p2 = self.step_max if self.use_quick_solver else self.min_iteration_step
        error_eva_history = [np.nan]
        error_con_history = [np.nan]

        for i in range(self.max_num_iterations):
            try:
                error_eva, _, error_con, _ = flowsheet.calculate_cycle_for_pressures(
                    p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state
                )
            except ValueError as err:
                logger.error(f"Error during subcritical calculation: {err}")
                return None

            if abs(error_eva) < 1e-4 and abs(error_con) < 1e-4:
                logger.info(f"Subcritical converged after {i + 1} iterations.")
                flowsheet.iteration_converged = True
                fs_state.set(name="converged", value=1, unit="-", description="Algorithm converged (1 true, 0 false)")
                return flowsheet.calculate_outputs_for_valid_pressures(
                    p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state, save_path_plots=self.save_path_plots
                )

            if len(error_eva_history) > 1 and np.sign(error_eva) != np.sign(error_eva_history[-1]):
                step_p1 = max(step_p1 / 5, self.min_iteration_step)
            if len(error_con_history) > 1 and np.sign(error_con) != np.sign(error_con_history[-1]):
                step_p2 = max(step_p2 / 5, self.min_iteration_step)

            error_eva_history.append(error_eva)
            error_con_history.append(error_con)

            p_1 -= np.sign(error_eva) * step_p1
            p_2 += np.sign(error_con) * step_p2

            if p_2 >= p_max:
                p_2 -= step_p2
                step_p2 = max(step_p2 / 10, self.min_iteration_step)

        logger.warning("Subcritical calculation failed to converge within max iterations.")
        return None

    def _handle_transcritical(self, flowsheet, inputs, p_1_start, p_2_start, fs_state):
        """
        Handles the iteration for the transcritical case with the robust nested loop logic.
        """
        p_1 = p_1_start
        p_2 = p_2_start
        step_p2_bar = 5.0
        min_step_p2_bar = 0.1
        p_2_history = []
        cop_history = []
        differential_history = []

        try:
            _, p_crit, _ = flowsheet.compressor.med_prop.get_critical_point()
            p_2_min_limit = p_crit + 1e5
        except (ImportError, AttributeError) as e:
            logger.warning(f"Could not get p_crit automatically ({e}). Using fallback value 74e5 Pa.")
            p_2_min_limit = 74e5

        if self.show_iteration:
            fig_iterations, ax_iterations = plt.subplots(2, 2, figsize=(12, 8))

        for i_outer in range(150):
            if p_2 < p_2_min_limit:
                logger.warning(
                    f"p_2 ({p_2 * 1e-5:.2f} bar) is below the minimum limit ({p_2_min_limit * 1e-5:.2f} bar). Stopping optimization.")
                p_2 = p_2_history[-1] * 1e5 if p_2_history else p_2_start
                flowsheet.iteration_converged = True
                fs_state.set(name="converged", value=1, unit="-", description="Algorithm converged (1 true, 0 false)")
                return flowsheet.calculate_outputs_for_valid_pressures(
                    p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state, save_path_plots=self.save_path_plots
                )

            logger.debug(f"Outer loop {i_outer + 1}: Testing p_2 = {p_2 * 1e-5:.2f} bar")

            p_1_stable = False
            step_p1 = 1e4
            min_step_p1 = 10
            error_eva_history_inner = [np.nan]

            for i_inner in range(100):
                try:
                    error_eva, _, _, _ = flowsheet.calculate_cycle_for_pressures(
                        p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state
                    )
                except ValueError as err:
                    logger.error(f"Inner loop error at p_1={p_1 * 1e-5:.2f}, p_2={p_2 * 1e-5:.2f}: {err}")
                    return None

                if abs(error_eva) < 1e-3:
                    logger.debug(f"Inner loop converged after {i_inner + 1} steps. Stable p_1 = {p_1 * 1e-5:.2f} bar.")
                    p_1_stable = True
                    break

                if i_inner > 0 and np.sign(error_eva) != np.sign(error_eva_history_inner[-1]):
                    step_p1 = max(step_p1 / 2.0, min_step_p1)

                error_eva_history_inner.append(error_eva)
                p_1 += np.sign(error_eva) * step_p1

            if not p_1_stable:
                logger.warning(f"Inner loop for p_1 did not converge for p_2 = {p_2 * 1e-5:.2f} bar. Stopping.")
                return None

            Q_con = flowsheet.condenser.calc_Q_flow()
            P_el = flowsheet.calc_electrical_power(fs_state=fs_state, inputs=inputs)
            if P_el <= 0:
                logger.error("Compressor power is zero or negative, cannot calculate COP.")
                return None
            current_cop = Q_con / P_el

            if not p_2_history:
                p_2_history.append(p_2 * 1e-5)
                cop_history.append(current_cop)
                p_2 -= step_p2_bar * 1e5
                continue

            local_differential = (current_cop - cop_history[-1]) / (p_2 * 1e-5 - p_2_history[-1]) if (p_2 * 1e-5 -
                                                                                                      p_2_history[
                                                                                                          -1]) != 0 else 0
            logger.info(
                f"Outer Iteration {i_outer + 1}: p_2={p_2 * 1e-5:.2f} bar, stable_p_1={p_1 * 1e-5:.2f} bar, COP={current_cop:.4f}, diff={local_differential:.4f}"
            )

            p_2_history.append(p_2 * 1e-5)
            cop_history.append(current_cop)
            differential_history.append(local_differential)

            if self.show_iteration:
                ax_iterations[0, 0].cla();
                ax_iterations[0, 1].cla()
                ax_iterations[0, 0].plot(p_2_history, cop_history, 'o-');
                ax_iterations[0, 0].set_title('COP vs. p_2')
                ax_iterations[0, 1].plot(p_2_history[1:], differential_history, 'o-');
                ax_iterations[0, 1].set_title('Differential vs. p_2')
                plt.tight_layout();
                plt.pause(1e-3)

            if len(differential_history) > 1 and np.sign(differential_history[-1]) != np.sign(differential_history[-2]):
                step_p2_bar = max(step_p2_bar / 3, min_step_p2_bar)

            if step_p2_bar <= min_step_p2_bar:
                logger.info(f"Converged: p_2 step size ({step_p2_bar:.3f} bar) is below tolerance.")
                if self.show_iteration: plt.close(fig_iterations)
                flowsheet.iteration_converged = True
                fs_state.set(name="converged", value=1, unit="-", description="Algorithm converged (1 true, 0 false)")
                return flowsheet.calculate_outputs_for_valid_pressures(
                    p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state, save_path_plots=self.save_path_plots
                )

            p_2 += np.sign(local_differential) * step_p2_bar * 1e5 if local_differential != 0 else -step_p2_bar * 1e5

        logger.warning("Breaking: Max outer iterations reached for p_2 optimization.")
        if self.show_iteration: plt.close(fig_iterations)
        return None

    def calc_steady_state(
            self,
            flowsheet: BaseCycle,
            inputs: Inputs,
            fluid: str = None
    ) -> Union[FlowsheetState, None]:
        flowsheet.iteration_converged = False
        try:
            p_1_start, p_2_start, p_max, fs_state = self._initial_setup(
                flowsheet=flowsheet, inputs=inputs, fluid=fluid
            )
        except TypeError:
            p_1_start, p_2_start = flowsheet.p_1_start, flowsheet.p_2_start
            p_max = 200e5
            fs_state = FlowsheetState()

        if flowsheet.flowsheet_name == "StandardTranscritical":
            return self._handle_transcritical(flowsheet, inputs, p_1_start, p_2_start, fs_state)
        else:
            return self._handle_subcritical(flowsheet, inputs, p_1_start, p_2_start, p_max, fs_state)