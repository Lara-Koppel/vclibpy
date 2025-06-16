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
        Handles the iteration for the transcritical case with robust convergence and plotting.
        """
        p_1 = p_1_start
        p_2 = p_2_start
        step_p2_bar = 5.0
        min_step_p2_bar = 0.1

        # History-Listen
        p_1_history, p_2_history, cop_history, error_eva_history, dT_eva_history, dT_con_history, differential_history = (
        [] for _ in range(7))

        last_converged_cop = np.nan
        last_p2_bar = np.nan

        try:
            _, p_crit, _ = flowsheet.compressor.med_prop.get_critical_point()
            p_2_min_limit = p_crit + 1e5
        except (ImportError, AttributeError) as e:
            logger.warning(f"Could not get p_crit automatically ({e}). Using fallback value 74e5 Pa.")
            p_2_min_limit = 74e5

        if self.show_iteration:
            plt.ion()
            fig_iterations, ax_iterations = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
            ax_labels = [("error_eva in %", "COP in -"),
                         ("$\Delta T_\mathrm{Min, Eva}$ in K", "$\Delta T_\mathrm{Min, Con}$ in K"),
                         ("$p_1$ in bar", "$p_2$ in bar")]
            for i, row in enumerate(ax_iterations):
                for j, ax in enumerate(row):
                    ax.set_ylabel(ax_labels[i][j])
                    ax.grid(True)
                    if i == 2: ax.set_xlabel("Total Iteration Step")
            plt.tight_layout(pad=2.0)

        for i_outer in range(150):
            if p_2 < p_2_min_limit:
                if self.show_iteration:
                    if self.save_path_plots:
                        filepath = self.save_path_plots.joinpath(f"{inputs.get_name()}_convergence_plot_LOW_P2.png")
                        fig_iterations.savefig(filepath)
                    plt.close(fig_iterations)
                return None

            p_1_stable = False
            step_p1 = 1e4
            min_step_p1 = 10
            error_eva_history_inner = [np.nan]

            for i_inner in range(100):
                try:
                    error_eva, dT_min_eva, error_con, dT_min_con = flowsheet.calculate_cycle_for_pressures(p_1=p_1,
                                                                                                           p_2=p_2,
                                                                                                           inputs=inputs,
                                                                                                           fs_state=fs_state)
                    Q_con = flowsheet.condenser.calc_Q_flow();
                    P_el = flowsheet.calc_electrical_power(fs_state=fs_state, inputs=inputs)
                    current_cop = Q_con / P_el if P_el > 0 else np.nan
                except Exception:
                    p_1 += min_step_p1
                    continue

                # Datensammlung bei JEDEM inneren Schritt
                current_values = [p_1 * 1e-5, p_2 * 1e-5, current_cop, error_eva, dT_min_eva, dT_min_con]
                all_histories = [p_1_history, p_2_history, cop_history, error_eva_history, dT_eva_history,
                                 dT_con_history]
                for lst, val in zip(all_histories, current_values): lst.append(val)

                if self.show_iteration and (i_inner % 10 == 0 or i_inner < 5):
                    plot_data_map = {0: error_eva_history, 1: cop_history, 2: dT_eva_history, 3: dT_con_history,
                                     4: p_1_history, 5: p_2_history}
                    plot_window = 50
                    total_steps = len(p_1_history)
                    iterations_to_plot = range(max(0, total_steps - plot_window), total_steps)
                    for i, ax in enumerate(ax_iterations.flatten()):
                        ax.cla();
                        ax.plot(iterations_to_plot, plot_data_map[i][-plot_window:], 'o-', markersize=2);
                        ax.grid(True)
                    for i, row in enumerate(ax_iterations):
                        for j, ax in enumerate(row):
                            ax.set_ylabel(ax_labels[i][j])
                            if i == 2: ax.set_xlabel("Total Iteration Step")
                    plt.tight_layout(pad=2.0)
                    plt.pause(0.01)

                # --- ROBUSTES ABBRUCHKRITERIUM ---
                is_converged_by_accuracy = abs(error_eva) < 1e-3
                is_oscillating = i_inner > 0 and np.sign(error_eva) != np.sign(error_eva_history_inner[-1])
                if is_oscillating: step_p1 = max(step_p1 / 2.0, min_step_p1)
                is_stuck_at_min_step = step_p1 <= min_step_p1

                if is_converged_by_accuracy or (is_oscillating and is_stuck_at_min_step):
                    logger.info(
                        f"Inner loop converged for p_2={p_2 * 1e-5:.2f} bar after {i_inner + 1} steps. Final p_1={p_1 * 1e-5:.2f} bar.")
                    p_1_stable = True
                    break  # ERFOLG! Verlasse die innere Schleife.

                error_eva_history_inner.append(error_eva)
                p_1 += np.sign(error_eva) * step_p1

            if not p_1_stable:
                logger.warning(
                    f"Inner loop for p_1 did not converge for p_2 = {p_2 * 1e-5:.2f} bar. Stopping outer loop.")
                if self.show_iteration and self.save_path_plots:
                    fig_iterations.savefig(
                        self.save_path_plots.joinpath(f"{inputs.get_name()}_convergence_plot_FAILED.png"))
                if self.show_iteration: plt.close(fig_iterations)
                return None

            # --- ÄUßERE SCHLEIFENLOGIK (wird jetzt erreicht) ---
            final_cop_for_this_p2 = cop_history[-1]
            p2_bar_current = p_2 * 1e-5
            if not np.isnan(last_p2_bar):
                delta_p2 = p2_bar_current - last_p2_bar
                local_differential = (final_cop_for_this_p2 - last_converged_cop) / delta_p2 if abs(
                    delta_p2) > 1e-9 else 0
                differential_history.append(local_differential)

                if len(differential_history) > 1 and np.sign(differential_history[-1]) != np.sign(
                        differential_history[-2]):
                    step_p2_bar = max(step_p2_bar / 3, min_step_p2_bar)

                if step_p2_bar <= min_step_p2_bar:
                    logger.info("Outer loop converged: p_2 step size is below tolerance.")
                    if self.show_iteration and self.save_path_plots:
                        fig_iterations.savefig(
                            self.save_path_plots.joinpath(f"{inputs.get_name()}_convergence_plot_SUCCESS.png"))
                    if self.show_iteration: plt.close(fig_iterations)
                    flowsheet.iteration_converged = True;
                    fs_state.set(name="converged", value=1, unit="-")
                    return flowsheet.calculate_outputs_for_valid_pressures(p_1=p_1, p_2=p_2, inputs=inputs,
                                                                           fs_state=fs_state,
                                                                           save_path_plots=self.save_path_plots)

                p_2 += np.sign(
                    local_differential) * step_p2_bar * 1e5 if local_differential != 0 else -step_p2_bar * 1e5
            else:
                p_2 -= step_p2_bar * 1e5

            last_converged_cop = final_cop_for_this_p2
            last_p2_bar = p2_bar_current

        logger.warning("Breaking: Max outer iterations reached.")
        if self.show_iteration and self.save_path_plots:
            fig_iterations.savefig(self.save_path_plots.joinpath(f"{inputs.get_name()}_convergence_plot_MAX_ITER.png"))
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