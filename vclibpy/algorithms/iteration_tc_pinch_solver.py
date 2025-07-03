import logging
from typing import Union
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import fsolve

try:
    import CoolProp.CoolProp as CP
except ImportError:
    CP = None

from vclibpy import Inputs, FlowsheetState
from vclibpy.algorithms.base import Algorithm
from vclibpy.flowsheets import BaseCycle

logger = logging.getLogger(__name__)


class Iteration_TC_Pinch(Algorithm):
    """
    Algorithm to calculate steady states with an iteration based approach.
    This version uses a master 'fsolve' to find a target pinch point in the gas cooler.
    """

    def __init__(self, **kwargs):
        """Initialize class with kwargs"""
        self.min_iteration_step = kwargs.pop("min_iteration_step", 10)
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
        Handles the iteration for the subcritical case. (Unchanged)
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
        Handles the iteration for the transcritical case by directly solving for the target pinch point.
        """
        # === "Gedächtnis" für p_1 erstellen ===
        # Wir verwenden eine Liste, damit sie in der verschachtelten Funktion geändert werden kann.
        p_1_last_successful = [p_1_start]
        history_data = []

        def get_pinch_error_for_p2(p_2_guess_array):
            p_2_guess = p_2_guess_array[0]  # fsolve arbeitet mit Arrays

            # Harte untere Druckgrenze, um unsinnige Bereiche zu vermeiden
            try:
                _, p_crit, _ = flowsheet.compressor.med_prop.get_critical_point()
                p_2_min_limit = p_crit + 1.5e5  # 1.5 bar Sicherheitsabstand
            except (ImportError, AttributeError):
                p_2_min_limit = 75.5e5  # Fallback-Wert

            if p_2_guess < p_2_min_limit:
                return 100.0  # Hoher Fehlerwert

            # === Innere Schleife zur Regelung von p_1 auf error_eva = 0 ===
            # Starte mit dem letzten erfolgreichen p_1-Wert aus dem "Gedächtnis"
            p_1 = p_1_last_successful[0]

            step_p1 = 1e4
            min_step_p1 = 10
            error_eva_history_inner = [np.nan]

            for i_inner in range(100):
                try:
                    error_eva, _, _, dT_min_con = flowsheet.calculate_cycle_for_pressures(p_1=p_1, p_2=p_2_guess,
                                                                                          inputs=inputs,
                                                                                          fs_state=fs_state)
                except Exception as e:
                    logging.debug(f"Inner calculation failed for p_2={p_2_guess * 1e-5:.2f} bar. Error: {e}")
                    return 100.0

                # Konvergenz-Prüfung für die innere p_1 Schleife
                is_converged = abs(error_eva) < 1e-3
                if is_converged:
                    # ERFOLG! Speichere den neuen p_1-Wert im "Gedächtnis"
                    p_1_last_successful[0] = p_1
                    target_pinch = 3.0
                    pinch_error = dT_min_con - target_pinch

                    history_data.append({
                        'p_2_guess_bar': p_2_guess / 1e5,
                        'p_1_found_bar': p_1 / 1e5,
                        'dT_min_con_K': dT_min_con,
                        'pinch_error_K': pinch_error,
                        'error_eva': error_eva,
                    })

                    logging.info(
                        f"Tested p_2={p_2_guess * 1e-5:.2f} bar -> p_1={p_1 * 1e-5:.2f} bar -> dT_min_con={dT_min_con:.2f} K -> Pinch-Error={pinch_error:.2f} K")
                    fs_state.set("p_eva", p_1, "Pa")
                    return pinch_error

                # Anpassung von p_1 für den nächsten Schritt
                if i_inner > 0 and np.sign(error_eva) != np.sign(error_eva_history_inner[-1]):
                    step_p1 = max(step_p1 / 2.0, min_step_p1)
                error_eva_history_inner.append(error_eva)
                p_1 += np.sign(error_eva) * step_p1

            logging.warning(f"Inner loop for p_1 did not converge for p_2={p_2_guess * 1e-5:.2f} bar.")
            return 100.0  # Hoher Fehlerwert

        # === Master-Solver ===
        p2_initial_guess = 100e5  # Start bei 100 bar

        try:
            # Finde den p_2, bei dem der Pinch-Fehler null ist (also dT_min_con = 3.0)
            p_2_solution_array, _, success, msg = fsolve(get_pinch_error_for_p2, x0=[p2_initial_guess], xtol=1e4,
                                                         full_output=True)

            if not success:
                logger.error(f"Master solver 'fsolve' FAILED to converge. Message: {msg}")
                return None

            p_2_solution = p_2_solution_array[0]
            # Der finale p_1 Wert ist der letzte, den wir uns gemerkt haben.
            final_p_1 = p_1_last_successful[0]

            logger.info(
                f"Final solution found at p_con={p_2_solution * 1e-5:.2f} bar and p_eva={final_p_1 * 1e-5:.2f} bar.")

            # Finale Berechnung mit der gefundenen Lösung, um den Zustand sicherzustellen
            flowsheet.calculate_cycle_for_pressures(p_1=final_p_1, p_2=p_2_solution, inputs=inputs, fs_state=fs_state)
            flowsheet.iteration_converged = True

            return flowsheet.calculate_outputs_for_valid_pressures(
                p_1=final_p_1,
                p_2=p_2_solution,
                inputs=inputs,
                fs_state=fs_state,
                save_path_plots=self.save_path_plots
            )

        except Exception as e:
            logger.error(f"The master solver for pinch point failed with an unexpected error: {e}")
            import traceback
            traceback.print_exc()
            return None

        finally:
            # === NEU: Iterationsverlauf IMMER speichern ===
            if self.save_path_plots and history_data:
                logger.info("Saving iteration history to CSV...")
                df_history = pd.DataFrame(history_data)
                filepath = self.save_path_plots.joinpath(f"{inputs.get_name()}_iteration_history.csv")
                df_history.to_csv(filepath, sep=';', decimal=',', index=False)
                logger.info(f"Iteration history saved to {filepath}")
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