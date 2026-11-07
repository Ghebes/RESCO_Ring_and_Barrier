import logging

import numpy as np

from resco_benchmark.config.config import config as cfg
import resco_benchmark.mdp_options.ring_barrier as rb
from resco_benchmark.agents.static.fixed import FIXED, FixedAgent

logger = logging.getLogger(__name__)


# Config values of phase or null will use direct 'set next phase' actions


class RingAndBarrier:
    """Dual-ring, two-barrier action set for any RESCO map.

    Actions (identical on every signal): 0 hold, 1 advance ring 1, 2 advance
    ring 2, 3 advance both / cross the barrier.  Each signal's state -> phase
    map is derived from the map's ``phase_pairs`` and ``pair_to_act_map``; see
    ``resco_benchmark.mdp_options.ring_barrier`` for the ring layout and the
    rules for omitted phases and missing split combinations.

    FHWA Traffic Signal Timing Manual, section 4.2.2:
    https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter4.htm
    """

    HOLD_BOTH = rb.HOLD_BOTH
    ADVANCE_RING_1 = rb.ADVANCE_RING_1
    ADVANCE_RING_2 = rb.ADVANCE_RING_2
    ADVANCE_BOTH = rb.ADVANCE_BOTH

    # Agents whose actions are phase_pairs indices cannot drive 4 R&B actions.
    INCOMPATIBLE_ALGORITHMS = {"MPLight", "AdvancedMPLight", "CoSLight"}

    def __init__(self, obs_act, signals):
        if cfg.algorithm in self.INCOMPATIBLE_ALGORITHMS:
            raise ValueError(
                f"RingAndBarrier is not supported with {cfg.algorithm}: its actions "
                "are phase_pairs indices. Use an independent agent such as IDQN "
                "or IPPO."
            )
        self.signals = signals
        self.num_acts = rb.NUM_ACTS
        self.max_green = cfg.ring_max_green
        self.through_min_green = cfg.ring_through_min_green
        self.through_max_green = cfg.ring_through_max_green
        for name, value in (
            ("ring_max_green", self.max_green),
            ("ring_through_min_green", self.through_min_green),
            ("ring_through_max_green", self.through_max_green),
        ):
            if value is not None and value <= 0:
                raise ValueError(f"{name} must be positive or null")

        pair_to_act_map = cfg.get("pair_to_act_map")
        self.controllers = dict()
        for signal_id in signals:
            pair_to_act = None
            if pair_to_act_map is not None:
                pair_to_act = pair_to_act_map.get(signal_id)
                if pair_to_act is None:
                    raise ValueError(
                        f"pair_to_act_map has no entry for signal {signal_id!r}"
                    )
            phase_map = rb.build_phase_map(
                cfg.phase_pairs,
                pair_to_act,
                num_green=obs_act[signal_id][1],
                signal_id=signal_id,
            )
            self.controllers[signal_id] = rb.RingBarrierSignal(
                phase_map,
                max_green=self.max_green,
                through_min_green=self.through_min_green,
                through_max_green=self.through_max_green,
                signal_id=signal_id,
            )
            logger.info(
                "RingAndBarrier %s: %s",
                signal_id,
                {"".join(map(str, k)): v for k, v in sorted(phase_map.items())},
            )
        self.reset()

    def reset(self):
        """Start each episode at barrier 0's entry state."""
        for controller in self.controllers.values():
            controller.reset()

    def features(self, signal_id):
        scale = self.through_max_green or self.max_green or 120.0
        return self.controllers[signal_id].features(elapsed_scale=scale)

    def act(self, acts):
        mapped_acts = dict(acts)
        for signal_id in self.signals:
            action = acts[signal_id]
            if not isinstance(action, (int, np.integer)):
                raise TypeError(
                    f"RingAndBarrier action for {signal_id} must be an integer"
                )
            if action < 0 or action >= self.num_acts:
                raise ValueError(
                    f"RingAndBarrier action for {signal_id} must be between 0 "
                    f"and {self.num_acts - 1}; received {action}"
                )
            mapped_acts[signal_id] = self.controllers[signal_id].step(
                int(action), cfg.step_length
            )
        return mapped_acts


#   Replaces action space with simplified set of [keep, increase, decrease] following fixed timing config
class FixedCyclePlan:
    def __init__(self, obs_act, signals):
        self.signals = signals
        self.fixed_agent = FIXED(obs_act)
        self.num_acts = 3

    def act(self, acts):
        for signal in self.signals:
            agt_act = acts[signal]
            if agt_act == 0:
                pass
            elif agt_act == 1:
                self.fixed_agent.agents[signal].increase_current_phase_length()
            elif agt_act == 2:
                self.fixed_agent.agents[signal].decrease_current_phase_length()
            else:
                raise NotImplementedError()
        acts = self.fixed_agent.act(observation=self.signals)
        return acts


# Follow config fixed time defined cycle, choose to stay in current phase or go next
class FixedCycle:
    def __init__(self, obs_act, signals):
        self.signals = signals
        self.fixed_agent = FIXED(obs_act)
        self.num_acts = 2

        # Force all phases to 1 step length
        for signal in self.signals:
            for i in range(len(self.fixed_agent.agents[signal].plan)):
                self.fixed_agent.agents[signal].plan[i] = 1
            self.fixed_agent.agents[signal].active_phase = (
                len(self.fixed_agent.agents[signal].plan) - 2
            )

    def act(self, acts):
        if cfg.algorithm == "FIXED":
            return acts
        for signal in self.signals:
            agt_act = acts[signal]
            print(signal, agt_act)
            if agt_act == 0:  # Keep same phase
                self.fixed_agent.agents[signal].active_phase_len = 0
            elif agt_act == 1:  # Go next
                self.fixed_agent.agents[signal].active_phase_len = np.inf
            else:
                raise NotImplementedError()
        acts = self.fixed_agent.act(observation=self.signals)
        print("mask", acts)
        return acts


class PlanPick:
    def __init__(self, obs_act, signals):
        self.signals = signals
        # Only valid/tested for saltlake
        cfg["equal_plan"] = dict()
        cfg["vertical_plan"] = dict()
        cfg["horizontal_plan"] = dict()
        cfg["horizontal_plan"]["fixed_phase_order_idx"] = 0
        cfg["vertical_plan"]["fixed_phase_order_idx"] = 0
        cfg["equal_plan"]["fixed_phase_order_idx"] = 0
        cfg["equal_plan"]["fixed_timings"] = [4, 0, 4, 4, 4, 4, 4, 0]
        cfg["vertical_plan"]["fixed_timings"] = [
            4,
            0,
            4,
            4,
            4 + cfg.priority_offset,
            4 + cfg.priority_offset,
            4 + cfg.priority_offset,
            0,
        ]
        cfg["horizontal_plan"]["fixed_timings"] = [
            4 + cfg.priority_offset,
            0,
            4 + cfg.priority_offset,
            4 + cfg.priority_offset,
            4,
            4,
            4,
            0,
        ]

        self.equal_plans = dict()
        obs_act = obs_act.copy()
        obs_act[1] = 8
        for signal in self.signals:
            self.equal_plans[signal] = FixedAgent(obs_act, "equal_plan")
        self.vertical_plans = dict()
        for signal in self.signals:
            self.vertical_plans[signal] = FixedAgent(obs_act, "vertical_plan")
        self.horizontal_plans = dict()
        for signal in self.signals:
            self.horizontal_plans[signal] = FixedAgent(obs_act, "horizontal_plan")
        self.num_acts = 3

    def act(self, acts):
        new_acts = dict()
        for signal in self.signals:
            agt_act = acts[signal]
            if agt_act == 0:
                new_acts[signal] = self.equal_plans[signal].act(None)
            elif agt_act == 1:
                new_acts[signal] = self.vertical_plans[signal].act(None)
            elif agt_act == 2:
                new_acts[signal] = self.horizontal_plans[signal].act(None)
            else:
                print(signal, agt_act)
                raise NotImplementedError()
        return new_acts


# Continuous or discrete, choose current phase's length
class PhaseLength:
    pass  # TODO
