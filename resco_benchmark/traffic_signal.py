from __future__ import annotations
import logging

from resco_benchmark.config.config import config as cfg

if cfg.libsumo and not cfg.gui:
    import libsumo as traci
else:
    cfg.libsumo = False
    import traci
import traci.constants as tc

logger = logging.getLogger(__name__)


def create_yellows(phases: dict[int, str]) -> dict[str, str]:
    yellow_transitions: dict[str, str] = (
        dict()
    )  # current phase + next phase keyed to corresponding yellow phase

    for current_phase in range(len(phases)):
        for next_phase in range(len(phases)):
            intermediate_phase = list()
            current_colors = phases[current_phase]
            next_colors = phases[next_phase]

            for light_idx, color in enumerate(current_colors):
                next_color = next_colors[light_idx]

                green_priority = color == "G"
                green_low_priority = color == "g"
                red_next = next_color == "r"
                right_red_next = next_color == "s"

                if green_priority or green_low_priority:
                    if red_next or right_red_next:
                        intermediate_phase.append("y")
                        continue
                intermediate_phase.append(color)

            if "y" in intermediate_phase:
                transition_key = str(current_phase) + "_" + str(next_phase)
                yellow_transitions[transition_key] = "".join(intermediate_phase)
        all_red = "".join(["r" for _ in range(len(phases[current_phase]))])
        yellow_transitions["all_red"] = all_red
    return yellow_transitions


class Signal:
    def __init__(self, sumo, signal_id: int) -> None:
        self.sumo = sumo
        self.signal_id: int = signal_id
        self.signals: dict[str, Signal] | None = (
            None  # Used to allow observation communication, init in MultiSignal
        )

        self.lanes: list[str] = list()
        self.outbound_lanes: list[str] = list()
        self.inbounds_fr_direction: dict[str, list[str]] = dict()
        self.out_lane_to_signal_id: dict[str, str] = dict()
        self.lane_sets: dict[str, list[str]] = cfg[self.signal_id]["lane_sets"]

        self.lane_sets_outbound: dict[str, list[str]] = dict()
        for key in self.lane_sets:
            self.lane_sets_outbound[key] = list()

        self._find_neighbors()

        self.lane_lengths: dict[str, int] = (
            dict()
        )  # Used to determine vehicle distance on lanes & state val

        self.lane_speed_limits: dict[str, int] = dict()  # State val

        for lane in self.lanes:
            self.lane_lengths[lane] = self.sumo.lane.getLength(lane)
            self.lane_speed_limits[lane] = self.sumo.lane.getMaxSpeed(lane)

        self.observation: Observation = Observation(
            self.lane_lengths, self.lane_speed_limits
        )

        # Register subscriber
        # Junc location diff from lane start location, > max_distance will be filtered in observe()
        junc_dist: int = (
            cfg.max_distance + 25
        )  # Some leeway since junctions are defined at the center (filtered later)
        traci.junction.subscribeContext(
            self.signal_id,
            tc.CMD_GET_VEHICLE_VARIABLE,
            junc_dist,
            [
                tc.VAR_LANE_ID,
                tc.VAR_LANEPOSITION,
                tc.VAR_ACCELERATION,
                tc.VAR_SPEED,
                tc.VAR_FUELCONSUMPTION,
                tc.VAR_WAITING_TIME,
                tc.VAR_ALLOWED_SPEED,
                tc.VAR_TYPE,
                tc.VAR_TIMELOSS,
            ],
        )

        self.green_phases: dict[int, str] = dict()
        idx = 0
        logics = self.sumo.trafficlight.getAllProgramLogics(signal_id)
        # With ACTUATED a NEMA program is loaded too; phases come from the network's own.
        logic = next((lg for lg in logics if lg.programID != "NEMA-actuated"), logics[0])
        for p in logic.getPhases():
            if "y" not in p.state and p.state.count("r") + p.state.count("s") != len(
                p.state
            ):
                self.green_phases[idx] = p.state
                idx += 1

        self.yellow_transitions = create_yellows(self.green_phases)

        self.current_phase: int = 0
        self.next_phase: int = self.current_phase
        self.time_in_phase: int = 0
        self.time_since_phase: dict[int, int] = dict()
        for i in range(len(self.green_phases)):
            self.time_since_phase[i] = 0

        self.clearance_phase = 0

    def _find_neighbors(self) -> None:
        reversed_directions: dict[str, str] = {"N": "S", "E": "W", "S": "N", "W": "E"}

        for direction in self.lane_sets:
            for lane in self.lane_sets[direction]:
                inbound_to_direction: str = direction.split("-")[0]
                inbound_fr_direction = reversed_directions[inbound_to_direction]
                if inbound_fr_direction in self.inbounds_fr_direction:
                    dir_lanes: list[str] = self.inbounds_fr_direction[
                        inbound_fr_direction
                    ]
                    if lane not in dir_lanes:
                        dir_lanes.append(lane)
                else:
                    self.inbounds_fr_direction[inbound_fr_direction] = [lane]
                if lane not in self.lanes:
                    self.lanes.append(lane)

        # Populate outbound lane information
        down_stream: dict[str, str] = cfg[self.signal_id]["downstream"]
        for direction in down_stream:
            dwn_signal = down_stream[direction]
            if dwn_signal is not None:  # A downstream intersection exists
                dwn_lane_sets: dict[str, list[str]] = cfg[dwn_signal][
                    "lane_sets"
                ]  # Get downstream signal's lanes
                for key in dwn_lane_sets:  # Find all inbound lanes from upstream
                    if key.split("-")[0] == direction:  # Downstream direction matches
                        dwn_lane_set: list[str] = dwn_lane_sets[key]
                        if dwn_lane_set is None:
                            raise Exception("Invalid signal config")
                        for lane in dwn_lane_set:
                            if lane not in self.outbound_lanes:
                                self.outbound_lanes.append(lane)
                            self.out_lane_to_signal_id[lane] = dwn_signal
                            for selfkey in self.lane_sets:
                                if (
                                    selfkey.split("-")[1] == key.split("-")[0]
                                ):  # Out dir. matches dwnstrm in dir.
                                    self.lane_sets_outbound[selfkey] += dwn_lane_set
        for key in self.lane_sets_outbound:  # Remove duplicates
            self.lane_sets_outbound[key] = list(set(self.lane_sets_outbound[key]))

    def switch_phase(self, new_phase: int) -> None:
        if new_phase != self.current_phase:
            self.time_since_phase[self.current_phase] = 0
            self.time_since_phase[new_phase] = 0
            self.time_in_phase = 0
            key = str(self.current_phase) + "_" + str(new_phase)
            if key in self.yellow_transitions:
                self.sumo.trafficlight.setRedYellowGreenState(
                    self.signal_id, self.yellow_transitions[key]
                )
                self.next_phase = new_phase
                if "g" in self.green_phases[self.current_phase]:
                    self.clearance_phase = cfg.clearance_length
            else:
                self.sumo.trafficlight.setRedYellowGreenState(
                    self.signal_id, self.green_phases[new_phase]
                )
                self.current_phase = new_phase
                self.next_phase = None

    def step(self):
        self.time_in_phase += 1
        for phase in range(len(self.green_phases)):
            self.time_since_phase[phase] += 1

        self._inter_step_observe()

        if self.next_phase is not None:
            if self.time_in_phase >= cfg.yellow_length:
                if self.clearance_phase != 0:
                    self.sumo.trafficlight.setRedYellowGreenState(
                        self.signal_id, self.yellow_transitions["all_red"]
                    )
                    self.clearance_phase -= cfg.step_ratio
                else:
                    self.sumo.trafficlight.setRedYellowGreenState(
                        self.signal_id, self.green_phases[self.next_phase]
                    )
                    self.time_in_phase = 0
                    self.current_phase = self.next_phase
                    self.next_phase = None

    def _inter_step_observe(self) -> None:
        subscription = traci.junction.getContextSubscriptionResults(self.signal_id)
        for veh_id in subscription:
            if veh_id.startswith("ghost"):
                continue
            vehicle = subscription[veh_id]
            veh_lane = vehicle[tc.VAR_LANE_ID]
            if veh_lane not in self.lane_lengths:
                continue

            distance_from_light = (
                self.lane_lengths[veh_lane] - vehicle[tc.VAR_LANEPOSITION]
            )
            if distance_from_light > cfg.max_distance:
                continue

            vehicle[tc.VAR_VEHICLE] = veh_id
            vehicle[tc.VAR_POSITION] = distance_from_light
            # Only provide delay visible from the intersection
            vehicle[tc.VAR_TIMELOSS] = (
                vehicle[tc.VAR_SPEED] - vehicle[tc.VAR_ALLOWED_SPEED]
            ) / vehicle[tc.VAR_ALLOWED_SPEED]

            self.observation.add_vehicle(Vehicle(vehicle))

    def observe(self) -> Observation:
        # noinspection PyProtectedMember
        self.observation._step(self.time_since_phase)
        return self.observation


class Vehicle:
    def __init__(self, vehicle: dict) -> None:
        self.veh_id: str = vehicle[tc.VAR_VEHICLE]
        self.lane_id: str = vehicle[tc.VAR_LANE_ID]
        self.type: str = vehicle[tc.VAR_TYPE]
        self.times_observed: int = 1
        self.times_observed_last_step: int = 0

        self.speed: float = vehicle[tc.VAR_SPEED]
        self.acceleration: float = vehicle[tc.VAR_ACCELERATION]
        self.position: float = vehicle[tc.VAR_POSITION]
        self.queued: bool = False

        # These values accumulate over time
        self.wait: float = (
            0.0  # SUMO carries waiting time between intersections, calc manually
        )
        self.delay: float = vehicle[tc.VAR_TIMELOSS]
        self.total_speed: float = vehicle[tc.VAR_SPEED]
        self.total_acceleration: float = vehicle[tc.VAR_ACCELERATION]
        self.fuel_consumption: float = vehicle[tc.VAR_FUELCONSUMPTION]
        self.length = 5  # TODO get from vehicle type, passenger car assumed for now
        self.min_gap = 2.5  # TODO get from vehicle type, passenger car assumed for now, only use for queue distance, otherwise should be unknown

    @property
    def average_speed(self) -> float:
        return self.total_speed / self.times_observed

    @property
    def average_acceleration(self) -> float:
        return self.total_acceleration / self.times_observed

    def observe(self, vehicle: Vehicle) -> None:
        self.times_observed += 1
        self.speed = vehicle.speed
        self.acceleration = vehicle.acceleration
        self.position = vehicle.position

        if vehicle.speed < 0.1:
            self.queued = True
        if self.queued:
            self.wait += cfg.step_ratio

        self.total_acceleration += vehicle.acceleration
        self.total_speed += vehicle.speed
        self.fuel_consumption += vehicle.fuel_consumption
        self.delay += vehicle.delay


# TODO account for lane changes?
class Lane:
    def __init__(self, lane_id: str, max_speed: float, length: float) -> None:
        self.lane_id: str = lane_id
        self.vehicles: dict[str, Vehicle] = dict()

        self.vehicle_count = 0
        self.queued: int = 0
        self.arrived: int = 0
        self.departed: int = 0
        self.max_wait: float = 0.0

        self.max_speed = max_speed
        self.length = length

    @property
    def approaching(self) -> int:
        return len(self.vehicles) - self.queued

    def add_vehicle(self, vehicle: Vehicle) -> None:
        if vehicle.veh_id in self.vehicles:
            self.vehicles[vehicle.veh_id].observe(vehicle)
        else:
            self.vehicles[vehicle.veh_id] = vehicle


class Observation:
    def __init__(self, lane_lengths, lane_speed_limits) -> None:
        self.lanes: dict[str, Lane] = dict()
        self.lane_lengths: dict[str, int] = lane_lengths
        self.lane_speed_limits: dict[str, int] = lane_speed_limits

        for lane_id in lane_lengths:
            self.lanes[lane_id] = Lane(
                lane_id, lane_speed_limits[lane_id], lane_lengths[lane_id]
            )

        self.time_since_phase: dict[int, int] = dict()
        self.vehicle_count: int = 0
        self.departed: int = 0
        self.arrived: int = 0
        self.total_wait: int = 0
        self.total_queued: int = 0
        self.max_queue: int = 0

    def add_vehicle(self, vehicle: Vehicle) -> None:
        lane_id = vehicle.lane_id
        self.lanes[lane_id].add_vehicle(vehicle)

    # Compute totals / averages, only ever call this once per step_length
    def _step(self, time_since_phase: dict[int, int]) -> None:  # TODO rewrite
        vehicle_count: int = 0
        self.time_since_phase = time_since_phase
        self.departed = 0
        self.total_wait = 0
        self.total_queued = 0
        self.max_queue = 0
        for lane_id in self.lanes:
            lane = self.lanes[lane_id]
            lane.queued = 0
            lane.departed = 0
            lane_vehicle_count: int = 0  # For computing arrived

            pending_removal: list[str] = list()
            for vehicle in lane.vehicles.values():
                if vehicle.times_observed_last_step == vehicle.times_observed:
                    self.departed += 1
                    lane.departed += 1
                    pending_removal.append(vehicle.veh_id)
                else:
                    lane_vehicle_count += 1
                    if vehicle.queued:
                        self.total_queued += 1
                        lane.queued += 1
                    self.total_wait += vehicle.wait
                    if vehicle.wait > lane.max_wait:
                        lane.max_wait = vehicle.wait
                vehicle.times_observed_last_step = vehicle.times_observed

            lane.vehicle_count = lane_vehicle_count
            lane.arrived = lane_vehicle_count - (lane.vehicle_count - lane.departed)
            lane.vehicle_count = lane_vehicle_count

            vehicle_count += lane_vehicle_count
            if lane.queued > self.max_queue:
                self.max_queue = lane.queued

            for veh_id in pending_removal:
                del lane.vehicles[veh_id]

        self.arrived = vehicle_count - (self.vehicle_count - self.departed)
        self.vehicle_count = vehicle_count

    def get_lane(self, lane_id: str) -> Lane | None:
        return self.lanes[lane_id]
