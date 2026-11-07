"""SUMO NEMA actuated ring-and-barrier baseline.

The signals are run entirely by SUMO's NEMA controller, which MultiSignal
generates for every signal from the network (resco_benchmark/utils/nema.py)
and loads with --additional-files.  MultiSignal never switches phases for this
algorithm, so the actions returned here are ignored.
"""

from resco_benchmark.agents.agent import IndependentAgent, Agent


class ACTUATED(IndependentAgent):
    def __init__(self, obs_act):
        super().__init__(obs_act)
        for key in obs_act:
            self.agents[key] = NoOpAgent()


class NoOpAgent(Agent):
    def act(self, observation):
        return 0

    def observe(self, observation, reward, done, info):
        pass
