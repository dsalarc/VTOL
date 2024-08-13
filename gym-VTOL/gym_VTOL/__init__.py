from gym.envs.registration import register

register(
    id='Vahana_VertFlight-v0',
    entry_point='gym_VTOL.envs:Vahana_VertFlight',
)


register(
    id='Vahana_Development-v0',
    entry_point='gym_VTOL.envs:Vahana_Development',
)

