from swarm_ctrl.blimp_swarm import viconBlimps
from swarm_ctrl.vicon import Vicon
from CONFIG import *
import time
blimp_ids=[2]
x=viconBlimps(cfg_paths=[create_config_file(blimp_id) for blimp_id in blimp_ids],vicon=Vicon())
for i in range(5):
    x.move_agent(0,(0.,0.0,0,0.5))
    time.sleep(1)

x.shutdown()