from tsp.tsb import *

points = 10
R = 2
num = 3
off = R*num/2 + .5
depot = np.array((0, 0, 1))

pounts = 10
infos = [{'pos': np.random.normal((1, 1, 0), (5, 5, 5)).clip([-10, -10, 1]),
          'tau': 1E0}
         for i in range(pounts)]
array = np.array
infos = [{'pos': array([-4.13007858, 2.21171989, 1.]), 'tau': 1.0},
         {'pos': array([-1.02735794, 3.71258206, 1.]), 'tau': 1.0},
         {'pos': array([-6.10099879, -2.66872272, 2.29806299]), 'tau': 1.0},
         {'pos': array([-1.44366215, 6.23788356, 1.]), 'tau': 1.0},
         {'pos': array([1.70957165, -2.35378516, 1.]), 'tau': 1.0},
         {'pos': array([-0.03658502, -2.71775426, 1.]), 'tau': 1.0},
         {'pos': array([-6.15198514, 10.884783, 1.]), 'tau': 1.0},
         {'pos': array([2.49765138, 7.08216976, 2.52365775]), 'tau': 1.0},
         {'pos': array([0.4479817, -4.1943272, 1.]), 'tau': 1.0},
         {'pos': array([-5.59681296, 0.53872593, 7.12257391]), 'tau': 1.0}]
pounts_in=5
infos = []
for i in range(pounts_in):
    pos=np.random.normal((0, 0, 2), (.5, .5, 0)).clip([-10, -10, 1])
    infos.append({'pos': pos,
          'tau': 1E0})
R=4
pounts_out=10
for i in range(pounts_out):
    infos.append({'pos': np.array((R*np.cos(2*np.pi*i/pounts_out),R*np.sin(2*np.pi*i/pounts_out),2)),
        'tau': 1E0})
pounts=pounts_out+pounts_in
bb = travelingSalesBlimp(num_agents=2,
                      start_zone=lambda i: depot + np.random.normal((0, 0, 0), (1, 1, 0)),
                      num_points=pounts,
                      spawn_pt_info=lambda i: infos[i],
                      alpha=1E-2,
                      grads_per_epoch=1000,
                      converge_tol=None,
                      depot=depot,
                      depot_tol=.5,
                      #debug=True,
                      wakeup=[COPPELIA_WAKEUP])

print(bb.experiments(1))
bb.kill()
quit()
bb = localSearchBlimp(num_agents=2,
                      start_zone=lambda i: depot + np.random.normal((0, 0, 0), (1, 1, 0)),
                      num_points=pounts,
                      spawn_pt_info=lambda i: infos[i],
                      alpha=1E-2,
                      grads_per_epoch=1000,
                      converge_tol=None,
                      depot=depot,
                      depot_tol=.5,
                      debug=True,
                      wakeup=[COPPELIA_WAKEUP])
bb.init_exp(True)
handles = bb.pointData.keys()
H = [h for h in handles if not bb.pointData[h]['is depot']]
if False:
    pp = [[], []]
    for h in H:
        pp[int(np.random.random()*1.1)].append(h)
    costs = []
    for part_index in range(int(2**len(H))):
        part_str = bin(2**len(H) + part_index)[3:]
        pp = [[], []]
        for i, h in enumerate(H):
            pp[int(part_str[i])].append(h)
        goal_list = bb.make_goal_list(pp)
        bb.create_times(goal_list)
        cost = bb.get_tot_disc_entropy(goal_list).item()
        costs.append(cost)
    print(costs)
    print(costs.index(max(costs)))
    print(costs[costs.index(max(costs))])
    quit()

H = [h for h in handles if not bb.pointData[h]['is depot']]
costs = []
part_index = 179
part_str = bin(2**len(H) + part_index)[3:]
pp = [[], []]
for i, h in enumerate(H):
    pp[int(part_str[i])].append(h)

for part in (pp, bb.make_goal_partition()):
    print('partition')
    print(part)
    cost = bb.get_disc_entropy_from_part(part)
    print()
    goal_list = bb.make_goal_list(part)
    print('goal list')
    print(goal_list)

    print('cost')
    print(cost)
    costs.append(cost)

    for key in bb.dwell_solution:
        print(key)
        for ele in bb.dwell_solution[key]:
            print(ele)
        print()
print(costs)
bb.experiments(1)
bb.kill()
quit()
bb = doubleBlimp(lambda i: depot + np.random.normal((0, 0, 0), (1, 1, 0)),
                 num_points=10,
                 spawn_pt_info=lambda i: {
                     'pos': .25*np.random.normal((0, 0, 0), (1, 10, 1)) +
                            np.array((R*np.cos(i*2*np.pi/points) + (3*R*(i//points)) - off,
                                      0,
                                      1.5 + R + R*np.sin(i*2*np.pi/points))),
                     'tau': 1E0},
                 alpha=1E-3,
                 grads_per_epoch=2000,
                 converge_tol=None,
                 depot=np.array((-off, 0, R + 1)),
                 depot_tol=.5
                 )

print(bb.experiments(1))
