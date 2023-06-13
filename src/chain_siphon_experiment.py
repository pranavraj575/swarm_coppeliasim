from chain_siphon import *
import csv
from matplotlib import pyplot as plt
import time, sys
from zmqRemoteApi import RemoteAPIClient

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))


client = RemoteAPIClient()
sim = client.getObject('sim')
startZone = {'xmin': .5, 'xmax': 6,
             'ymin': -2, 'ymax': 2,
             'zmin': 1, 'zmax': 1.25,
             }

MODELDIR = DIR + '/ros_ctrl_models/blimp_narrow.ttm'
SCENEDIR = DIR + '/scenes/WallClimb.ttt'
narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
modelToLoad = narrowModelPath
sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
pause_time = 1
if len(sys.argv)>1:
    name=sys.argv[1]
else:
    name = 'CONTROL'

save_dir = os.path.join(DIR, 'output', name)
#save_dir=None

if save_dir is None:
    print("WARNING SAVE DIR IS NOT SPECIFIED, SO THIS IS NOT BEING SAVED")

def wall_obstacle_vector(pos, obs_range=1., obs_rep=1):
    #for a 3m high yz wall, assume pos has positive z
    if pos[2]<=3:
        if abs(pos[0])<obs_range:
            dist=max(.00001,abs(pos[0]))
            vect=np.array((pos[0]/dist,0,0))
            return obs_rep*vect/(dist**2)
            #since force vector/displacement is just going to be in the x direction in this case
        #too far away in this case
        return np.zeros(3)

    # now pos is higher than wall
    pt=np.array((0,pos[1],3.)) # point on wall closest to the thing, has same y value as pos
    vect=pos-pt
    dist=np.linalg.norm(vect)
    if dist<obs_range:
        dist=max(dist,.00001)
        vect=vect/dist # normalized now
        return obs_rep*vect/(dist**2)

    return np.zeros(3)
if name=="CONTROL":
    params = [{
        'num_agents': num_agents,
        'trials': 30,
        'weight0': .5,
        'weight1': 0,
        'weight2': 0,
        'weight3': 0,
        'weight4': .5,
        'height':1,
        'height tol':.25,
        'end time': 300,
        'etta':.2,
        'max speed':1.,
    } for num_agents in range(1,25)]
    params = [{
        'num_agents': num_agents,
        'trials': 30,
        'weight0': .75,
        'weight1': 0,
        'weight2': 0,
        'weight3': 0,
        'weight4': .75,
        'height':1,
        'height tol':.25,
        'end time': 300,
        'etta':.2,
        'max speed':1.,
    } for num_agents in range(1,25)]
elif name=="CHAINS":
    params = [{
        'num_agents': num_agents,
        'trials': 30,
        'weight0': .5,
        'weight1': 0,
        'weight2': .002,
        'weight3': 1,
        'weight4': .5,
        'height':1,
        'height tol':.25,
        'end time': 300,
        'etta':.2,
        'max speed':1.,
    }  for num_agents in range(1,25)]
    params = [{
        'num_agents': num_agents,
        'trials': 30,
        'weight0': .75,
        'weight1': 0,
        'weight2': .002,
        'weight3': 1,
        'weight4': .75,
        'height':1,
        'height tol':.25,
        'end time': 300,
        'etta':.2,
        'max speed':1.,
    }  for num_agents in range(1,25)]
else:
    raise Exception("invalid name: "+name)
if save_dir is not None and not os.path.exists(save_dir):
    os.makedirs(save_dir)

fake_params = [{
    'num_agents': 15,
    'trials': 1,
    'weight0': .75,
    'weight1': .0,
    'weight2': .002,
    'weight3': 1,
    'weight4': .75,
    'height':1,
    'height tol':.25,
    'end time': 300,
    'etta':.4,
        'max speed':1.,
} ]
ffake_params = [{
    'num_agents': 15,
    'trials': 1,
    'weight0': .75,
    'weight1': .0,
    'weight2': .00,
    'weight3': 0,
    'weight4': .75,
    'height':1,
    'height tol':.25,
    'end time': 300,
    'etta':.4,
        'max speed':1.,
} ]
params_to_save = [param for param in params[0]]
params_to_save.sort()
records_to_save = ['succ',
                   'variance',
                   'time per trial',
                   ]
fields = params_to_save + records_to_save

rclpy.init()
for param in params:
    print('starting:',param)
    key=[param[p] for p in params_to_save]
    if save_dir is not None:
        filename = os.path.join(save_dir, 'data.csv')
        oldfields = None
        olddata = []

        if os.path.exists(filename):
            csvfile=open(filename)
            spamreader = csv.reader(csvfile)
            for row in spamreader:
                if oldfields is None:
                    oldfields = row
                else:
                    olddata.append(row)
            csvfile.close()
            if oldfields == fields:
                skip=False
                for row in olddata:
                    if row[:len(key)]==[str(k) for k in key]:
                        skip=True
                        break
                if skip:
                    print('PREVIOUS EXPIRIMENT FOUND WITH RESULTS:')
                    print({field:row[fields.index(field)]for field in records_to_save})
                    continue
            else:
                raise Exception("FIELDS IN SAVED FILE DO NOT MATCH, MAKE A NEW FILE")
    start_time=time.time()
    record = {'succ': []}
    for trial in range(param['trials']):
        sim.stopSimulation()
        time.sleep(pause_time)

        sim.loadScene(sceneNamePath)
        time.sleep(pause_time)

        agentHandles = []
        chains = []
        height=param['height']
        height_tol=param['height tol']
        height_imp=1

        for i in range(param['num_agents']):
            agentHandle = sim.loadModel(modelToLoad)
            agentHandles.append(agentHandle)  # Save loaded models' handles for analysis later
            for _ in range(100):  # give up at some point
                xPos = round(random.uniform(startZone["xmin"], startZone["xmax"]), 2)
                yPos = round(random.uniform(startZone["ymin"], startZone["ymax"]), 2)
                zPos = round(random.uniform(startZone["zmin"], startZone["zmax"]), 2)
                sim.setObjectPosition(agentHandle, -1, [xPos, yPos, zPos])
                collisionResult, collidingObjectHandles = checkForCollisionWrapper(agentHandle, sim)
                if not collisionResult:
                    break
            pp = Chains(i,
                        param['num_agents'],
                        goal_value=lambda pos: pos[0],#+ height_imp*((pos[2]-height-height_tol)**2/2 if abs(pos[2]-height)>height_tol else 0),  # potential value
                        goal_field=lambda pos:np.array((-1,0,0)),
                        #goal_field=lambda pos: np.array((-1, 0, height_imp*((height-pos[2])-height_tol*np.sign(height-pos[2])) if abs(height-pos[2])>height_tol else 0)),  # gradient to follow
                        #debug=i==0,
                        debug=False,
                        )
            chains.append(pp)
        sim.startSimulation()
        while rclpy.ok():
            for i in range(len(chains)):
                chains[i].siphon(weight=tuple(param['weight' + str(eye)] for eye in range(5)),
                                 use_ultra=True,
                                 point_obstacles=[],
                                 obs_vector=wall_obstacle_vector,
                                 workspace=((-5, startZone['xmax']),
                                            (startZone['ymin'],startZone['ymax']),
                                            (height, height_tol+height)),
                                 etta=param['etta'],
                                 max_speed=param['max speed']
                                 )
                for thingy in SWARM_DATA:
                    pass
                    #print(thingy, [round(v, 2) for v in SWARM_DATA[thingy]])
            if sim.getSimulationTime() > param['end time']:
                break

        sim.pauseSimulation()
        time.sleep(pause_time)
        succ = 0
        for i in range(len(chains)):
            if chains[i].state['x'] < 0:
                succ += 1
            chains[i].break_chain()
        record['succ'].append(succ)
        print('trial',trial+1,
              ':', succ,'succeeded;',
              'running average of',round(sum(record['succ'])/len(record['succ']),3),'succeeded;',
              'avg time per trial:',round((time.time()-start_time)/len(record['succ'])),
              ' '*10,
              end='\r')
    print()
    record['variance']=np.var(record['succ'])
    record['time per trial']=round((time.time()-start_time)/param['trials'])
    record['succ']=np.mean(record['succ'])
    newrow=[param[p] for p in params_to_save] + [record[r] for r in records_to_save]
    print('results:',record)

    if save_dir is not None:
        filename = os.path.join(save_dir, 'data.csv')
        oldfields = None
        olddata = []

        if os.path.exists(filename):
            csvfile=open(filename)
            spamreader = csv.reader(csvfile)
            for row in spamreader:
                if oldfields is None:
                    oldfields = row
                else:
                    olddata.append(row)
            csvfile.close()
        olddata.append(newrow)
        #olddata.sort(key=lambda row:int(row[0]))
        # increasing by num agents

        csvfile= open(filename, 'w')
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(fields)
        csvwriter.writerows(olddata)
        csvfile.close()
