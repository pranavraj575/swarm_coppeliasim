import subprocess
import psutil
COPPELIA_WAKEUP = '~/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh'
import time
p=subprocess.Popen(COPPELIA_WAKEUP,stdout=subprocess.PIPE,shell=True)

death=[
psutil.STATUS_ZOMBIE,
psutil.STATUS_DEAD,
psutil.STATUS_STOPPED]
def kill_individual(proc_pid,sleep=.01):
    process = psutil.Process(proc_pid)
    while process.status() not in death:
        process.kill()
        time.sleep(sleep)
def kill(proc_pid):
    """
    kills a process and processes it spawned

    @param proc_pid: id to kill
    """
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        try:
            kill_individual(proc.pid)
        except:
            pass
    kill_individual(process.pid)
input()
pp=psutil.Process(p.pid)
print(pp.status())
kill(p.pid)
print(pp.status())
input()
print(pp.status())