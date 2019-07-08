import vrep

maxVelocity=2
def calculateOmega():
    k_angular_p = 2.0 * maxVelocity
    k_angular_D = 0.25
    k_angular_S = 1.0
    k_angular_I = 2.5

    return 10

def vomega2bytecodes(v, omega, g, L=0.216):
    """
    Turns v and omega into control codes for differential drive robot
    v: forward velocity
    omega: angular velocity
    g: gain constant
    L: length of axle, default .216
        >>> right = -1.9914
        >>> left = -2.2074
        >>> right - left
        0.21599999999999975
    """
    v_comm = v
    v_diff = omega * L / 2.0
    ctrl_sig_left = (v_comm - v_diff) / float(g)
    ctrl_sig_right = (v_comm + v_diff) / float(g)
    return ctrl_sig_left, ctrl_sig_right
    
print 'Program started'
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',8000,True,True,5000,5)
if clientID!=-1:
    print 'Connected to remote API server'
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_return_ok:
        print 'Number of objects in the scene: ',len(objs)
    else:
        print 'Remote API function call returned with error code: ',res            

    _ = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    ctrl_sig_left, ctrl_sig_right=vomega2bytecodes(2,calculateOmega(),1)
    _,leftMotor=vrep.simxGetObjectHandle(
            clientID, 'ePuck_leftJoint', vrep.simx_opmode_oneshot_wait)
    _ = vrep.simxSetJointTargetVelocity(
                clientID,leftMotor,ctrl_sig_left,vrep.simx_opmode_oneshot_wait) # set left wheel velocity
    
else:
    print 'Failed connecting to remote API server'
print 'Program ended'




