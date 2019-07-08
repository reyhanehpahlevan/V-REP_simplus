
velLeft=0
velRight=0

paramIsNoise=false
paramNoiseProb=0.16
paramNoiseMean=0
paramNoiseSD=1

actualizeLEDs=function()
    if (relLedPositions==nil) then
        relLedPositions={{-0.0343,0,0.0394},{-0.0297,0.0171,0.0394},{0,0.0343,0.0394},
                    {0.0297,0.0171,0.0394},{0.0343,0,0.0394},{0.0243,-0.0243,0.0394},
                    {0.006,-0.0338,0.0394},{-0.006,-0.0338,0.0394},{-0.0243, -0.0243,0.0394}}
    end
    if (drawingObject) then
        simRemoveDrawingObject(drawingObject)
    end
    type=sim_drawing_painttag+sim_drawing_followparentvisibility+sim_drawing_spherepoints+
        sim_drawing_50percenttransparency+sim_drawing_itemcolors+sim_drawing_itemsizes+
        sim_drawing_backfaceculling+sim_drawing_emissioncolor
    drawingObject=simAddDrawingObject(type,0,0,bodyElements,27)
    m=simGetObjectMatrix(ePuckBase,-1)
    itemData={0,0,0,0,0,0,0}
    simSetLightParameters(ledLight,0)
    for i=1,9,1 do
        if (ledColors[i][1]+ledColors[i][2]+ledColors[i][3]~=0) then
            p=simMultiplyVector(m,relLedPositions[i])
            itemData[1]=p[1]
            itemData[2]=p[2]
            itemData[3]=p[3]
            itemData[4]=ledColors[i][1]
            itemData[5]=ledColors[i][2]
            itemData[6]=ledColors[i][3]
            simSetLightParameters(ledLight,1,{ledColors[i][1],ledColors[i][2],ledColors[i][3]})
            for j=1,3,1 do
                itemData[7]=j*0.003
                simAddDrawingObjectItem(drawingObject,itemData)
            end
        end
    end
end

getLightSensors=function()
    data=simReceiveData(0,'EPUCK_lightSens')
    if (data) then
        lightSens=simUnpackFloatTable(data)
    end
    return lightSens
end

rosGetCameraData=function()
    local data,w,h=simGetVisionSensorCharImage(handleCamera)
    d={}
    d['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id="a"}
    d['height']=h
    d['width']=w
    d['encoding']='rgb8'
    d['is_bigendian']=1
    d['step']=w*3
    d['data']=data
    return d
end

randomNormal=function()  -- normal distribution, centered on mean 0, std dev 1
  --return sqrt(-2*log(drand())) * cos(2*M_PI*drand());
  return math.sqrt(-2 * math.log(math.random())) * math.cos(2 * math.pi * math.random())
end

clampValue=function(value, min, max)
    if (value <= min) then
        return min
    elseif (value >= max) then
        return max
    end
    
    return value
end

rosGetProximityData=function(proxData)
    d={{}, {}, {}, {}, {}, {}, {}, {}}

    for i=1,8,1 do
        d[i]['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id = ePuckName .. "/base_prox" .. (i-1)}
        d[i]['radiation_type']=1 --infrared
        d[i]['field_of_view']=0.52359878 -- 30 deg
        d[i]['min_range']=0.0015
        d[i]['max_range']=noDetectionDistance
        if (paramIsNoise and paramNoiseProb > 0) then
            -- test probability of noise
            if (math.random() <= paramNoiseProb) then
                noise = paramNoiseMean + paramNoiseSD * randomNormal() -- noise generation

                if (proxData[i] == d[i]['min_range']) then
                    d[i]['range'] = proxData[i] + noise -- WITH noise
                elseif (proxData[i] == d[i]['max_range']) then
                    d[i]['range'] = proxData[i] - noise -- WITH noise
                else 
                    if (math.random() < 0.5) then
                        d[i]['range'] = proxData[i] - noise -- WITH noise
                    else
                        d[i]['range'] = proxData[i] + noise -- WITH noise
                    end
                end
                -- make sure that the value does not exceed minimum/maximum values
                d[i]['range'] = clampValue(d[i]['range'], d[i]['min_range'], d[i]['max_range'])

            else
                d[i]['range'] = proxData[i] -- no noise
            end
        else
            d[i]['range'] = proxData[i] -- no noise
        end
    end
    return d
end

rosGetImuData=function()
    msg = {}

    accelData=simTubeRead(accelCommunicationTube)
    if (accelData) then
        acceleration=simUnpackFloatTable(accelData)
        msg['header'] = {seq=0,stamp=simExtRosInterface_getTime(), frame_id = ePuckName .. "/base_link"}
        msg['linear_acceleration'] = {}
        msg['linear_acceleration']['x'] = acceleration[1]
        msg['linear_acceleration']['y'] = acceleration[2]
        msg['linear_acceleration']['z'] = acceleration[3]
        msg['linear_acceleration_covariance'] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01}
end

    gyroData=simTubeRead(gyroCommunicationTube)
    if (gyroData) then
        angularVariations=simUnpackFloatTable(gyroData)
        msg['header'] = {seq=0,stamp=simExtRosInterface_getTime(), frame_id = ePuckName .. "/base_link"}
        msg['angular_velocity'] = {}
        msg['angular_velocity']['x'] = angularVariations[1]
        msg['angular_velocity']['y'] = angularVariations[2]
        msg['angular_velocity']['z'] = angularVariations[3]
        msg['angular_velocity_covariance'] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01}
    end

    return msg
end

rosCallbackCmdLed=function(msg)
    for i=1,9,1 do
        if (msg.data[i] == 1) then
            ledColors[i] = {1, 0, 0} --red
        else
            ledColors[i] = {0, 0, 0} --off
        end
    end
end

rosCallbackCmdVel=function(msg)
    -- constants, taken from e-puck ROS driver (epuck_driver_cpp)
    WHEEL_DIAMETER=40 * s        -- mm.
    WHEEL_SEPARATION=53 * s    -- Separation between wheels (mm).

    linear=msg.linear.x * 8.9
    angular=msg.angular.z

    -- determine velocity in e-puck firmware scale (-1000; 1000)
    velLeft = (linear - (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;
    velRight = (linear + (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;

    simAddStatusbarMessage('epuck '.. ePuckName ..' newVel '.. velLeft .. ' ' .. velRight)
end

-- function for tf transform message generation
-- adapted from http://www.forum.coppeliarobotics.com/viewtopic.php?f=5&t=6198#p24920
--getTransformStamped=function(objHandle,name,relTo,relToName)
rosGetTransformStamped=function(childHandle, referenceHandle, childName, referenceName)
    t=simExtRosInterface_getTime()
    p=simGetObjectPosition(childHandle,referenceHandle)
    o=simGetObjectQuaternion(childHandle,referenceHandle)
    return {
        header={
            stamp=t,
            frame_id=referenceName
        },
        child_frame_id=childName,
        transform={
            -- ROS has definition x=front y=side z=up
            translation={x=p[1],y=p[2],z=p[3]},--V-rep
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}--v-rep
        }
    }
end

threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
        st=simGetSimulationTime()

        opMode=simGetScriptSimulationParameter(sim_handle_self,'opMode')
        lightSens=getLightSensors()
        s=simGetObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.05*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do
            res,dist=simReadProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end
        if (opMode==0) then -- We wanna follow the line
            if (math.mod(st,2)>1.5) then
                intensity=1
            else
                intensity=0
            end
            for i=1,9,1 do
                ledColors[i]={intensity,0,0} -- red
            end
            -- Now make sure the light sensors have been read, we have a line and the 4 front prox. sensors didn't detect anything:
            if lightSens and ((lightSens[1]<0.5)or(lightSens[2]<0.5)or(lightSens[3]<0.5)) and (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                if (lightSens[1]>0.5) then
                    velLeft=maxVel
                else
                    velLeft=maxVel*0.25
                end
                if (lightSens[3]>0.5) then
                    velRight=maxVel
                else
                    velRight=maxVel*0.25
                end
            else
                velRight=maxVel
                velLeft=maxVel
                if (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                    -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
                    if (proxSensDist[1]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[1]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[1]/noDetectionDistance))
                    end
                    if (proxSensDist[6]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[6]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[6]/noDetectionDistance))
                    end
                else
                    -- Obstacle in front. Use Braitenberg to avoid it
                    for i=1,4,1 do
                        velLeft=velLeft+maxVel*braitFrontSens_leftMotor[i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                        velRight=velRight+maxVel*braitFrontSens_leftMotor[5-i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                    end
                end
            end
        end
        if (opMode==1) then -- We wanna follow something!
            index=math.floor(1+math.mod(st*3,9))
            for i=1,9,1 do
                if (index==i) then
                    ledColors[i]={0,0.5,1} -- light blue
                else
                    ledColors[i]={0,0,0} -- off
                end
            end
            velRightFollow=maxVel
            velLeftFollow=maxVel
            minDist=1000
            for i=1,8,1 do
                velLeftFollow=velLeftFollow+maxVel*braitAllSensFollow_leftMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                velRightFollow=velRightFollow+maxVel*braitAllSensFollow_rightMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                if (proxSensDist[i]<minDist) then
                    minDist=proxSensDist[i]
                end
            end

            velRightAvoid=0
            velLeftAvoid=0
            for i=1,8,1 do
                velLeftAvoid=velLeftAvoid+maxVel*braitAllSensAvoid_leftMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                velRightAvoid=velRightAvoid+maxVel*braitAllSensAvoid_rightMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
            end
            if (minDist>0.025*s) then minDist=0.025*s end
            t=minDist/(0.025*s)
            velLeft=velLeftFollow*t+velLeftAvoid*(1-t)
            velRight=velRightFollow*t+velRightAvoid*(1-t)
        end

        if (opMode==2) then -- ROS interface
            -- publish camera
            simExtRosInterface_publish(pubCamera,rosGetCameraData())

            -- publish proximity
            proximityData = rosGetProximityData(proxSensDist)
            for i=1,8,1 do
                simExtRosInterface_publish(pubProx[i], proximityData[i])
            end

            -- publish TF
            tf={}
            tf[1] = rosGetTransformStamped(ePuckBase, -1, ePuckName .. "/base_link", "odom")
            for i=1,8,1 do
                tf[i+1] = rosGetTransformStamped(dummyProxSens[i], ePuckBase, ePuckName .. "/base_prox" .. (i-1), ePuckName .. "/base_link")
            end
            tf[10] = rosGetTransformStamped(leftWheel, ePuckBase, ePuckName .. "/left_wheel", ePuckName .. "/base_link")
            tf[11] = rosGetTransformStamped(rightWheel, ePuckBase, ePuckName .. "/right_wheel", ePuckName .. "/base_link")
            tf[12] = rosGetTransformStamped(ePuckBodyTop, ePuckBase, ePuckName .. "/body_top", ePuckName .. "/base_link")
            simExtRosInterface_sendTransforms(tf)

            -- publish IMU
            simExtRosInterface_publish(pubIMU, rosGetImuData())
        end

        simSetJointTargetVelocity(leftMotor,velLeft)
        simSetJointTargetVelocity(rightMotor,velRight)
        actualizeLEDs()
        simSwitchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end

-- Put some initialization code here:

-- Get proximity noise parameters
-- If not set, use default values, specified at the beginning of this file
if (simGetScriptSimulationParameter(sim_handle_self,'isNoise') ~= nil) then
    paramIsNoise=simGetScriptSimulationParameter(sim_handle_self,'isNoise')
end
if (simGetScriptSimulationParameter(sim_handle_self,'NoiseProb') ~= nil) then
    paramNoiseProb=simGetScriptSimulationParameter(sim_handle_self,'NoiseProb')
end
if (simGetScriptSimulationParameter(sim_handle_self,'NoiseMean') ~= nil) then
    paramNoiseMean=simGetScriptSimulationParameter(sim_handle_self,'NoiseMean')
end
if (simGetScriptSimulationParameter(sim_handle_self,'NoiseSD') ~= nil) then
    paramNoiseSD=simGetScriptSimulationParameter(sim_handle_self,'NoiseSD')
end

simSetThreadSwitchTiming(200) -- We will manually switch in the main loop
bodyElements=simGetObjectHandle('ePuck_bodyElements')
leftMotor=simGetObjectHandle('ePuck_leftJoint')
rightMotor=simGetObjectHandle('ePuck_rightJoint')
leftWheel=simGetObjectHandle('ePuck_leftWheel')
rightWheel=simGetObjectHandle('ePuck_rightWheel')
ePuck=simGetObjectHandle('ePuck')
ePuckBase=simGetObjectHandle('ePuck_base')
ePuckBodyTop=simGetObjectHandle('ePuck_ring')
ledLight=simGetObjectHandle('ePuck_ledLight')
proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
dummyProxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
    proxSens[i]=simGetObjectHandle('ePuck_proxSensor'..(i-1))
    dummyProxSens[i]=simGetObjectHandle('dummy_proxSensor'..(i-1))
end
maxVel=120*math.pi/180
ledColors={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}

-- Braitenberg weights for the 4 front prox sensors (avoidance):
braitFrontSens_leftMotor={1,2,-2,-1}
-- Braitenberg weights for the 2 side prox sensors (following):
braitSideSens_leftMotor={-1,0}
-- Braitenberg weights for the 8 sensors (following):
braitAllSensFollow_leftMotor={-3,-1.5,-0.5,0.8,1,0,0,-4}
braitAllSensFollow_rightMotor={0,1,0.8,-0.5,-1.5,-3,-4,0}
braitAllSensAvoid_leftMotor={0,0.5,1,-1,-0.5,-0.5,0,0}
braitAllSensAvoid_rightMotor={-0.5,-0.5,-1,1,0.5,0,0,0}

-- ROS interface initialization
ePuckNumber = simGetNameSuffix(simGetScriptName(sim_handle_self))
if (ePuckNumber == -1) then
    ePuckNumber = 0
else
    ePuckNumber = ePuckNumber + 1
end
ePuckName = 'ePuck' .. ePuckNumber
simAddStatusbarMessage('epuck '.. ePuckName ..' initializing')
-- Check if the required RosInterface is there:
-- http://www.coppeliarobotics.com/helpFiles/en/rosTutorialIndigo.htm
moduleName=0
index=0
rosInterfacePresent=false
while moduleName do
    moduleName=simGetModuleName(index)
    if (moduleName=='RosInterface') then
        rosInterfacePresent=true
    end
    index=index+1
end

-- Accelerometer tube open
accelCommunicationTube=simTubeOpen(0,'accelerometerData'..simGetNameSuffix(nil),1) -- put this in the initialization phase
-- Gyroscope tube open
gyroCommunicationTube=simTubeOpen(0,'gyroData'..simGetNameSuffix(nil),1)

--setup handles for various components
handleCamera=simGetObjectHandle('ePuck_camera')

--create publishers (proximity, imu, camera, ground_truth)
pubCamera = simExtRosInterface_advertise('/' .. ePuckName ..'/camera', 'sensor_msgs/Image')
simExtRosInterface_publisherTreatUInt8ArrayAsString(pubCamera) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)

pubProx = {}
for i=1,8,1 do
    pubProx[i] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity'..(i-1), 'sensor_msgs/Range')
end
pubIMU = simExtRosInterface_advertise('/' .. ePuckName ..'/imu', 'sensor_msgs/Imu')

--create subscribers (leds, movement)
subLED=simExtRosInterface_subscribe('/' .. ePuckName .. '/cmd_led','std_msgs/UInt8MultiArray','rosCallbackCmdLed')
subVel=simExtRosInterface_subscribe('/' .. ePuckName .. '/cmd_vel','geometry_msgs/Twist','rosCallbackCmdVel')

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Put some clean-up code here:

for i=1,9,1 do
    ledColors[i]={0,0,0} -- no light
end
actualizeLEDs()
