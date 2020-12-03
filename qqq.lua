function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end
function __getObjectQuaternion__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectQuaternion(a,b)
end
function sub_target_angle(msg_num)

    for j = 1, 20 do
        sim.setJointTargetPosition(joint[j],msg_num.data[j])
        --sim.addStatusbarMessage("data: " ..msg_num.data[j])
        --20191023
        --sim.setJointTargetVelocity(joint[j],1)
        --sim.setJointForce(joint[j],msg_num.data[j])
    end

end

function getTransformStamped(objHandle,name,relTo,relToName)
    -- This function retrieves the stamped transform for a specific object
    t=sim.getSimulationTime()    --set simulation time
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

if (sim_call_type==sim.syscb_init) then

    joint = {}
    Force_sensor = {}
    objectHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objectName=sim.getObjectName(objectHandle)

    joint[1]=sim.getObjectHandle("joint_0")
    joint[2]=sim.getObjectHandle("joint_1")
    joint[3]=sim.getObjectHandle("joint_2")
    joint[4]=sim.getObjectHandle("joint_3")
    joint[5]=sim.getObjectHandle("joint_4")
    joint[6]=sim.getObjectHandle("joint_5")
    joint[7]=sim.getObjectHandle("joint_6")
    joint[8]=sim.getObjectHandle("joint_7")
    joint[9]=sim.getObjectHandle("joint_8")
    joint[10]=sim.getObjectHandle("joint_9")
    joint[11]=sim.getObjectHandle("joint_10")
    joint[12]=sim.getObjectHandle("joint_11")
    joint[13]=sim.getObjectHandle("joint_12")
    joint[14]=sim.getObjectHandle("joint_13")
    joint[15]=sim.getObjectHandle("joint_14")
    joint[16]=sim.getObjectHandle("joint_15")
    joint[17]=sim.getObjectHandle("joint_16")
    joint[18]=sim.getObjectHandle("joint_17")
    joint[19]=sim.getObjectHandle("joint_18")
    joint[20]=sim.getObjectHandle("joint_tail")

    for i=0, 19 do
        Force_sensor[i+1]=sim.getObjectHandle("Force_sensor#"..i)
    end

    moduleName=0
    index=0
    rosInterfacePresent=false
    while moduleName do
        moduleName=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            rosInterfacePresent=true
        end
        index=index+1
    end

    -- Prepare the float32 publisher and subscriber (we subscribe to the topic we advertise):
    if rosInterfacePresent then

        publisher=simROS.advertise('/simulationTime','std_msgs/Float32')
        pub_force_data=simROS.advertise('/force_data','sensor_msgs/JointState')
        pub_motor_angle=simROS.advertise('/current_motor_angle','std_msgs/Float32MultiArray')
        pub_imu_data=simROS.advertise('/imu_data_vrep','std_msgs/Float32MultiArray')
        --20170624_akiyama_force_xaxis
        pub_forcex_data=simROS.advertise('/force_x','std_msgs/Float32MultiArray')
        -----------
        --20171129_akiyama_torque
        pub_torque_data=simROS.advertise('/torque_data','std_msgs/Float32MultiArray')
        -----------

        --publisher_test=simROS.advertise('/test','snake_msgs_abe/FsensorDataRaw')
        --publisher_test=simROS.advertise('/test','geometry_msgs/Point32')
        --simROS.publisherTreatUInt8ArrayAsString(publisher_test)

        sub_num=simROS.subscribe('/target_motor_angle','std_msgs/Float32MultiArray','sub_target_angle')
 
    end
end

if (sim_call_type==sim.syscb_actuation) then
    -- Send an updated simulation time message, and send the transform of the object attached to this script:

    local force_data_buff = {}
    local imu_data_buff
    local qyro_data_buff
    local qyro_data = {}
    local imu_data = {} 
    local force_data = {}
    local motor_data = {}

    local fsensor_id = {}
    local fsensor_theta = {}
    local fsensor_scale = {}
    local fsensor_data = {}

    local test_data = {}

    --20170624_akiyama_force_xaxis
    local forcex_data = {}
    table.insert(forcex_data,sim.getSimulationTime())
    --20171128_akiyama_torque
    local torque_data = {}
    table.insert(torque_data,sim.getSimulationTime())
    ------------------------

    gyro_data_buff = sim.getStringSignal("GyroData")
    imu_data_buff = sim.getStringSignal("AccelData")
    imu_data = sim.unpackFloatTable(imu_data_buff)
    gyro_data = sim.unpackFloatTable(gyro_data_buff)
   -- sim.addStatusbarMessage("data: " ..imu_data_buff)

    for i = 1, 3 do
        table.insert(imu_data,gyro_data[i])
    end

    for i = 1, 20 do
        result, Force, Torque = sim.readForceSensor(Force_sensor[i])
        theta = math.atan2(Force[3], Force[2])
        scale = math.sqrt(math.pow(Force[2],2) + math.pow(Force[3],2))

        --force_data = sim.unpackFloatTable(force_data_buff[i])
        table.insert(fsensor_id, i-1)
        table.insert(fsensor_theta, theta)
        table.insert(fsensor_scale, scale)
        --table.insert(force_data[i],1,i)
        
        --20170624_akiyama_force_xaxis
        table.insert(forcex_data, Force[3])
        ---------------------
    end
    fsensor_data['header'] = {seq = 0, stamp = sim.getSimulationTime(), frame_id = "none"}
    fsensor_data['name'] = {"fsensor"}
    fsensor_data['position'] = fsensor_id
    fsensor_data['velocity'] = fsensor_theta
    fsensor_data['effort']   = fsensor_scale
   ---[[
    for j = 1, 20 do
       motor_data[j] = sim.getJointPosition(joint[j])
        --sim.addStatusbarMessage( j..": " ..motor_data[j])
       table.insert(torque_data, sim.getJointForce(joint[j]))
    end
    table.insert(motor_data, 1, sim.getSimulationTime())
--]]
    if rosInterfacePresent then
        simROS.publish(publisher,{data=sim.getSimulationTime()})
        
        simROS.publish(pub_force_data, fsensor_data)
        simROS.publish(pub_motor_angle,{data=motor_data})
        simROS.publish(pub_imu_data,{data=imu_data})

        --20170624_akiyama_force_xaxis
        simROS.publish(pub_forcex_data,{data=forcex_data})
        --20171128_akiyama_torque
        simROS.publish(pub_torque_data,{data=torque_data})
        -------------------------
        offset_tf=getTransformStamped(objectHandle,objectName,-1,'world')
        offset_tf.child_frame_id='offset_frame'
        offset_tf.transform.translation.y = offset_tf.transform.translation.y+1
        simROS.sendTransform(offset_tf)
        simROS.sendTransform(getTransformStamped(objectHandle,objectName,-1,'world'))

    --test_data['timestamp'] = sim.getSimulationTime()
    --test_data['theta'] = math.abs(-1.0)
    --test_data['scale'] = 1.0
    --test_data['x'] = 1.0
    --test_data['y'] = 1.0
    --test_data['z'] = 1.0

        --simROS.publish(publisher_test, test_data)
        -- To send several transforms at once, use simROS.sendTransforms instead
    end

end

if (sim_call_type==sim.syscb_cleanup) then
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(publisher)
        simROS.shutdownPublisher(pub_force_data)
        simROS.shutdownPublisher(pub_motor_angle)
        simROS.shutdownPublisher(pub_imu_data)

        simROS.shutdownPublisher(pub_forcex_data)
        simROS.shutdownPublisher(pub_torque_data)

        --simROS.shutdownPublisher(publisher_test)

        simROS.shutdownSubscriber(sub_num)
    end
end
