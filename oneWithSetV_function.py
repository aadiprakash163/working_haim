from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import time
from math import sqrt, pi

# import matplotlib.pyplot as plt


try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
PORT = 8873

lastVeh=[0,0,0,0,0,0,0,0,0,0,0,0] #array that saves the latest vehicle entered in a lane
lastVehDepTime = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #array that saves depart time of the latest vehicle in lane
reservation_stack = [[[0,0]for i in range(11)] for j in range(17)] # reservation details at each CP
totalLength = 100.0  # approach length
delay_array = []  
velocity_array = []
safety_gap = 0.1
acc = 3.0; dec = 3.0


def generate_routefile():
    random.seed(50)  # make tests reproducible
    N = 3600 # number of time steps
    K = 1
    # demand per second from different directions
    pWE = 1. /K
    pEW = 1. /K
    pNS = 1. /K
    pES = 1. /K
    pEN = 1. /K
    pWS = 1. /K
    pWN = 1. /K
    pNE = 1. /K
    pNW = 1. /K
    pSE = 1. /K
    pSW = 1. /K
    pSN = 1. /K
    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="type1" accel="%f" decel="%f" sigma="0" length="3" width = "2" minGap="0.0" minGapLat = "0.0" maxSpeed="25" guiShape="passenger"/>
        <vType id="type2" accel="%f" decel="%f" sigma="0" length="3" width = "2" minGap="0.0" minGapLat = "0.0" maxSpeed="25" guiShape="passenger"/>
    	<route id="WtoE" edges="1i 2o" />
    	<route id="WtoS" edges="1i 3o" />
    	<route id="WtoN" edges="1i 4o" />
    	<route id="EtoS" edges="2i 3o" />
    	<route id="EtoN" edges="2i 4o" />
    	<route id="EtoW" edges="2i 1o" />
    	<route id="StoW" edges="3i 1o" />
    	<route id="StoE" edges="3i 2o" />
    	<route id="StoN" edges="3i 4o" />
    	<route id="NtoS" edges="4i 3o" />
    	<route id="NtoW" edges="4i 1o" />"
    	<route id="NtoE" edges="4i 2o" />""" %(acc,dec,acc,dec), file=routes)
        lastVeh = 0
        vehNr = 0
        for i in range(N):
            # All straight going paths
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="WtoE_%i" type="type1" route="WtoE" depart="%i" departSpeed="%f" departLane= "1"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="EtoW_%i" type="type1" route="EtoW" depart="%i" departSpeed="%f" departLane= "1"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="NtoS_%i" type="type2" route="NtoS" depart="%i" departSpeed="%f" departLane= "1"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSN:
                print('    <vehicle id="StoN_%i" type="type1" route="StoN" depart="%i" departSpeed="%f" departLane= "1"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            # All left turning paths
            if random.uniform(0, 1) < pES:
                print('    <vehicle id="EtoS_%i" type="type1" route="EtoS" depart="%i" departSpeed="%f" departLane= "2"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSW:
                print('    <vehicle id="StoW_%i" type="type1" route="StoW" depart="%i" departSpeed="%f" departLane= "2"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pWN:
                print('    <vehicle id="WtoN_%i" type="type1" route="WtoN" depart="%i" departSpeed="%f" departLane= "2"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNE:
                print('    <vehicle id="NtoE_%i" type="type1" route="NtoE" depart="%i" departSpeed="%f" departLane= "2"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i

            # All right turning paths
            if random.uniform(0, 1) < pEN:
                print('    <vehicle id="EtoN_%i" type="type1" route="EtoN" depart="%i" departSpeed="%f" departLane= "0"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pWS:
                print('    <vehicle id="WtoS_%i" type="type1" route="WtoS" depart="%i" departSpeed="%f" departLane= "0"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNW:
                print('    <vehicle id="NtoW_%i" type="type1" route="NtoW" depart="%i" departSpeed="%f" departLane= "0"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSE:
                print('    <vehicle id="StoE_%i" type="type1" route="StoE" depart="%i" departSpeed="%f" departLane= "0"/>' % (
                    vehNr, i,random.uniform(5,17)), file=routes)
                vehNr += 1
                lastVeh = i
            
        print("</routes>", file=routes)
    print("Number of vehicles generated: ", vehNr)

def get_conflict_points(routeId):
    """ 
    Assign codes to the routes
    Codes are assigned in clockwise direction starting from W side in the following order
    1). Left turn(0,1,2,3) 2). Straight(4,5,6,7) 3). Right turn(8,9,10,11)
    """
    if routeId == 'WtoN':
        rId=0; cp1 = 11; cp2 = 16; cp3 = 13; cp4 = 3         
    elif routeId == 'NtoE':
        rId=1; cp1 = 2; cp2 = 13; cp3 = 14; cp4 = 6
    elif routeId == 'EtoS':
        rId=2; cp1 = 5; cp2 = 14; cp3 = 15; cp4 = 9
    elif routeId == 'StoW':
        rId=3; cp1 = 8; cp2 = 15; cp3 = 16; cp4 = 12
    elif routeId == 'WtoE':
        rId=4; cp1 = 10; cp2 = 9; cp3 = 8; cp4 = 7
    elif routeId == 'NtoS':
        rId=5; cp1 = 1; cp2 = 12; cp3 = 11; cp4 = 10
    elif routeId == 'EtoW':
        rId=6; cp1 = 4; cp2 = 3; cp3 = 2; cp4 = 1
    elif routeId == 'StoN':
        rId=7; cp1 = 7; cp2 = 6; cp3 = 5; cp4 = 4
    elif routeId == 'WtoS':
        rId=8; cp1=cp2=cp3=cp4= 0
    elif routeId == 'NtoW':
        rId=9; cp1=cp2=cp3=cp4= 0
    elif routeId == 'EtoN':
        rId=10; cp1=cp2=cp3=cp4= 0
    else: 
        rId=11; cp1=cp2=cp3=cp4= 0
        
    return(rId,cp1,cp2,cp3,cp4)


def get_cp_distances(routeId):
    """
    Returns intersection lenghts and distances between Conflict points
    """
    l = 3.5 # width of one lane
    R = 7*l/2
    theta = 1.12788528 # in radians
    beta = 0.541099526 # in radians


    # if routeId == 'WtoE' or routeId=='EtoW' or routeId=='NtoS' or routeId=='StoN': #all straight paths
    #     intersectionLength = 18.5; d1 = 4.6; d2 = 4.128613; d3 = 1.042773; d4 = 4.128613      
    # elif routeId=='WtoN' or routeId=='NtoE' or routeId=='EtoS' or routeId=='StoW': #all left turns
    #     intersectionLength = 16.9714285; d1 = 4.783443; d2 = 1.060431; d3 = 5.283678; d4 = 1.060431
    # else:
    #     intersectionLength = 2.357143; d1=d2=d3=d4 = 0 

    if routeId == 'WtoE' or routeId=='EtoW' or routeId=='NtoS' or routeId=='StoN': #all straight paths
        intersectionLength = 6*l
        d1 = 1.5*l
        d2 = l*(6-sqrt(10.0))
        d3 = l*sqrt(10)
        d4 = 4.5*l      
    elif routeId=='WtoN' or routeId=='NtoE' or routeId=='EtoS' or routeId=='StoW': #all left turns
        intersectionLength = (pi/2)*3.5*l
        d1 = 3.5*l*(pi/2 - theta)
        d2 = 3.5*l*beta
        d3 = 3.5*l*(pi/2 - beta)
        d4 = 3.5*l*(theta)
    else:
        intersectionLength = (pi/2)*l*0.5; d1=d2=d3=d4 = 0 
    return(intersectionLength,d1,d2,d3,d4)

def insert_in_stack(cp,intime,passing_time, vid):
    """
    Updates the reservation stack, sort it and deletes the last reseravtion in it.
    """
    reservation_stack[cp].append([intime, intime+passing_time])
    reservation_stack[cp] = sorted(reservation_stack[cp])
    # if reservation_stack[cp][-1][0] < reservation_stack[cp][-2][1]:  # if in time of this vehicle is less than out time of previous vehicle
    #     print("*****************overlapping reservation********************")
    for i in range(len(reservation_stack[cp])-1):
        assert reservation_stack[cp][i][1] <= reservation_stack[cp][i+1][0], "Assertion error occured"

    del reservation_stack[cp][0]
    if cp ==8:
        print("CP 8 reservation for ", vid, "is: ", intime )

def find_window(V_temp,vel,arrival_time,cp,d,length):
    """
    Return True if the vehicle has a window at the given CP and given velocity else returns False 
    """
    
    tt1 = abs(V_temp-vel)/acc
    tt2 = (totalLength-abs(V_temp*V_temp - vel*vel)/(2*acc)+d)/V_temp
    pass_time = 2*length/V_temp
    intime = arrival_time + (tt1 + tt2)*1000
    outtime = intime + pass_time*1000 + safety_gap *1000
    for i in range(len(reservation_stack[cp])-1):
        if reservation_stack[cp][i][1] < intime and reservation_stack[cp][i+1][0] > outtime:
            return True
        else: 
            continue
    return False

def detect_conflict(V, vel, in_time, length, c3, c4, d3, d4):
    tt1 = abs(V - vel)/acc
    tt2 = (totalLength - abs(V*V - vel*vel)/(2*acc) + d3) / V
    b_time = tt1 + tt2
    reach_time = in_time + b_time*1000
    # leave_time = reach_time + (length / V) * 1000
    cp3_clear = True
    # for i in range(len(reservation_stack[c3])-1):
    #     cp3_clear = cp3_clear or (reservation_stack[cp3][i][1] < intime and reservation_stack[cp3][i+1][0] > outtime)
    if reservation_stack[c3][-1][1] > reach_time:
        cp3_clear = False


    tt1 = abs(V - vel)/acc
    tt2 = (totalLength - abs(V*V - vel*vel)/(2*acc) + d4) / V
    b_time = tt1 + tt2
    reach_time = in_time + b_time*1000
    # leave_time = reach_time + (length / V) * 1000
    cp4_clear = True
    # for i in range(len(reservation_stack[c4])-1):
    #     cp4_clear = cp4_clear or (reservation_stack[cp4][i][1] < intime and reservation_stack[cp4][i+1][0] > outtime)
    if reservation_stack[c4][-1][1] > reach_time:
        cp3_clear = False

    if cp3_clear and cp4_clear: return True
    else: return False

# def get_vreservation(VehId, vel, arrival_time, cp1, cp2, cp3, cp4, d1, d2, d3, d4, V_lane):
    
        
#     return(min(V1, V2, V3, V4))

def Nem(arrival_time, cp1, cp2, arrival_vel, dis_to_c1, dis_to_c2,v_max):
    # double D, ti, vi, to, T, v1, v2;
    # safety_gap = 1.0
    ti = arrival_time;
    to = reservation_stack[cp1][-1][1] + safety_gap*1000;
    T = (to-ti)/1000.0;
    vi = arrival_vel;

    D = dis_to_c1; 

    if(to==safety_gap*1000 or (acc*T*(acc*T + 2*vi) - 2*acc*D) < 0 or to <= ti):
        v1 = v_max
    elif(vi*T < D): # if acceleration is needed
        v1 = (vi + acc*T) - sqrt(acc*T*(acc*T + 2*vi) - 2*acc*D);
    elif(vi*T > D): # if deceleration is needed
        v1 = (vi - acc*T) + sqrt(acc*T*(acc*T - 2*vi) + 2*acc*D);
    else: v1 = vi
    # v1 = min(v1, v_max);
    
    to = reservation_stack[cp2][-1][1] + safety_gap*1000;
    T = (to-ti)/1000.0;
    D = dis_to_c2 

    if(to==safety_gap*1000 or (acc*T*(acc*T + 2*vi) - 2*acc*D) < 0 or to <= ti):
        v2 = v_max
    elif(vi*T < D):
    	v2 = (vi + acc*T) - sqrt(acc*T*(acc*T + 2*vi) - 2*acc*D);
    elif(vi*T > D):
        v2 = (vi - acc*T) + sqrt(acc*T*(acc*T - 2*vi) + 2*acc*D);
    else: v2 = vi;
    # v2 = min(v2, v_max);

    # v_fefs_array[v_id] = min(v1, v2);
    # v_fefs = v_fefs_array[v_id];
    return min(v1, v2, v_max)


def setVelocity(VehId):
    V_MAX = 17.0    
    # safety_gap = 1.0
    V_final = 0
    traci.vehicle.setSpeedMode(VehId, 0)
    
    # Get all the required details of the vehicle
    routeId = traci.vehicle.getRouteID(VehId)
    vel = traci.vehicle.getSpeed(VehId)										# To be verified
    arrival_time = traci.simulation.getCurrentTime()     					# To be verified
    length = traci.vehicle.getLength(VehId)
    
    #From the routeId, get cp numbers
    rId, cp1, cp2, cp3, cp4 = get_conflict_points(routeId)  

    #From the routeId, get cp distances
    intersectionLength, d1, d2, d3, d4 = get_cp_distances(routeId)  

    V = V_MAX
    

    trans_time1 = (V-vel)/acc # Velocity transition time
    trans_time2 = (totalLength-(abs((V*V-vel*vel)/(2*acc)))+ intersectionLength + 50)/V 
    b_time = trans_time1 + trans_time2
    
    dep_time = arrival_time + (b_time)*1000.0
    
    if lastVeh[rId] != 0.0:     
        while ((lastVehDepTime[rId]) + safety_gap*1000> dep_time ):# and V > 0.2:
            V=V-0.01
            trans_time1 = abs((V-vel)/acc)
            trans_time2 = (totalLength-abs((V*V-vel*vel))/(2*acc)+intersectionLength + 50)/V
            b_time = trans_time1 + trans_time2
            dep_time = arrival_time + b_time * 1000.0
            if V<0:
                print('V_lane crossed zero...Exiting')
                exit()
    
    V_lane = V
    
    # print("V_lane: %r" %V_lane)

    if routeId=="WtoS" or routeId=="StoE" or routeId == "EtoN" or routeId == "NtoW":
        V_final = V_lane
        # traci.vehicle.setSpeedMode(VehId,0)        
        # traci.vehicle.setSpeed(VehId,V_final)        
        lastVehDepTime[rId] = dep_time
        lastVeh[rId]=VehId
        # print("Right bound vehicle ", VehId, "'s velocity is: ", V_lane)
        return (V_final, vel, abs(V_final-vel)/acc)
        
    else:
    
        V_fefs = Nem(arrival_time, cp1, cp2, vel, totalLength + d1, totalLength + d2,V_lane)
        V = V_lane        
        c1_clear =c2_clear =c3_clear =c4_clear = False
        while not(c1_clear and c2_clear and c3_clear and c4_clear):# and V_temp > 0.01:
            c1_clear = find_window(V,vel,arrival_time,cp1,d1,length)
            c2_clear = find_window(V,vel,arrival_time,cp2,d2,length)
            c3_clear = find_window(V,vel,arrival_time,cp3,d3,length)
            c4_clear = find_window(V,vel,arrival_time,cp4,d4,length)
            V -= 0.01
            if V<V_fefs:
                V = 0; break

        V_window = V
        # if V_window>0:
            # print("\n$$$$$$$$$$$$$$$$  V_window for %r is %r \n" %(VehId,V_window))

        V_reservation = min(Nem(arrival_time, cp3, cp4, vel, totalLength+d3, totalLength+d4, V_fefs), V_fefs)

        no_conflict = detect_conflict(V_fefs, vel, arrival_time, length, cp3, cp4, d3, d4)

        if V_window > 0: 
            V_final = V_window; 
            print("V_window is the final velocity %r" %V_window)
        else:
            if no_conflict: 
            	V_final = V_fefs
            	print(VehId, ": V_fefs is final velocity ")
            else: 
            	V_final = V_reservation
            	print(VehId, ": V_reservation is final velocity ")

        # V_final = V_fefs

        print(VehId, "      :    ", V_final)        
        time = abs((V_final - vel)/acc)
        passing_time = (2*length / V_final) * 1000 

        trans_time1 = abs((V_final - vel)/acc)
        trans_time2 = (totalLength - (abs(V_final*V_final - vel*vel)/(2*acc)) + d1) / V_final
        dep_time = arrival_time + (trans_time1+ trans_time2)*1000
        # print("cp1 time: %r" %dep_time)
        insert_in_stack(cp1, dep_time, passing_time, VehId)

        trans_time2 = (totalLength - (abs(V_final*V_final - vel*vel)/(2*acc)) + d2) / V_final
        dep_time = arrival_time + (trans_time1+ trans_time2)*1000
        # print("cp2 time: %r" %dep_time)
        insert_in_stack(cp2, dep_time, passing_time, VehId)

        trans_time2 = (totalLength - (abs(V_final*V_final - vel*vel)/(2*acc)) + d3) / V_final
        dep_time = arrival_time + (trans_time1+ trans_time2)*1000
        # print("cp3 time: %r" %dep_time)
        insert_in_stack(cp3, dep_time, passing_time, VehId)

        trans_time2 = (totalLength - (abs(V_final*V_final - vel*vel)/(2*acc)) + d4) / V_final
        dep_time = arrival_time + (trans_time1+ trans_time2)*1000
        # print("cp4 time: %r" %dep_time)
        insert_in_stack(cp4, dep_time, passing_time, VehId)

        trans_time2 = (totalLength - (abs(V_final*V_final - vel*vel)/(2*acc)) + intersectionLength + 50) / V_final
        dep_time = arrival_time + (trans_time1+ trans_time2)*1000
        # print("dep time: %r" %dep_time)
        lastVehDepTime[rId] = dep_time


        lastVeh[rId] = VehId
        return (V_final, vel, time)

vehicles_list = []
def update_vehice_velocities(step):
    global vehicles_list
    # vehicles_list = [vehicle for vehicle in vehicles_list if vehicle[1]-1 >= step]
    vehicles_list = [vehicle for vehicle in vehicles_list if traci.vehicle.getSpeed(vehicle[0]) != vehicle[3]]
    # print(step, vehicles_list)
    for vehicle in vehicles_list:
        v_id = vehicle[0]        
        delta_v = vehicle[2]
        v_final = vehicle[3]
        current_vel = traci.vehicle.getSpeed(v_id)
        # traci.setSpeedMode(v_id, 0)
        if vehicle[1] <= step:
            traci.vehicle.setSpeed(v_id, v_final)
            continue

        # while abs(traci.vehicle.getSpeed(v_id) - (current_vel + delta_v)) > 0.1:
        
        # if current_vel != vehicle[4]:
        #     traci.vehicle.setSpeed(v_id, (vehicle[4] + delta_v))
        #     vehicle[4] = vehicle[4] + delta_v
        # if v_id == "StoN_22": print(vehicle, current_vel)
        # assert current_vel == vehicle[4], "Velocity update did not happen"
        # else:
        traci.vehicle.setSpeed(v_id, (current_vel + delta_v))        
        vehicle[4] = current_vel + delta_v

    return


      

    

def run(): 
    """execute the TraCI control loop"""
    traci.init(PORT)
    # start = time.clock()
    # vehIn = False   
    # temp = 0
    step_len = 0.1 #sec
    step = 0
    # # Untill there is no vehicle left to enter
    while traci.simulation.getMinExpectedNumber() > 0: 
        
        traci.simulationStep()
        vehIdsInSimStep = traci.simulation.getDepartedIDList()
        # print(vehIdsInSimStep)
        # numVehInSimStep = traci.simulation.getDepartedNumber()

        for vehId in vehIdsInSimStep:
            # vehId = vehIdsInSimStep[i]
            
            (V_final, vel, dur) = setVelocity(vehId) # dur is in seconds
            num_step = max(round(dur / step_len), 1)
            delta_v = (V_final - vel) / num_step
            end_step = step + num_step + 1
            # if vehId == 'WtoN_76':
            # print("Vehid: %r, v_in: %r, V_final: %r, duration: %r, steps: %r, step: %r, end_step: %r" %(vehId, vel, V_final, dur, num_step, step, end_step))                           
            vehicles_list.append([vehId, end_step, delta_v, V_final, vel])
            
        update_vehice_velocities(step)
        


        # print(step, vehicles_list)    
        step += 1 
    # end = time.clock()
    # # print('Avg time per vehicle:', (end-start)/vehEntered)
    # plt.plot(datapoints)
    # plt.show()
    # # print(datapoints)

    # print("Average delay of all the vehicles: ", delayStack[-1])
    # print("Average velocity: ", Avg_vel)

    # print("Max delay in ms: %r" %max(delay_array))
    # print("Min delay in ms: %r" %(min(delay_array)))
    # print("Averge delay: %r" %(sum(delay_array)/len(delay_array)))
    # print("Max vel: %r" %max(velocity_array))
    # print("Min vel: %r" %(min(velocity_array)))
    # print("Averge vel: %r" %(sum(velocity_array)/len(velocity_array)))
    # print(delay_array)
    traci.close()
    sys.stdout.flush()

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


if __name__ == "__main__":
    options = get_options()
    generate_routefile()

    #exit(0)
    
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    #generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen(['sumo-gui', "-c", "data/cross.sumocfg", "--step-length", "0.1", "--fcd-output", "fcd_output.xml", 
                                    "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    run()
    
    sumoProcess.wait()