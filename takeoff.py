from dronekit import connect, VehicleMode
import time

connection_string = '/dev/ttyS0'
vehicle = connect(connection_string, wait_ready=True)

def printStatus():
    print("--------------------------" )
    print(" System status: %s" % vehicle.system_status.state)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" Armed: %s" % vehicle.armed) 
    print(" Mode: %s" % vehicle.mode.name )
    print(" GlobalLocation: %s" % vehicle.location.global_frame)
    print(" Altitude: %s" % vehicle.location.global_relative_frame.alt)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        printStatus()
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        printStatus()
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

printStatus()
arm_and_takeoff(3)
print "wait for 10sec..."
for i in range(10):
    printStatus()
    time.sleep(1)
print "Landing"
vehicle.mode = VehicleMode("LAND")
while vehicle.armed:
    print " Altitude: ", vehicle.location.global_relative_frame.alt
    time.sleep(1)

print "Landed. Close connection..."
vehicle.close()
print "completed."