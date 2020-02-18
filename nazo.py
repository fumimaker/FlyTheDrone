


from dronekit import connect 
import time                


connection_string = "tcp:localhost:5760"


print( "FC: %s" % (connection_string) )  
vehicle = connect(connection_string, wait_ready=True)  

try:
    while True:
        print("--------------------------" )
        print(" GPS: %s" % vehicle.gps_0 )
        print(" Battery: %s" % vehicle.battery )
        print(" Last Heartbeat: %s" % vehicle.last_heartbeat )
        print(" Is Armable?: %s" % vehicle.is_armable )
        print(" System status: %s" % vehicle.system_status.state )
        print(" Mode: %s" % vehicle.mode.name )

        time.sleep(1)

except( KeyboardInterrupt, SystemExit):
    print( "SIGINT" )


vehicle.close()
