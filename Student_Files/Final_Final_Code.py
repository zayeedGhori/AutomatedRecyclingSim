import sys, math, time
sys.path.append('../')
from Common.project_library import *

# Modify the information below according to you setup and uncomment the entire section

# 1. Interface Configuration
project_identifier = 'P3B' # enter a string corresponding to P0, P2A, P2A, P3A, or P3B
ip_address = '169.254.57.135' # enter your computer's IP address
hardware = False # True when working with hardware. False when working in the simulation

# 2. Servo Table configuration
short_tower_angle = 315 # enter the value in degrees for the identification tower 
tall_tower_angle = 90 # enter the value in degrees for the classification tower
drop_tube_angle = 180 #270# enter the value in degrees for the drop tube. clockwise rotation from zero degrees

# 3. Qbot Configuration
bot_camera_angle = -21.5 # angle in degrees between -21.5 and 0

# 4. Bin Configuration
# Configuration for the colors for the bins and the lines leading to those bins.
# Note: The line leading up to the bin will be the same color as the bin 

# All bins offset at 0.15 m
# Bin colors: 1, red; 2, green; 3, blue; 4, white
bin1_offset = 0.15 # offset in meters
bin1_color = [1,0,0] # e.g. [1,0,0] for red
bin2_offset = 0.15
bin2_color = [0,1,0]
bin3_offset = 0.15
bin3_color = [0,0,1]
bin4_offset = 0.15
bin4_color = [1,1,1]

#--------------- DO NOT modify the information below -----------------------------

if project_identifier == 'P0':
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    bot = qbot(0.1,ip_address,QLabs,None,hardware)
    
elif project_identifier in ["P2A","P2B"]:
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    arm = qarm(project_identifier,ip_address,QLabs,hardware)

elif project_identifier == 'P3A':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    configuration_information = [table_configuration,None, None] # Configuring just the table
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    
elif project_identifier == 'P3B':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    qbot_configuration = [bot_camera_angle]
    bin_configuration = [[bin1_offset,bin2_offset,bin3_offset,bin4_offset],[bin1_color,bin2_color,bin3_color,bin4_color]]
    configuration_information = [table_configuration,qbot_configuration, bin_configuration]
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    bins = bins(bin_configuration)
    bot = qbot(0.1,ip_address,QLabs,bins,hardware)
    

#---------------------------------------------------------------------------------
# STUDENT CODE BEGINS
#---------------------------------------------------------------------------------

# The home position for the qbot
qbot_home = [1.5,0,0]

# variables for qarm positions
home_coord = [0.406, 0.0, 0.483]
container_coord = [0.661, 0.0, 0.275]
load_coord = [0.019, -0.515, 0.494]
hopper_coord = [0.012, -0.338, 0.548]

# Holds data for all bins
bin_data = {
    'Bin01' : {'offset': bin1_offset,
               'color' : bin1_color},
    'Bin02' : {'offset': bin2_offset,
               'color' : bin2_color},
    'Bin03' : {'offset' : bin3_offset,
               'color' : bin3_color},
    'Bin04' : {'offset' : bin4_offset,
               'color' : bin4_color}
}

# Joshua Lim How
# Returns a bin to a dispensed container
def sort():
    # Get a random container ID from 1 to 6
    ID=random.randrange(1,7)

    # Dispense container and get container properties
    material, mass, location = table.dispense_container(ID, True)
    
    return location, mass # Return location and mass

# Zayeed Ghori
# Move the qarm to load a container onto the qbot hopper
def place_container():
    # Move arm to container location
    arm.move_arm(*container_coord)
    time.sleep(2)

    # Close gripper 35 degrees
    arm.control_gripper(35)
    time.sleep(1)

    # Move arm to home location, without opening the gripper
    arm.move_arm(*home_coord)
    time.sleep(2)

    # Move arm to hopper location, adjacent to the qbot hopper
    arm.move_arm(*hopper_coord)
    time.sleep(2)

    # Move arm to load location, inside the hopper
    arm.move_arm(*load_coord)
    time.sleep(2)

    # Open gripper 35 degrees
    arm.control_gripper(-35)
    time.sleep(1)

    arm.home() # return to home position

# Zayeed Ghori
# Loading function, uses qarm to move container from table to qbot.
# Also determines if a container should be loaded onto the bot.
def load(location, container_mass):
    # Variables that are used to determine if a container should be loaded
    total_mass, total_num_containers, new_location = 0, 0, ''

    place_container() # Place the first container

    # Update variables to match the containers on the hopper
    total_num_containers += 1
    total_mass += container_mass

    # While a new container can be placed
    while (True):

        # Get bin number and container mass from dispense phase
        new_location, container_mass = sort()
        time.sleep(0.5)

        # If there are less than 3 containers and
        # the next container bin is the same as the first location and
        # the total mass of the containers on the hopper +
        # the next container mass is less than 90g,
        # load the container
        if (
            total_num_containers < 3 and
            new_location == location and
            total_mass + container_mass < 90
            ):

            place_container() # Put the container on the hopper
            

            # Update variables to determine if conditions are still met
            # next iteration
            total_num_containers += 1
            total_mass += container_mass
 
        # Otherwise return the location of the container that does not
        # get put onto the hopper and stop the loop
        else:
            return new_location

# Zayeed Ghori
# Transfers containers from the recycling station to the appropriate
# bin location
def transfer(location):

    # Activate color sensor
    bot.activate_color_sensor()

    # Follow line until near a bin, get to a suitable deposit location,
    # then stop
    while (True):

        # Read the percieved [R, G, B] value from the color sensor
        color = bot.read_color_sensor()[0]

        # if the percieved color is not the same as the correct bin color
        # then follow the line
        if (color != bin_data[location]['color']):
            
            follow_line()

        # When the proper color is sensed, follow the line for 8s more
        # then stop the qbot.
        else:
            bot.deactivate_color_sensor() # Deactivate color sensor
            bot.stop() # Stop the bot

            ## Timer that runs follow_line() for follow_time in seconds ##
            
            start_time = time.time() # time follow_line() starts
            follow_time = 8 # Desired time for follow_line() to run, 8s
            time_elapsed = 0 # The elapsed time, initially 0s

            # While the elapsed time is less then the desired time,
            # continue following the line
            while (time_elapsed <= follow_time):
                time_elapsed = time.time() - start_time # Update elapsed time
                follow_line()
    
            bot.stop() # Stop the bot
            
            return # End

# Zayeed Ghori
# Qbot follows the line by checking if both line sensors can sense the line
# otherwise change the speed of the appropriate side
def follow_line():
    # If one side is not on the line, slow down the other side

    sensors = bot.line_following_sensors() # Store sensor values

    # If both sensors sense a line, go forward at 0.05m/s
    if sensors == [1,1]:
        bot.set_wheel_speed([0.05,0.05])

    # If the right side does not sense a line, move the left side at 0.025m/s
    # and the right at 0.05 m/s, turning the qbot left
    elif sensors == [1,0]:
        bot.set_wheel_speed([0.025,0.05])

    # If the left side does not sense a line, move the right side at 0.025m/s
    # and the left at 0.05 m/s, turning the qbot left
    elif sensors == [0,1]:
        bot.set_wheel_speed([0.05,0.025])

    # Otherwise continusly turn left to try and find the line
    else:
        bot.set_wheel_speed([0, 0.025])

# Joshua Lim How
# Deposits the container(s) into the bin
def deposit():
    bot.activate_linear_actuator()
    bot.dump()
    bot.deactivate_linear_actuator()
    
# Joshua Lim How
# Brings the bot from a bin location to the home coordinates within an error
def return_home():
    # While the qbot is not at qbot_home within a certain error,
    # continue following the line
    while (True):
        
        position = bot.position() # Store current bot's position
        x, y, z = position # Split position into its components
        
        home_x, home_y, home_z = qbot_home # Split qbot-home into its coordinates

        deadzone = 0.05 # The bot can be off the home coordinates by 0.05m
        
        # if the x and y positions are within the deadzone of the home coordinates,
        # stop the bot
        if (home_x - deadzone <= x <= home_x + deadzone and
            home_y - deadzone*0.5 <= y <= home_y + deadzone*0.5):

            bot.set_wheel_speed([0, 0]) # Set the wheel speed to 0
            bot.rotate(5) # rotate 5 degrees for a better loading position

            # Stop the bot and end
            bot.stop()
            return

        # Otherwise continue following the line
        else:
            follow_line()

# Zayeed and Joshua
# The main function
def main():
    # For the first time, get an initial location and mass and deposit a
    # container
    location, container_mass = sort()

    # Sort containers forever!
    while (True):
        
        # Get the new_location which is the location of the next cycle from
        # load() after it completes using location and container_mass
        print('\nLoading...')
        new_location = load(location, container_mass)

        # transfer container(s) to the bin location
        print('\nTransferring containers to ' + location + '.')        
        transfer(location)

        # deposit the container(s)
        print('\nDepositing container(s)...')
        deposit()

        # return home
        print('\nDeposit successful, returning home!')
        return_home()
        
        location = new_location # the location for the cycle is updated

main()



#---------------------------------------------------------------------------------
# STUDENT CODE ENDS
#---------------------------------------------------------------------------------
