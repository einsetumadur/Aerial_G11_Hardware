from cflib.utils import uri_helper

import mathieu as ma

uri = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E711')
up_period = 20 #ms
###################### GLOBAL VARIABLES ############################

STARTX = 0.2
STARTY = 1.0
BOXHEIGHT = 0.125

STABILIZER = False
MAXTIME = 240 # seconds
LANDRATE = 0.2 # m/s
MAXSPD = 0.3 # m/s
OBSAVOID = True

command_function = ma.get_command

