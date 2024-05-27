from cflib.utils import uri_helper
import controller as cont

uri = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E711')
up_period = 20 #ms
###################### GLOBAL VARIABLES ############################

STARTX = 0.5
STARTY = 1.5
BOXHEIGHT = 0.0 # 0.125

STABILIZER = False
MAXTIME = 240 # seconds
LANDRATE = 0.2 # m/s
MAXSPD = 0.3 # m/s
OBSAVOID = True

command_function = cont.get_command

