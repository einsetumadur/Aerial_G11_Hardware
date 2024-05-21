from cflib.utils import uri_helper
import gilles as gi
import mathieu as ma
import flightphases as ff

uri = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E711')
up_period = 20 #ms
###################### GLOBAL VARIABLES ############################

STABILIZER = False
MAXTIME = 240 # seconds
LANDRATE = 0.2 # m/s
MAXSPD = 0.3 # m/s
OBSAVOID = True

command_function = ff.get_command

