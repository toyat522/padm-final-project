import numpy as np

# For animation purposes
JOINT_MOVE_SLEEP = 0.04                    # Sleep time between joint movements
ACTION_SLEEP     = 1                       # Sleep time between actions
MOVE_SLEEP       = 0.03                    # Sleep time for translation (for animation purposes)

# Important constants
NUM_JOINTS       = 7                       # Robot has 7 joints
SUGAR_BOX_POSE   = (-0.1, 0.65, np.pi / 4) # Starting position of sugar box
SPAM_BOX_POSE    = (0.2, 1.1, np.pi / 4)   # Starting position of spam box
JOINT_STEP_SIZE  = 0.02                    # Step size of robot arm interpolation and RRT
GOAL_SAMPLE      = 50                      # Sample goal every X times
GOAL_THRESHOLD   = 0.15                    # If distance less than this, then it is in goal state
PATH_LENGTH      = 200                         # Path length for constraint optimization solver

# Pose of objects after being moved
MOVED_SUGAR_POSE = (0.1, 1.0, -0.6),  (0, 0, 0, 1) 
MOVED_SPAM_POSE  = (0.5, 1.2, -0.69), (0, 0, 0, 1) 
STORED_SPAM_POSE = (0.2, 1.2, -0.69),  (0, 0, 0, 1) 

# Hard-coded joints
INIT_JOINT          = (0.01200158428400755, -0.5697816014289856, 5.6801487517077476e-05, -2.8105969429016113, -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405)
GRAB_SUGAR_JOINT    = (-0.10029298683257405, 0.8075911195804472, 0.06859836909844828, -1.4332957958078696, 0.019711396650709645, 3.024185878659845, 0.6934382572596789)
PLACE_SUGAR_JOINT   = (-0.8333839072116455, 0.8531772629931089, 0.4617390279322891, -1.381649993484035, 0.30737591885150817, 2.8317178374508676, -0.11349712635040898)
GRAB_SPAM_JOINT     = (-0.2952415992141341, 0.7212760573498352, -0.5825837608323721, -1.9038112672075966, 2.43044497001763, 1.7815317165161315, -1.6864013281698946)
DRAWER_CLOSED_INTERMEDIATE = (0.9511187775262453, -1.7396685466779997, -1.4872401065919303, -1.860683910611634, -0.4777817191196396, 2.7740711389454673, 1.1463138823909778)
DRAWER_CLOSED_JOINT = (-1.338462590924967, 1.0771823871221202, 0.18560027115857658, -1.5159392166440764, 1.7676538992294342, 1.87285428791979, -0.2862073274847683)
DRAWER_OPEN_JOINT   = (-1.8639326887555299, 0.9974156273734971, 0.16949275359024293, -1.685524605166771, 1.32792178177285, 1.6055275415848749, -0.30157951704149566)
STORE_SPAM_JOINT    = (0.8141463627352259, -1.7561537326062215, -1.7082003565419186, -1.9041165022131183, -0.06483606045010992, 2.7332450779369095, -0.954724282200727)

