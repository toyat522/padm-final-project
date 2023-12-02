import numpy as np

SUGAR_BOX_POSE = (-0.15, 0.65, np.pi / 4) # Starting position of sugar box
SPAM_BOX_POSE  = (0.2, 1.1, np.pi / 4)    # Starting position of spam box
MOVE_SLEEP     = 0.03                     # Sleep time for translation (for animation purposes)
POS_STEP_SIZE  = 0.01                     # Step size of robot arm interpolation and RRT

GRAB_SUGAR_JOINT    = (-0.07, np.pi / 2, np.pi, 0, np.pi / 4, np.pi, np.pi)
PLACE_SUGAR_JOINT   = (-np.pi / 5, np.pi / 2, np.pi, 0, np.pi / 4, np.pi, np.pi)
GRAB_SPAM_JOINT     = (-np.pi / 5 + 0.05, 7 * np.pi / 40, 0, -12 * np.pi / 20, -3 * np.pi / 4, 3 * np.pi / 4, np.pi)
DRAWER_CLOSED_JOINT = (-7 * np.pi / 10 + 0.05, 3 * np.pi / 5, 5 * np.pi / 12 + 0.05, -3 * np.pi / 5, -3 * np.pi / 4, 3 * np.pi / 4, np.pi)
DRAWER_OPEN_JOINT   = (-np.pi + 0.2, 3 * np.pi / 5, np.pi / 3, -3 * np.pi / 4 + 0.1, -3 * np.pi / 4, 7 * np.pi / 8, np.pi)
STORE_SPAM_JOINT    = (-7 * np.pi / 8 + 0.2, 3 * np.pi / 8, np.pi / 2, -3 * np.pi / 4, -np.pi / 4, 7 * np.pi / 8, np.pi)


