# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0
TRACE=1

# Enable Cordio library
LIB_CORDIO = 1

# Cordio library options
INIT_PERIPHERAL = 1
INIT_CENTRAL = 0

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
TRACE = 1

# Add services directory to build
IPATH += services
VPATH += services

# SET advertising name
PROJ_CFLAGS += -DADV_NAME=\"This\ is\ the\ real\ world\"


### CONFIGURE security
# /*! Authentication and bonding flags */
#PROJ_CFLAGS += -DAUTH_BOND='DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG | DM_AUTH_MITM_FLAG'

# /*! Initiator key distribution flags */
#PROJ_CFLAGS += -DINIT_KEY_DIST='DM_KEY_DIST_IRK'

# /*! Responder key distribution flags */
#PROJ_CFLAGS += -DRESP_KEY_DIST='DM_KEY_DIST_LTK | DM_KEY_DIST_IRK'

# /*! TRUE if Out-of band pairing data is present */
#PROJ_CFLAGS += -DOUT_OF_BAND=FALSE

# /*! TRUE to initiate security upon connection*/
PROJ_CFLAGS += -DINIT_SECURITY=TRUE




