# Dagu_ComMotionCommon
Common library base for both the Dagu ComMotion controller firmware and an upcoming Dagu ComMotion host library currently in development.

This library is required in order to be able to compile later versions of the Dagu ComMotion firmware: https://github.com/sgparry/Dagu_ComMotion. It is also required if you wish to use the upcoming Dagu ComMotion Host library on your host MCU.

## Installation

To install, first ensure you have Git installed. Open an OS or Git Bash shell. Change directory to the Libraries sub-folder of your Arduino sketchbook area:

`cd ~/Arduino/Libraries`

Use git to pull the source:
 
`git clone https://github.com/sgparry/Dagu_ComMotionCommon.git`

Restart the Arduino IDE.

If you wish to use the library yourself, e.g. you are implementing your own host library or direct interface, simply include the main header file:

#include <Dagu_ComMotionCommon.h>
