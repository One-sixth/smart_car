python3

import RPi.GPIO as io
io.setmode(io.BOARD)
io.setup(11, 0)
io.output(11, 1)
io.setup(13, 0)
io.output(13, 1)
