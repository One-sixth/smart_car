import RPi.GPIO as io

io.setmode(io.BOARD)

for i in range(40):
    try:
        io.setup(i, io.OUT)
        io.output(i, 0)
    except:
        pass

io.cleanup()
