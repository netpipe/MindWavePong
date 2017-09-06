import mindwave, time

headset = mindwave.Headset('/dev/ttyUSB1', '6F0A')
time.sleep(2)

headset.connect()
print "Connecting..."

while headset.status != 'connected':
    time.sleep(0.5)
    if headset.status == 'standby':
        headset.connect()
        print "Retrying connect..."
print "Connected."

while True:
    time.sleep(.5)
    print "Attention: %s, Meditation: %s, theta: %s,delta: %s,highalpha: %s,gamma: %s" % (headset.attention, headset.meditation,headset.theta,headset.delta,headset.highalpha,headset.lowgamma)

