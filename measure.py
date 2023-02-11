from time import sleep
import picar_4wd as fc

SPEED = 10
print(f'Speed = 10')


for i in range(10):
    dist = fc.get_distance_at(0)
    fc.forward(SPEED)
    sleep(0.1)
    fc.stop()
    print(f'i = {i} dist = {dist}')


fc.turn_left(SPEED)
sleep(20)
fc.stop()

