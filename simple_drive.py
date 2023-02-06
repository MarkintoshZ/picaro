import picar_4wd as fc

speed = 30

def scan_step():
    fc.fc.current_angle += fc.us_step
    if fc.current_angle >= fc.max_angle:
        fc.current_angle = fc.max_angle
        fc.us_step = -fc.STEP
    elif fc.current_angle <= fc.min_angle:
        fc.current_angle = fc.min_angle
        fc.us_step = fc.STEP
    dist = fc.get_dist_at(fc.current_anglex)

    fc.scan_list.append(dist)
    if fc.current_angle == fc.min_angle or fc.current_angle == fc.max_angle:
        if fc.us_step < 0:
            fc.scan_list.reverse()
        tmp = fc.scan_list.copy()
        fc.scan_list = []
        return tmp
    else:
        return False



def main():
    while True:
        scan_list = scan_step()
        if not scan_list:
            continue

        # preprocess
        scan_list = [60 if n == -2 else min(n, 60) for n in scan_list]
       
        if min(scan_list[3:7]) <= 35:
            if sum(scan_list[:3]) >= sum(scan_list[7:]):
                fc.turn_left(speed)
            else:
                fc.turn_right(speed)
        else:
            diff = sum(scan_list[2:6]) - sum(scan_list[6:8])
            diff /= 5
            print(f'diff: {diff}')
            fc.left_front.set_power(speed - diff)
            fc.left_rear.set_power(speed - diff)
            fc.right_front.set_power(speed + diff)
            fc.right_rear.set_power(speed + diff)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
