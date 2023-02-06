import picar_4wd as fc

speed = 30

def main():
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue
       
        if scan_list[3:7] != [2,2,2,2]:
            if sum(scan_list[:3]) >= sum(scan_list[7:]):
                fc.turn_left(speed)
            else:
                fc.turn_right(speed)
        else:
            fc.forward(speed)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
