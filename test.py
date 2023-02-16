from thorpy.comm.discovery import discover_stages
from thorpy.comm.from_port import from_port


if __name__ == '__main__':
    from thorpy.message import *
    
    #stages = list(discover_stages())
    #print(stages)
    #s = stages[0]
    
    s = from_port("/dev/ttyUSB0")

    #s.home()
    
    s.print_state()

    #quit()
    s.home(block=True)

    s.print_state()

    #quit()
    import IPython
    IPython.embed()
    
    quit()
    from time import sleep
    zzz=0
    while True:
        print(zzz)
        s.print_state()
        zzz+=1
        sleep(1)
        if zzz == 8:
            break

    s.print_state()
    s.move_jog(1)
    sleep(3)
    s.print_state()
    input()
    s.move_abs(20)

    while True:
        print(zzz)
        s.print_state()
        zzz+=1
        sleep(1)
        if zzz == 8:
            break

    del s, stages
