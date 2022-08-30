from thorpy.comm.discovery import discover_stages

if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    s = stages[0]
    
    #s.home()
    
    #s.print_state()
    
    #import IPython
    #IPython.embed()
    from time import sleep
    zzz=0
    while True:
        print(zzz)
        zzz+=1
        sleep(1)
        if zzz == 20:
            break

    s.print_state()
    #s.home()
    input()
    del s, stages
