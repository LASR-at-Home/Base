import numpy as np 

def generate_lift_wait_box(corner_zero):
    x,y = corner_zero
    wait_box = np.zeros((4,2))
    print(wait_box)

    wait_box[0] = [x,y]
    wait_box[1] = [x-3.1,y]
    wait_box[2] = [x-3.1,y+2.5] 
    wait_box[3] = [x,y+2.5]

    return wait_box



wait_box = generate_lift_wait_box([3.9606051445,-0.823928892612]) 
print(wait_box)