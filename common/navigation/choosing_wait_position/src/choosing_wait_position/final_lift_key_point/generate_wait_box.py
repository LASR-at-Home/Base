import numpy as np 


WAITING_AREA_LONG_SIDE = 1.2
WAITING_AREA_SHORT_SIDE = 0.6


def generate_lift_wait_box(corner_zero,area):
    x,y = corner_zero
    #This is the corner that if you were facing the lift, would be the left corner of the lift. 


    wait_box = np.zeros((4,2))
    print(wait_box)
    see_wait_area_pos = [0,0]

    if (area=='606_lift'):
        wait_box[0] = [x,y]
        wait_box[1] = [x-3.5,y]
        wait_box[2] = [x-3.5,y+2.7] 
        wait_box[3] = [x,y+2.5]

        see_wait_area_pos = [x,y+2.5]
    elif(area=='6th floor lift'):
        wait_box[0] = [x,y]
        wait_box[1] = [x-3.5,y]
        wait_box[2] = [x-3.5,y+2.7] 
        wait_box[3] = [x,y+2.5]


   
    return wait_box, see_wait_area_pos





corner = [4.10421085358,-0.823522567749]


wait_box,see_wait_area_pos = generate_lift_wait_box(corner,'606_lift')
print(wait_box)
print(see_wait_area_pos)
