#!/usr/bin/env python3

'''returns the overlapped area between two bounding boxes'''

def overlap_area(A, B):
    left = max(A['x1'], B['x1'])
    bottom = max(A['y2'], B['y2'])
    right = min(A['x2'], B['x2'])
    top = min(A['y1'], B['y1'])
    return (right - left) * (top - bottom)

'''
Returns the overlap percentage,
S is the smallest area
'''
def get_percentage_overlap(S, x, y, w, h):
    biggest_area = w * h
    o_area = overlap_area(S, get_corners(x, y, w, h))
    try:
        return (o_area/biggest_area)*100
    except ZeroDivisionError:
        return 0

# '''
# Returns the overlap percentage,
# S is the smallest area
# '''
# def get_percentage_overlap(S, B):
#     biggest_area = B[2] * B[3]
#     o_area = overlap_area(S, B)
#     return (o_area/biggest_area)*100

'''checks if two bounding boxes overlap'''
def is_overlapping(A, B):
    return (A['x1'] < B['x2'] and A['x2'] > B['x1'] and
            A['y1'] > B['y2'] and A['y2'] < B['y1'])

'''
returns the top right and bottom left corner
(x1, y1) X______________
        |             |
        |_____________X  (x2, y2)
'''
def get_corners(x, y, w, h):
    return {
        'x1': x,
        'y1': y + h,
        'x2': x + w,
        'y2': y
    }
