#!/usr/bin/env python3
from .bbox_helpers import get_percentage_overlap, get_corners

SOFA = [0, 0, 100, 50]

P1 = [55, 2, 30, 50]
P2 = [2, 3, 40, 50]
P3 = [30, 1, 25, 50]

PEOPLE = [P1, P2]

'''splits the sofa into amount_of_pieces'''
def split_sofa(sofa, amount_of_pieces):
    pieces = []
    piece_length = int(sofa[2] / amount_of_pieces)
    for i in range(amount_of_pieces):
        pieces.append([sofa[0] + piece_length * i, sofa[1], piece_length, sofa[3]])
    return pieces


'''Gets the person overlapping of the couch'''
def person_overlap(person, sofa_chunch):
    p = get_corners(person[0], person[1], person[2], person[3])
    return get_percentage_overlap(p, sofa_chunch[0], sofa_chunch[1], sofa_chunch[2], sofa_chunch[3])


'''
returns the mostly overlapped sofa chunk starting from index 1
------------------------
  1    |   2    |  3
------------------------
'''
def get_sofa_chunk(sofa, person, amount_of_pieces):
    sofa_chuncks = split_sofa(sofa, amount_of_pieces)
    print("sofa_chuncks: ", sofa_chuncks)
    index = None
    max_overlap = 0
    for i in range(len(sofa_chuncks)):
        overlap = person_overlap(person, sofa_chuncks[i])
        if overlap > max_overlap:
            max_overlap = overlap
            index = i + 1
    return index


'''gets the first availible sofa chunch starting from 1'''
def get_available_sofa_chunk(sofa, people, amount_of_pieces):
    sofa_availability = [True for i in range(amount_of_pieces)]
    for person in people:
        index = get_sofa_chunk(sofa, person, amount_of_pieces)
        print('sofa chunk taken: ', index)
        if index:
            sofa_availability[index - 1] = False

    for i in range(len(sofa_availability)):
        if sofa_availability[i]:
            return i + 1
    return None


'''Gets the centroid of the sofa'''
def get_centroid_of_sofa_chunk(sofa, people, amount_of_pieces):
    available_chunk = get_available_sofa_chunk(sofa, people, amount_of_pieces)
    if available_chunk:
        piece_length = int(sofa[2] / amount_of_pieces)
        sofa_chunk = [sofa[0] + piece_length * (available_chunk - 1), sofa[1], piece_length, sofa[3]]
        return [sofa_chunk[0] + sofa[2] / 2, sofa_chunk[1] + sofa_chunk[3] / 2]
    else:
        return None

#! just for testing
if __name__ == '__main__':
    print("percentage overlap on sofa: ", get_available_sofa_chunk(SOFA, PEOPLE, 3))
    print("get centroid: ", get_centroid_of_sofa_chunk(SOFA, PEOPLE, 3))
