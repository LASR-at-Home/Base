
#!/usr/bin/env python3
'''  A class that stores the information of a person image after a person was detected '''

class PersonImageInfo():

    def __init__(self):
        self.img = None
        self.keypoints = None
        self.face_bb = None

    def create_torso_bb(self):
        try:
            right_shoulder = self.keypoints["R-Sho"]
            left_shoulder = self.keypoints["L-Sho"]
            right_hip = self.keypoints["R-Hip"]
            left_hip = self.keypoints["L-Hip"]

            if left_shoulder and right_hip:
                x1, y2, _, _ = right_hip[0]
                x2, y1, _, _ = left_shoulder[0]
            elif right_shoulder and left_hip:
                y2 = left_hip[0][1]
                y1 = right_shoulder[0][1]
                x1 = right_shoulder[0][0]
                x2 = left_hip[0][0]
            elif right_shoulder and left_shoulder:
                x1, y2, _, _  = right_shoulder[0]
                x2 = left_shoulder[0][0]
                y1 = self.img.shape[0]
            elif right_shoulder:
                x1, y2, _, _ = right_shoulder[0]
                x2 = x1 + (self.img.shape[1] // 3)
                y1 = self.img.shape[0]
            elif left_shoulder:
                x2, y1, _, _ = left_shoulder[0]
                x1 = max(0, x2 - (self.img.shape[1] // 2))
                y2 = self.img.shape[0]
            elif right_hip and left_hip:
                x1, y1, _, _ = right_hip[0]
                x2 = left_hip[0][0]
                y2 = max(0, y1 - (self.img.shape[0] // 2))
            elif right_hip:
                x1, y2, _, _ = right_hip[0]
                x2 = x1 + (self.img.shape[1] // 3)
                y1 = self.img.shape[0] // 2
            elif left_hip:
                x2, y1, _, _ = left_hip[0]
                x1 = max(0, x2 - (self.img.shape[1] // 3))
                y2 = self.img.shape[0] // 2
            else:
                x1 = y1 = 0
                y2, x2 = self.img.shape[:2]
            
            return min(y1, y2), min(x1, x2), max(y1, y2), max(x1, x2)
        except Exception:
            return None

    def torso(self):
        y1, x1, y2, x2 = self.create_torso_bb()
        return self.img[y1:y2, x1:x2]

    def face(self):

        x1, y1, x2, y2 = self.face_bb
        return self.img[y1:y2, x1:x2]
    
    def face_dims(self):

        return (
            abs(self.face_bb[2] - self.face_bb[0]),
            abs(self.face_bb[3] - self.face_bb[1])
        )