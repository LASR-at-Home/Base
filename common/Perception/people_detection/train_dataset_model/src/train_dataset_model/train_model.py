from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
import pickle
import os
import rospkg

# Detect faces, extract embeddings, and fit our SVM model to the embeddings data.

# construct the argument parser and parse the arguments
# load the face embeddings
def train_model():
    # self.path = os.path.join(rospkg.RosPack().get_path('people_detection'), 'dataset',
    #                          self.name)
    path = os.path.join(rospkg.RosPack().get_path('face_detection'), 'output', 'embeddings.pickle')
    path_output = os.path.join(rospkg.RosPack().get_path('face_detection'), 'output')

    print("[INFO] loading face embeddings...")
    print(open(path, "rb").read())
    data = pickle.loads(open(path, "rb").read())
    # encode the labels
    print("[INFO] encoding labels...")
    le = LabelEncoder()
    labels = le.fit_transform(data["names"])

    # train the model used to accept the 128-d embeddings of the face and
    # then produce the actual face recognition
    print("[INFO] training model...")
    recognizer = SVC(C=1.0, kernel="linear", probability=True)
    recognizer.fit(data["embeddings"], labels)

    # write the actual face recognition model to disk
    f = open(os.path.join(path_output, "recognizer.pickle"), "wb")
    f.write(pickle.dumps(recognizer))
    f.close()
    # write the label encoder to disk
    f = open(os.path.join(path_output, "le.pickle"), "wb")
    f.write(pickle.dumps(le))
    f.close()

