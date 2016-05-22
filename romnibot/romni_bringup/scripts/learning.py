import sklearn
from sklearn.datasets import load_sample_image
from sklearn.ensemble import RandomForestClassifier
import numpy as np
#import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy import misc
import warnings
from PIL import Image
import glob
import pickle

warnings.filterwarnings('ignore')
"""
 This method converts the input RGB image to Gray scale image
"""
def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])

"""
  This method loads the door images from the given path to be trained.
"""
def loadDoorImages(doorPath):
  for filename in glob.glob(doorPath):
    im=Image.open(filename)
    ar = np.array(im)
    gray = rgb2gray(ar)
    gray2D = gray.ravel()
    image_list.append(gray2D)
    class_list.append('door')


"""
  This method loads the non-door images from the given path to be trained.
"""
def loadNonDoorImages(nonDoorPath):
    for filename in glob.glob(nonDoorPath):
      im=Image.open(filename)
      ar = np.array(im)
      gray = rgb2gray(ar)
      gray2D = gray.ravel()
      image_list.append(gray2D)
      class_list.append('notdoor')

"""
  This method loads the trained classifier from given path.
"""
def loadClassifier(classifierPath):
  with open(classifierPath, 'rb') as f:
    clf = pickle.load(f)
    return clf;


"""
  This method saves the trained classifier into the given path.
"""
def saveClassifier(classifier, filename):
    with open(filename, 'wb') as output:
        pickle.dump(classifier, output, -1)

image_list = []
class_list = []
doorImagesPath = '/home/ck/Door/*.jpg';
notDoorImagesPath = '/home/ck/NotDoor/*.jpg';
classifierPath = 'classifier.pkl';
loadDoorImages(doorImagesPath);
loadNonDoorImages(notDoorImagesPath);

#classifier = loadClassifier(classifierPath);

classifier = RandomForestClassifier()

d1 = Image.open("/home/ck/predictionImage.JPG")
arD2 = np.array(d1)
gray = rgb2gray(arD2)
gray2D = gray.ravel()

classifier.fit(image_list, class_list)

#saveClassifier(classifier, classifierPath);

response = classifier.predict(gray2D)
prob = classifier.predict_proba(gray2D)
print(response)
print(prob)
print(set(class_list))
