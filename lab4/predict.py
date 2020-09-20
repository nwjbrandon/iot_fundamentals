from keras.models import load_model
import tensorflow as tf
from tensorflow.python.keras.backend import set_session

import numpy as np
from PIL import Image
from os import listdir
from os.path import join

MODEL_NAME = "flowers.hd5"

SAMPLE_PATH = "./samples"

dict={0: 'daisy', 1: 'dandelion', 2: 'roses', 3: 'sunflowers', 4: 'tulips'}

session = tf.compat.v1.Session(graph=tf.compat.v1.Graph())

def classify(model, image):
    with session.graph.as_default():
        set_session(session)
        result = model.predict(image)
        themax = np.argmax(result)
    return (dict[themax], result[0][themax], themax)

def load_image(image_fname):
    img = Image.open(image_fname)
    img = img.resize((249, 249))
    imgarray = np.array(img)/255.0
    final = np.expand_dims(imgarray, axis=0)
    return final

def main():
    with session.graph.as_default():
        set_session(session)
        model = load_model(MODEL_NAME)
        sample_files = listdir(SAMPLE_PATH)

        for filename in sample_files:
            filename = join(SAMPLE_PATH, filename)
            img = load_image(filename)
            label, prob, _ = classify(model, img)
            print("Prob: ", prob)
            print("File: ", filename)
            print("Label: ", label)

if __name__ == "__main__":
    main()