import cv2 as cv
import h5py
import os
import glob
import numpy as np

from edgetpu.classification.engine import ClassificationEngine


class Embeddings(object):
    def __init__(self, facenet_path, im_path, embeddings_dic_path):
        self.im_path = im_path
        self.embeddings_dic_path = embeddings_dic_path
        self.facenet_engine = ClassificationEngine(facenet_path)

        self.build_embedding_dic()

    def get_embedding(self, face):
        face = self.resize_face(face)
        emb = self.facenet_engine.classify_with_input_tensor(face, top_k=130, threshold=-0.1)
        emb.sort(key=lambda emb: emb[0])
        return emb

    def get_id_from_embedding(self, emb_to_id, threshold):
        comp_arr = []
        dic = h5py.File(self.embeddings_dic_path, 'r')
        emb_dic, name_dic = dic['embedding'][:], dic['name'][:]

        if np.shape(emb_dic)[0] == 0:
            dic.close()
            return 'Unknown'

        for i in range(len(emb_dic)):
            comp_arr.append(np.mean(np.square(emb_to_id - emb_dic[i])))

        if min(comp_arr) < threshold:
            dic.close()
            return name_dic[np.argmin(comp_arr)].decode()
        dic.close()
        return 'Unknown'

    def add_someone(self, face):
        i = len([name for name in os.listdir(self.im_path)])
        cv.imwrite(self.im_path + '/person-' + str(i) + '.jpg', face)
        self.build_embedding_dic()

    def resize_face(self, face):
        face = cv.resize(face, (160, 160), interpolation=cv.INTER_LINEAR)
        return np.asarray(face).flatten()

    def build_embedding_dic(self):
        dic = h5py.File(self.embeddings_dic_path, 'w')
        name_arr = [name.split('.')[0].encode() for name in os.listdir(self.im_path)]
        emb_arr = [self.get_embedding(cv.imread(face)) for face in glob.glob(self.im_path + "/*.jpg")]

        dic.create_dataset('name', data=name_arr)
        dic.create_dataset('embedding', data=emb_arr)
        dic.close()
        return dic
