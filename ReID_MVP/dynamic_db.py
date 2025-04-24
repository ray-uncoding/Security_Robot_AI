import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
import uuid

class DynamicFeatureDB:
    def __init__(self, max_features_per_id=10, similarity_threshold=0.6):
        self.db = {}  # {id: [feature_vector, feature_vector, ...]}
        self.max_features = max_features_per_id
        self.similarity_threshold = similarity_threshold

    def match(self, feature):
        best_id = None
        best_sim = 0.0

        for person_id, features in self.db.items():
            sims = cosine_similarity([feature], features)
            max_sim = np.max(sims)

            if max_sim > best_sim and max_sim >= self.similarity_threshold:
                best_sim = max_sim
                best_id = person_id

        return best_id, best_sim

    def update(self, person_id, new_feature):
        if person_id not in self.db:
            self.db[person_id] = [new_feature]
        else:
            self.db[person_id].append(new_feature)
            if len(self.db[person_id]) > self.max_features:
                self.db[person_id].pop(0)  # Remove oldest

    def register(self, new_feature):
        new_id = str(uuid.uuid4())[:8]
        self.db[new_id] = [new_feature]
        return new_id

    def match_or_register(self, feature):
        matched_id, score = self.match(feature)
        if matched_id:
            self.update(matched_id, feature)
            return matched_id, score, False  # existing ID
        else:
            new_id = self.register(feature)
            return new_id, 0.0, True  # new ID


if __name__ == "__main__":
    db = DynamicFeatureDB()
    dummy_vec = np.random.rand(512)
    person_id, score, is_new = db.match_or_register(dummy_vec)
    print("ID:", person_id, "(new:" , is_new, ")")
