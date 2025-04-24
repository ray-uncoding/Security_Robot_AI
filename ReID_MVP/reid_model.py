import torch
import torchvision.transforms as T
import numpy as np
import cv2
from torchreid import models

class ReIDExtractor:
    def __init__(self, model_name='osnet_x0_25', device=None):
        if device is None:
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.device = device

        self.model = models.build_model(name=model_name, num_classes=1000)
        state_dict = torch.load("ReID_MVP/osnet_x0_25_imagenet.pth", map_location=device)
        self.model.load_state_dict(state_dict)
        self.model.to(device)
        self.model.eval()

        # 修正 BatchNorm2d 問題（推論時不追蹤 running stats）
        for m in self.model.modules():
            if isinstance(m, torch.nn.BatchNorm2d):
                m.track_running_stats = False

        self.transform = T.Compose([
            T.ToPILImage(),
            T.Resize((256, 128)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225])
        ])

    def extract(self, image):
        if image is None or image.size == 0:
            return None

        img_tensor = self.transform(image).unsqueeze(0).to(self.device)
        with torch.no_grad():
            features = self.model(img_tensor)

        return features[0].cpu().numpy()


if __name__ == '__main__':
    extractor = ReIDExtractor()
    img = cv2.imread('test_person.jpg')
    feat = extractor.extract(img)
    print('Feature shape:', feat.shape)  # (512,)