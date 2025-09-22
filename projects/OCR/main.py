# --- Qt5 + OpenCV + Gemini OCR ---
import sys
import os
import cv2
import json
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QTextEdit
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from pydantic import BaseModel
import google.generativeai as genai


# 定義台灣身份證 Schema
class TaiwanIDCard(BaseModel):
    id_number: str
    name: str

# Gemini client 初始化，需傳入 api_key
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE"))
model = genai.GenerativeModel('gemini-1.5-pro')

class CameraWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Gemini OCR 身份證辨識')
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.capture_btn = QPushButton('拍照並辨識')
        self.capture_btn.clicked.connect(self.capture_and_ocr)
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.capture_btn)
        layout.addWidget(self.result_text)
        self.setLayout(layout)

        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)
        self.current_frame = None

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            self.current_frame = frame
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def capture_and_ocr(self):
        if self.current_frame is not None:
            # 儲存暫存圖片
            img_path = 'temp_capture.jpg'
            cv2.imwrite(img_path, self.current_frame)
            with open(img_path, 'rb') as f:
                image_bytes = f.read()
            # 呼叫 Gemini OCR
            try:
                # Create the image part
                img_part = {
                    "mime_type": "image/jpeg",
                    "data": image_bytes
                }
                # Make the API call
                resp = model.generate_content(
                    [img_part, "這是一張台灣身份證照片，請回傳身分證號(id_number)與姓名(name)。"],
                    generation_config=genai.GenerationConfig(
                        response_mime_type="application/json",
                        response_schema=TaiwanIDCard
                    )
                )

                # The model should return a function call, but if not, try to parse the text.
                if resp.candidates and resp.candidates[0].content and resp.candidates[0].content.parts:
                    part = resp.candidates[0].content.parts[0]
                    if part.function_call:
                        parsed_args = dict(part.function_call.args)
                        self.result_text.setText(json.dumps(parsed_args, ensure_ascii=False, indent=2))
                        self.result_text.append(f"\n---\nPython物件：\n身分證號: {parsed_args.get('id_number', 'N/A')}\n姓名: {parsed_args.get('name', 'N/A')}")
                    elif resp.text:
                        try:
                            # The model might just return a json string in text
                            parsed_args = json.loads(resp.text)
                            self.result_text.setText(resp.text)
                            self.result_text.append(f"\n---\nPython物件：\n身分證號: {parsed_args.get('id_number', 'N/A')}\n姓名: {parsed_args.get('name', 'N/A')}")
                        except json.JSONDecodeError:
                             self.result_text.setText(f"無法解析OCR回應。\n\n原始回應:\n{resp.text}")
                    else:
                        self.result_text.setText("無法解析OCR回應，且沒有收到文字回應。")
                else:
                    # Handle cases where the response is not as expected
                    error_message = "無法解析OCR回應。請再試一次。"
                    if resp.prompt_feedback and resp.prompt_feedback.block_reason:
                        error_message += f"\n原因: {resp.prompt_feedback.block_reason.name}"
                    
                    if resp.text:
                        error_message += f"\n\n原始回應:\n{resp.text}"

                    self.result_text.setText(error_message)
                    
            except Exception as e:
                self.result_text.setText(f"OCR 發生錯誤：{e}")
            finally:
                os.remove(img_path)

    def closeEvent(self, event):
        self.cap.release()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CameraWidget()
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec_())