#!/usr/bin/env python3
"""
簡化版 OCR - 使用 OBS 串流捕獲，優化性能
避免頻繁 resize 和多餘的 copy 操作
"""

import os
import sys
import time
import cv2
import json
import logging
import numpy as np
from pydantic import BaseModel
import google.generativeai as genai

# 添加專案路徑
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Insta_OpenCV.controller.insta_worker import InstaWorker
from obs_stream_capture import OBSStreamCapture

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 定義台灣身份證 Schema
class TaiwanIDCard(BaseModel):
    id_number: str
    name: str

def setup_gemini_api():
    """設置 Gemini API"""
    api_key = os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE")
    
    if api_key == "YOUR_API_KEY_HERE":
        print("請設置 GEMINI_API_KEY 環境變數")
        api_key = input("請輸入你的 Gemini API Key: ").strip()
        if not api_key:
            logger.error("未提供 API Key")
            return None
    
    try:
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-1.5-pro')
        logger.info("Gemini API 初始化成功")
        return model
    except Exception as e:
        logger.error(f"Gemini API 初始化失敗: {e}")
        return None

class SimpleOCRCapture:
    """簡化版 OCR 捕獲 - 性能優化"""
    
    def __init__(self):
        # 配置
        self.insta_ip = "192.168.1.188"
        self.rtmp_url = "rtmp://192.168.1.188:1935/live/preview"
        
        # OBS 組件
        self.insta_worker = None
        self.obs_video_capture = None
        
        # Gemini API
        self.model = setup_gemini_api()
        
        # 影像相關
        self.original_frame = None
        self.display_frame = None
        
        # ROI 相關
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        
        # 性能優化：固定顯示尺寸，避免重複 resize
        self.DISPLAY_WIDTH = 1280
        self.DISPLAY_HEIGHT = 640
        
        # 統計
        self.frame_count = 0
        self.fps = 0.0
        self.start_time = time.time()
        
        # 許可人員白名單
        self.whitelist = [
            {"id_number": "A123456789", "name": "王小明"},
            {"id_number": "B987654321", "name": "李小華"},
        ]
        
        self.running = False
        
    def start_system(self):
        """啟動系統"""
        try:
            logger.info("=== 啟動簡化版 OCR 系統 ===")
            
            # 1. 啟動 Insta Worker
            logger.info("1. 啟動 Insta Worker...")
            self.insta_worker = InstaWorker(ip_address=self.insta_ip)
            ready_event = self.insta_worker.start_preview_all()
            
            if not ready_event.wait(timeout=30):
                logger.error("Insta Worker 啟動超時")
                return False
            
            logger.info("   ✓ Insta Worker 已就緒")
            time.sleep(5)
            
            # 2. 啟動 OBS 視頻捕獲
            logger.info("2. 啟動 OBS 視頻捕獲...")
            self.obs_video_capture = OBSStreamCapture(
                stream_url=self.rtmp_url,
                capture_audio=False
            )
            
            if not self.obs_video_capture.start():
                logger.error("OBS 視頻捕獲啟動失敗")
                return False
            
            logger.info("   ✓ OBS 視頻捕獲已啟動")
            
            # 3. 創建顯示窗口
            cv2.namedWindow('Simple OCR', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Simple OCR', self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT)
            cv2.setMouseCallback('Simple OCR', self.mouse_callback)
            
            # 預分配 display_frame 以避免重複分配記憶體
            self.display_frame = np.zeros((self.DISPLAY_HEIGHT, self.DISPLAY_WIDTH, 3), dtype=np.uint8)
            
            logger.info("=== 簡化版 OCR 系統已就緒 ===")
            logger.info("操作說明:")
            logger.info("  滑鼠拖拽: 選擇 ROI 區域")
            logger.info("  'c': 清除 ROI")
            logger.info("  'o': 執行 OCR")
            logger.info("  's': 保存原始畫面")
            logger.info("  'q': 退出")
            
            self.running = True
            return True
            
        except Exception as e:
            logger.error(f"系統啟動失敗: {e}")
            return False
    
    def mouse_callback(self, event, x, y, flags, param):
        """滑鼠回調函數"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi_start = (x, y)
            self.roi_selecting = True
            self.roi_selected = False
            
        elif event == cv2.EVENT_MOUSEMOVE and self.roi_selecting:
            self.roi_end = (x, y)
            
        elif event == cv2.EVENT_LBUTTONUP and self.roi_selecting:
            self.roi_end = (x, y)
            self.roi_selecting = False
            self.roi_selected = True
            logger.info(f"ROI 選擇: ({self.roi_start[0]}, {self.roi_start[1]}) 到 ({x}, {y})")
    
    def convert_display_to_original_coords(self, display_coords):
        """將顯示坐標轉換為原始圖像坐標"""
        if self.original_frame is None:
            return None
        
        display_x, display_y = display_coords
        orig_h, orig_w = self.original_frame.shape[:2]
        
        # 計算縮放比例
        scale_x = orig_w / self.DISPLAY_WIDTH
        scale_y = orig_h / self.DISPLAY_HEIGHT
        
        # 轉換坐標
        orig_x = int(display_x * scale_x)
        orig_y = int(display_y * scale_y)
        
        # 確保坐標在有效範圍內
        orig_x = max(0, min(orig_x, orig_w - 1))
        orig_y = max(0, min(orig_y, orig_h - 1))
        
        return (orig_x, orig_y)
    
    def clear_roi(self):
        """清除 ROI 選擇"""
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        logger.info("ROI 選擇已清除")
    
    def capture_and_ocr(self):
        """拍照並執行 OCR"""
        if self.original_frame is None or not self.model:
            logger.warning("沒有可用影像或 Gemini API 未初始化")
            return
        
        try:
            # 決定處理區域
            if self.roi_selected and self.roi_start and self.roi_end:
                # 使用 ROI 區域
                orig_start = self.convert_display_to_original_coords(self.roi_start)
                orig_end = self.convert_display_to_original_coords(self.roi_end)
                
                if orig_start and orig_end:
                    x1 = min(orig_start[0], orig_end[0])
                    y1 = min(orig_start[1], orig_end[1])
                    x2 = max(orig_start[0], orig_end[0])
                    y2 = max(orig_start[1], orig_end[1])
                    
                    processed_frame = self.original_frame[y1:y2, x1:x2]
                    logger.info(f"使用 ROI 進行 OCR: {processed_frame.shape}")
                else:
                    logger.error("ROI 坐標轉換失敗")
                    return
            else:
                # 使用原始全畫面
                processed_frame = self.original_frame
                logger.info("使用全畫面進行 OCR")
            
            if processed_frame is None or processed_frame.size == 0:
                logger.error("處理的影像無效")
                return
            
            # 保存暫存圖片
            img_path = 'temp_ocr.jpg'
            cv2.imwrite(img_path, processed_frame)
            
            # 執行 OCR
            logger.info("正在執行 OCR...")
            with open(img_path, 'rb') as f:
                image_bytes = f.read()
            
            img_part = {
                "mime_type": "image/jpeg",
                "data": image_bytes
            }
            
            resp = self.model.generate_content(
                [img_part, "這是一張台灣身份證照片，請回傳身分證號(id_number)與姓名(name)。"],
                generation_config=genai.GenerationConfig(
                    response_mime_type="application/json",
                    response_schema=TaiwanIDCard
                )
            )
            
            # 處理回應
            ocr_result = None
            if resp.candidates and resp.candidates[0].content and resp.candidates[0].content.parts:
                part = resp.candidates[0].content.parts[0]
                
                if part.function_call:
                    ocr_result = dict(part.function_call.args)
                elif resp.text:
                    try:
                        ocr_result = json.loads(resp.text)
                    except json.JSONDecodeError:
                        logger.error("無法解析 OCR 回應")
                        return
            
            if ocr_result:
                # 許可判斷
                permitted = self.is_permitted(ocr_result)
                status = "✅ 許可人員" if permitted else "❌ 異常人員"
                
                result_text = f"""
OCR 辨識結果:
身分證號: {ocr_result.get('id_number', 'N/A')}
姓名: {ocr_result.get('name', 'N/A')}
身份判斷: {status}
                """
                
                logger.info(result_text)
                print(result_text)
            else:
                logger.error("OCR 辨識失敗")
            
            # 清理暫存檔案
            try:
                os.remove(img_path)
            except:
                pass
                
        except Exception as e:
            logger.error(f"OCR 處理錯誤: {e}")
    
    def is_permitted(self, ocr_result):
        """檢查是否為許可人員"""
        for person in self.whitelist:
            if (ocr_result.get("id_number") == person["id_number"] and
                ocr_result.get("name") == person["name"]):
                return True
        return False
    
    def run(self):
        """主運行循環 - 性能優化版本"""
        if not self.start_system():
            return
        
        try:
            while self.running:
                # 獲取最新幀
                frame = self.obs_video_capture.get_latest_frame()
                
                if frame is not None:
                    self.frame_count += 1
                    
                    # 保存原始畫面
                    self.original_frame = frame
                    
                    # 直接 resize 到預分配的 display_frame，避免記憶體重新分配
                    cv2.resize(frame, (self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT), 
                              dst=self.display_frame, interpolation=cv2.INTER_LINEAR)
                    
                    # 計算 FPS
                    current_time = time.time()
                    elapsed = current_time - self.start_time
                    if elapsed > 1.0:
                        self.fps = self.frame_count / elapsed
                        self.frame_count = 0
                        self.start_time = current_time
                    
                    # 直接在 display_frame 上繪製 ROI 和文字（避免 copy）
                    if self.roi_selected and self.roi_start and self.roi_end:
                        cv2.rectangle(self.display_frame, self.roi_start, self.roi_end, (0, 255, 0), 2)
                        cv2.putText(self.display_frame, "OCR Region", 
                                   (self.roi_start[0], self.roi_start[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    elif self.roi_selecting and self.roi_start and self.roi_end:
                        cv2.rectangle(self.display_frame, self.roi_start, self.roi_end, (255, 255, 0), 2)
                    
                    # 添加 FPS 信息
                    cv2.putText(self.display_frame, f"FPS: {self.fps:.1f}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 顯示原始分辨率
                    orig_h, orig_w = frame.shape[:2]
                    cv2.putText(self.display_frame, f"Original: {orig_w}x{orig_h}", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    cv2.imshow('Simple OCR', self.display_frame)
                
                # 處理按鍵
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    self.clear_roi()
                elif key == ord('o'):
                    self.capture_and_ocr()
                elif key == ord('s') and self.original_frame is not None:
                    timestamp = int(time.time())
                    filename = f"simple_ocr_frame_{timestamp}.jpg"
                    cv2.imwrite(filename, self.original_frame)
                    logger.info(f"已保存原始畫面: {filename}")
                
        except KeyboardInterrupt:
            logger.info("收到中斷信號")
        
        finally:
            self.stop_system()
    
    def stop_system(self):
        """停止系統"""
        logger.info("正在停止簡化版 OCR 系統...")
        
        self.running = False
        cv2.destroyAllWindows()
        
        if self.obs_video_capture:
            self.obs_video_capture.stop()
        
        if self.insta_worker:
            self.insta_worker.stop_all()
        
        logger.info("✓ 簡化版 OCR 系統已停止")

def main():
    """主函數"""
    try:
        ocr_capture = SimpleOCRCapture()
        ocr_capture.run()
    except Exception as e:
        logger.error(f"程序執行錯誤: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
