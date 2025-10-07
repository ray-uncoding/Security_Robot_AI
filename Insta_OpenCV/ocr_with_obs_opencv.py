#!/usr/bin/env python3
"""
Gemini OCR 身份證辨識 - 使用 OBS 串流捕獲和 OpenCV 顯示
Modified to use OBS stream capture with OpenCV display for better performance
"""

import os
import sys
import time
import cv2
import json
import logging
import threading
import numpy as np
from pydantic import BaseModel
import google.generativeai as genai

# 添加專案路徑
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.services.image_processing import ImageProcessingService
from Insta_OpenCV.services.system_monitor import get_global_system_monitor
from obs_stream_capture import OBSStreamCapture
from obs_audio import OBSAudioCapture

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 定義台灣身份證 Schema
class TaiwanIDCard(BaseModel):
    id_number: str
    name: str

# Gemini API 設置
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

class OCRCapture:
    """OCR 捕獲類別 - 使用 OpenCV 顯示"""
    
    def __init__(self):
        # 配置
        self.insta_ip = "192.168.1.188"
        self.rtmp_url = "rtmp://192.168.1.188:1935/live/preview"
        
        # 系統組件
        self.image_service = ImageProcessingService()
        self.system_monitor = get_global_system_monitor()
        self.system_monitor.start_monitoring()
        
        # OBS 組件
        self.insta_worker = None
        self.obs_video_capture = None
        self.obs_audio_capture = None
        
        # Gemini API
        self.model = setup_gemini_api()
        if not self.model:
            logger.error("Gemini API 初始化失敗，OCR 功能將無法使用")
        
        # 影像相關
        self.current_frame = None
        self.original_frame = None  # 保存原始未 resize 的畫面
        self.display_frame = None
        
        # ROI 相關
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        
        # 顯示相關
        self.display_scale = 0.33  # 顯示縮放比例（從 3840 -> 1280）
        self.frame_count = 0
        self.fps = 0.0
        self.start_time = time.time()
        
        # 預計算顯示尺寸以避免重複計算
        self.display_width = None
        self.display_height = None
        
        # 許可人員白名單
        self.whitelist = [
            {"id_number": "A123456789", "name": "王小明"},
            {"id_number": "B987654321", "name": "李小華"},
        ]
        
        # 狀態
        self.running = False
        
    def start_system(self):
        """啟動系統"""
        try:
            logger.info("=== 啟動 OCR 系統 ===")
            
            # 1. 啟動 Insta Worker
            logger.info("1. 啟動 Insta Worker...")
            self.insta_worker = InstaWorker(ip_address=self.insta_ip)
            ready_event = self.insta_worker.start_preview_all()
            
            if not ready_event.wait(timeout=30):
                logger.error("Insta Worker 啟動超時")
                return False
            
            logger.info("   ✓ Insta Worker 已就緒")
            time.sleep(5)  # 等待串流穩定
            
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
            
            # 3. 啟動 OBS 音頻捕獲
            logger.info("3. 啟動 OBS 音頻捕獲...")
            try:
                self.obs_audio_capture = OBSAudioCapture(stream_url=self.rtmp_url)
                if self.obs_audio_capture.start_capture():
                    logger.info("   ✓ OBS 音頻捕獲已啟動")
                else:
                    logger.warning("   ⚠ OBS 音頻捕獲啟動失敗")
                    self.obs_audio_capture = None
            except Exception as e:
                logger.warning(f"音頻捕獲初始化失敗: {e}")
                self.obs_audio_capture = None
            
            # 4. 創建顯示窗口
            cv2.namedWindow('OCR Camera', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('OCR Camera', 1280, 720)
            cv2.setMouseCallback('OCR Camera', self.mouse_callback)
            
            # 創建 ROI 預覽窗口
            cv2.namedWindow('ROI Preview', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ROI Preview', 400, 300)
            
            logger.info("=== OCR 系統已就緒 ===")
            logger.info("操作說明:")
            logger.info("  滑鼠拖拽: 選擇 ROI 區域")
            logger.info("  'c': 清除 ROI 選擇")
            logger.info("  'o': 執行 OCR 辨識")
            logger.info("  's': 保存當前幀")
            logger.info("  '+/-': 調整顯示大小")
            logger.info("  'q': 退出")
            
            self.running = True
            return True
            
        except Exception as e:
            logger.error(f"系統啟動失敗: {e}")
            return False
    
    def mouse_callback(self, event, x, y, flags, param):
        """滑鼠回調函數"""
        if self.current_frame is None:
            return
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # 開始選擇 ROI
            self.roi_start = (x, y)
            self.roi_selecting = True
            self.roi_selected = False
            logger.info(f"開始選擇 ROI: ({x}, {y})")
            
        elif event == cv2.EVENT_MOUSEMOVE and self.roi_selecting:
            # 更新 ROI 區域
            self.roi_end = (x, y)
            
        elif event == cv2.EVENT_LBUTTONUP and self.roi_selecting:
            # 完成 ROI 選擇
            self.roi_end = (x, y)
            self.roi_selecting = False
            self.roi_selected = True
            logger.info(f"ROI 選擇完成: ({self.roi_start[0]}, {self.roi_start[1]}) 到 ({x}, {y})")
            self.update_roi_preview()
    
    def convert_display_to_original_coords(self, display_coords):
        """將顯示坐標轉換為原始圖像坐標"""
        if self.original_frame is None:
            return None
        
        display_x, display_y = display_coords
        orig_h, orig_w = self.original_frame.shape[:2]
        disp_h, disp_w = self.display_frame.shape[:2]
        
        # 計算縮放比例
        scale_x = orig_w / disp_w
        scale_y = orig_h / disp_h
        
        # 轉換坐標
        orig_x = int(display_x * scale_x)
        orig_y = int(display_y * scale_y)
        
        # 確保坐標在有效範圍內
        orig_x = max(0, min(orig_x, orig_w - 1))
        orig_y = max(0, min(orig_y, orig_h - 1))
        
        return (orig_x, orig_y)
    
    def update_roi_preview(self):
        """更新 ROI 預覽 - 使用原始未 resize 的畫面"""
        if not (self.roi_selected and self.roi_start and self.roi_end and self.original_frame is not None):
            return
        
        try:
            # 轉換顯示坐標到原始圖像坐標
            orig_start = self.convert_display_to_original_coords(self.roi_start)
            orig_end = self.convert_display_to_original_coords(self.roi_end)
            
            if not (orig_start and orig_end):
                return
            
            # 確保坐標順序正確
            x1 = min(orig_start[0], orig_end[0])
            y1 = min(orig_start[1], orig_end[1])
            x2 = max(orig_start[0], orig_end[0])
            y2 = max(orig_start[1], orig_end[1])
            
            # 從原始畫面提取 ROI（未經 resize）
            roi_frame = self.original_frame[y1:y2, x1:x2]
            
            if roi_frame.size > 0:
                # 為了顯示，將 ROI 縮放到合適大小
                roi_h, roi_w = roi_frame.shape[:2]
                max_size = 400
                
                if roi_w > max_size or roi_h > max_size:
                    scale = max_size / max(roi_w, roi_h)
                    new_w = int(roi_w * scale)
                    new_h = int(roi_h * scale)
                    roi_display = cv2.resize(roi_frame, (new_w, new_h))
                else:
                    roi_display = roi_frame.copy()
                
                # 添加信息文字
                info_text = f"Original ROI: {roi_w}x{roi_h}"
                cv2.putText(roi_display, info_text, (10, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cv2.imshow('ROI Preview', roi_display)
                logger.info(f"ROI 預覽已更新: 原始尺寸 {roi_w}x{roi_h}")
            
        except Exception as e:
            logger.error(f"更新 ROI 預覽失敗: {e}")
    
    def clear_roi(self):
        """清除 ROI 選擇"""
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        
        # 清空 ROI 預覽窗口
        empty_frame = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(empty_frame, "No ROI selected", (100, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow('ROI Preview', empty_frame)
        
        logger.info("ROI 選擇已清除")
    
    def capture_and_ocr(self):
        """拍照並執行 OCR"""
        if self.current_frame is None:
            logger.warning("沒有可用的影像")
            return
        
        if not self.model:
            logger.error("Gemini API 未初始化")
            return
        
        try:
            # 決定處理區域
            if self.roi_selected and self.roi_start and self.roi_end:
                # 使用 ROI 區域（從原始畫面提取）
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
            
            # 品質增強
            processed_frame = self.image_service.enhance_frame_quality(processed_frame)
            
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
        """主運行循環"""
        if not self.start_system():
            return
        
        try:
            while self.running:
                # 獲取最新幀
                frame = self.obs_video_capture.get_latest_frame()
                
                if frame is not None:
                    self.frame_count += 1
                    
                    # 保存原始畫面（未 resize）
                    self.original_frame = frame.copy()
                    
                    # 只在第一次或縮放比例改變時計算顯示尺寸
                    original_height, original_width = frame.shape[:2]
                    if self.display_width is None or self.display_height is None:
                        self.display_width = int(original_width * self.display_scale)
                        self.display_height = int(original_height * self.display_scale)
                        logger.info(f"顯示尺寸設定: {self.display_width}x{self.display_height} (縮放: {self.display_scale:.2f})")
                    
                    # 為顯示創建縮放後的畫面（只在需要時 resize）
                    if (self.display_frame is None or 
                        self.display_frame.shape[1] != self.display_width or 
                        self.display_frame.shape[0] != self.display_height):
                        
                        self.display_frame = cv2.resize(frame, (self.display_width, self.display_height), 
                                                       interpolation=cv2.INTER_LINEAR)
                    else:
                        # 重用已有的 display_frame 尺寸，只更新內容
                        cv2.resize(frame, (self.display_width, self.display_height), 
                                  dst=self.display_frame, interpolation=cv2.INTER_LINEAR)
                    
                    self.current_frame = self.display_frame
                    
                    # 計算 FPS
                    current_time = time.time()
                    elapsed = current_time - self.start_time
                    if elapsed > 1.0:
                        self.fps = self.frame_count / elapsed
                        self.frame_count = 0
                        self.start_time = current_time
                    
                    # 繪製 ROI（直接在 display_frame 上繪製以避免額外的 copy）
                    if self.roi_selected and self.roi_start and self.roi_end:
                        cv2.rectangle(self.display_frame, self.roi_start, self.roi_end, (0, 255, 0), 2)
                        cv2.putText(self.display_frame, "OCR Region", 
                                   (self.roi_start[0], self.roi_start[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    elif self.roi_selecting and self.roi_start and self.roi_end:
                        cv2.rectangle(self.display_frame, self.roi_start, self.roi_end, (255, 255, 0), 2)
                    
                    # 添加信息文字
                    info_y = 30
                    cv2.putText(self.display_frame, f"FPS: {self.fps:.1f}", (10, info_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    info_y += 30
                    cv2.putText(self.display_frame, f"Original: {original_width}x{original_height}", (10, info_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    info_y += 25
                    cv2.putText(self.display_frame, f"Display: {self.display_width}x{self.display_height}", (10, info_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    if self.obs_audio_capture:
                        info_y += 25
                        audio_level = self.obs_audio_capture.get_audio_level()
                        cv2.putText(self.display_frame, f"Audio: {audio_level:.2f}", (10, info_y),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    
                    cv2.imshow('OCR Camera', self.display_frame)
                
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
                    filename = f"ocr_frame_{timestamp}.jpg"
                    cv2.imwrite(filename, self.original_frame)
                    logger.info(f"已保存原始畫面: {filename}")
                elif key == ord('+') or key == ord('='):
                    old_scale = self.display_scale
                    self.display_scale = min(1.0, self.display_scale + 0.1)
                    if self.display_scale != old_scale:
                        # 重新計算顯示尺寸
                        self.display_width = None
                        self.display_height = None
                    logger.info(f"顯示縮放: {self.display_scale:.1f}")
                elif key == ord('-'):
                    old_scale = self.display_scale
                    self.display_scale = max(0.2, self.display_scale - 0.1)
                    if self.display_scale != old_scale:
                        # 重新計算顯示尺寸
                        self.display_width = None
                        self.display_height = None
                    logger.info(f"顯示縮放: {self.display_scale:.1f}")
                
        except KeyboardInterrupt:
            logger.info("收到中斷信號")
        
        finally:
            self.stop_system()
    
    def stop_system(self):
        """停止系統"""
        logger.info("正在停止 OCR 系統...")
        
        self.running = False
        
        cv2.destroyAllWindows()
        
        if self.obs_audio_capture:
            self.obs_audio_capture.stop_capture()
        
        if self.obs_video_capture:
            self.obs_video_capture.stop()
        
        if self.insta_worker:
            self.insta_worker.stop_all()
        
        if hasattr(self, 'system_monitor'):
            self.system_monitor.stop_monitoring()
        
        logger.info("✓ OCR 系統已停止")

def main():
    """主函數"""
    try:
        ocr_capture = OCRCapture()
        ocr_capture.run()
    except Exception as e:
        logger.error(f"程序執行錯誤: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
