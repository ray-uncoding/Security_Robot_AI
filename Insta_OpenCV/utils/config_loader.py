# utils/config_loader.py

import json
import os

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/settings.json")

DEFAULT_SETTINGS = {
    "insta_ip": "192.168.1.100",
    "http_port": 80,
    "fingerprint": "",
    "heartbeat_interval": 1,
    "preview_mode": "rtmp",
    "review_allowed_ip": "192.168.1.101",
    "rtmp_url": "rtmp://192.168.1.100/live/stream",
    "resolution": "3840x1920",
    "bitrate": 8000000,
    "framerate": 30,
    "record_audio": True
}

def load_settings() -> dict:
    """
    讀取設定檔內容，若不存在則使用預設值並自動建立檔案。
    """
    if not os.path.exists(CONFIG_PATH):
        save_settings(DEFAULT_SETTINGS)
        return DEFAULT_SETTINGS.copy()
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)

def save_settings(settings: dict):
    """
    將設定內容儲存至設定檔。
    """
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(settings, f, indent=4, ensure_ascii=False)

def get_setting(key: str, default=None):
    """
    安全取得指定設定值，若不存在則回傳預設值。
    """
    settings = load_settings()
    return settings.get(key, default)

def update_setting(key: str, value):
    """
    更新單一設定值並自動儲存。
    """
    settings = load_settings()
    settings[key] = value
    save_settings(settings)
