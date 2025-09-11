"""ROI 區域選擇資料模型"""
from dataclasses import dataclass
from typing import Optional, Tuple
from PyQt5.QtCore import QPoint

@dataclass
class ROISelection:
    """ROI 區域選擇資料結構"""
    start_point: Optional[QPoint] = None
    end_point: Optional[QPoint] = None
    is_selecting: bool = False
    is_selected: bool = False
    
    @property
    def coordinates(self) -> Optional[Tuple[int, int, int, int]]:
        """返回 ROI 座標 (x1, y1, x2, y2)"""
        if not (self.start_point and self.end_point):
            return None
            
        x1 = min(self.start_point.x(), self.end_point.x())
        y1 = min(self.start_point.y(), self.end_point.y())
        x2 = max(self.start_point.x(), self.end_point.x())
        y2 = max(self.start_point.y(), self.end_point.y())
        
        return (x1, y1, x2, y2)
    
    @property
    def width(self) -> int:
        """ROI 寬度"""
        coords = self.coordinates
        return coords[2] - coords[0] if coords else 0
    
    @property
    def height(self) -> int:
        """ROI 高度"""
        coords = self.coordinates
        return coords[3] - coords[1] if coords else 0
    
    def clear(self):
        """清除 ROI 選擇"""
        self.start_point = None
        self.end_point = None
        self.is_selecting = False
        self.is_selected = False
    
    def is_valid(self) -> bool:
        """檢查 ROI 是否有效"""
        coords = self.coordinates
        if not coords:
            return False
        x1, y1, x2, y2 = coords
        return x2 > x1 and y2 > y1
