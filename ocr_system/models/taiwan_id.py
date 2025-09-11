"""台灣身份證資料模型"""
from pydantic import BaseModel, Field
from typing import Optional

class TaiwanIDCard(BaseModel):
    """台灣身份證資料結構"""
    id_number: str = Field(..., description="身分證號碼")
    name: str = Field(..., description="姓名")
    
    class Config:
        """Pydantic 配置"""
        json_encoders = {
            str: lambda v: v.strip() if v else ""
        }
        
    def __str__(self) -> str:
        return f"姓名: {self.name}, 身分證號: {self.id_number}"
    
    def to_dict(self) -> dict:
        """轉換為字典格式"""
        return {
            "id_number": self.id_number,
            "name": self.name
        }
