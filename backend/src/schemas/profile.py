# Profile and personalization request/response schemas

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ProfileUpdateRequest(BaseModel):
    """Request to update user profile preferences."""
    software_level: Optional[str] = None
    hardware_experience: Optional[str] = None
    learning_depth: Optional[str] = None
    display_name: Optional[str] = Field(None, max_length=100)
    bio: Optional[str] = Field(None, max_length=500)

    class Config:
        extra = "forbid"


class ProfileResponse(BaseModel):
    """Full profile response."""
    success: bool = True
    user: dict
    profile: dict


class PersonalizationResponse(BaseModel):
    """Response after personalizing a chapter."""
    success: bool = True
    chapter_id: str
    user_level: dict
    applied_sections: List[str] = []
