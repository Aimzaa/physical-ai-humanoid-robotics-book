# Authentication request/response schemas

from pydantic import BaseModel, EmailStr, Field, field_validator, BeforeValidator
from typing import Optional, Annotated
from datetime import datetime
from enum import Enum


# Simple enums for profile fields (Python 3.13 compatible)
class SoftwareLevel:
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"

    @classmethod
    def values(cls):
        return [cls.BEGINNER, cls.INTERMEDIATE, cls.ADVANCED]


class HardwareExperience:
    NONE = "None"
    BASIC = "Basic"
    HANDSON = "Hands-on"

    @classmethod
    def values(cls):
        return [cls.NONE, cls.BASIC, cls.HANDSON]


class LearningDepth:
    CONCEPTUAL = "Conceptual"
    PRACTICAL = "Practical"
    BOTH = "Both"

    @classmethod
    def values(cls):
        return [cls.CONCEPTUAL, cls.PRACTICAL, cls.BOTH]


# Validator functions
def validate_software_level(v):
    if v not in SoftwareLevel.values():
        raise ValueError(f"software_level must be one of {SoftwareLevel.values()}")
    return v


def validate_hardware_experience(v):
    if v not in HardwareExperience.values():
        raise ValueError(f"hardware_experience must be one of {HardwareExperience.values()}")
    return v


def validate_learning_depth(v):
    if v not in LearningDepth.values():
        raise ValueError(f"learning_depth must be one of {LearningDepth.values()}")
    return v


# Request schemas
class SignupRequest(BaseModel):
    """Request body for user signup."""
    email: EmailStr
    password: str = Field(..., min_length=8, max_length=128)
    confirm_password: str
    software_level: Annotated[str, BeforeValidator(validate_software_level)]
    hardware_experience: Annotated[str, BeforeValidator(validate_hardware_experience)]
    learning_depth: Annotated[str, BeforeValidator(validate_learning_depth)]

    @field_validator("confirm_password")
    @classmethod
    def passwords_match(cls, v: str, info) -> str:
        if "password" in info.data and v != info.data["password"]:
            raise ValueError("passwords do not match")
        return v


class SigninRequest(BaseModel):
    """Request body for user signin."""
    email: EmailStr
    password: str


# Response schemas
class UserResponse(BaseModel):
    """User information in responses."""
    id: str
    email: EmailStr
    created_at: datetime
    last_login: Optional[datetime] = None


class ProfileData(BaseModel):
    """User profile data for personalization."""
    software_level: str
    hardware_experience: str
    learning_depth: str
    personalization_enabled: bool = True
    last_personalized_chapter: Optional[str] = None
    created_at: datetime
    updated_at: Optional[datetime] = None


class AuthResponse(BaseModel):
    """Response after successful signup or signin."""
    success: bool = True
    user: UserResponse
    message: str = "Operation successful"


class UserProfileResponse(BaseModel):
    """Full user profile response."""
    user: UserResponse
    profile: ProfileData


class ErrorResponse(BaseModel):
    """Error response schema."""
    success: bool = False
    error: str
    code: str
