# Profile management API endpoints

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fastapi import APIRouter, HTTPException, status, Request
from fastapi.responses import JSONResponse

from schemas.profile import ProfileUpdateRequest, ProfileResponse
from services.auth_service import get_user_by_id, update_user_profile
from services.personalization_service import get_user_preferences
from middleware.auth import require_session

router = APIRouter(prefix="/api", tags=["Profile"])


@router.get("/profile", response_model=ProfileResponse)
async def get_profile(http_request: Request):
    """
    Get current user's profile.
    """
    session = await require_session(http_request)

    result = get_user_by_id(session.user_id)

    if not result:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    user_data, profile_data = result

    return ProfileResponse(
        success=True,
        user=user_data,
        profile=profile_data
    )


@router.put("/profile", response_model=ProfileResponse)
async def update_profile(request: ProfileUpdateRequest, http_request: Request):
    """
    Update user profile preferences.
    """
    session = await require_session(http_request)

    # Update profile
    updated_profile = update_user_profile(
        user_id=session.user_id,
        software_level=request.software_level if request.software_level else None,
        hardware_experience=request.hardware_experience if request.hardware_experience else None,
        learning_depth=request.learning_depth if request.learning_depth else None
    )

    if not updated_profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )

    # Get user data
    result = get_user_by_id(session.user_id)
    if not result:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    user_data, _ = result

    return ProfileResponse(
        success=True,
        user=user_data,
        profile=updated_profile
    )


@router.get("/preferences")
async def get_preferences(http_request: Request):
    """
    Get user preferences for personalization.
    """
    session = await require_session(http_request)

    prefs = get_user_preferences(session.user_id)

    if not prefs:
        # Return default preferences
        return {
            "software_level": "Beginner",
            "hardware_experience": "None",
            "learning_depth": "Both",
            "personalization_enabled": True
        }

    return prefs.to_dict()
