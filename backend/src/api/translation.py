# Translation API Router
# Feature: Chapter-Level Urdu Translation for Logged-in Users
# Date: 2025-12-28

from fastapi import APIRouter, HTTPException, status, Request
from pydantic import BaseModel
from typing import Optional
from datetime import datetime, timedelta
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from services.translation_service import TranslationService

router = APIRouter(prefix="/api/translate", tags=["Translation"])


# =============================================================================
# Pydantic Models
# =============================================================================

class TranslationRequest(BaseModel):
    chapter_id: str
    chapter_title: str
    content: str


class TranslationResponse(BaseModel):
    success: bool
    translated_content: Optional[str] = None
    cached: bool = False
    ttl_seconds: Optional[int] = None
    language: str = "ur"


class TranslationStatusResponse(BaseModel):
    has_translation: bool
    cached_at: Optional[str] = None
    ttl_remaining: Optional[int] = None
    language: Optional[str] = None


class ClearTranslationResponse(BaseModel):
    success: bool
    message: str


class ErrorResponse(BaseModel):
    detail: str
    code: str = "TRANSLATION_ERROR"


# =============================================================================
# Translation Endpoints
# =============================================================================

@router.post("/{chapter_id}", response_model=TranslationResponse)
async def translate_chapter(
    chapter_id: str,
    request: TranslationRequest,
):
    """
    Translate a chapter to Urdu.

    For logged-in users, uses profile-adaptive translation.
    For testing, works without authentication.
    """
    # Validate chapter_id matches request
    if chapter_id != request.chapter_id:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Chapter ID mismatch",
        )

    try:
        service = TranslationService()
        result = await service.translate(
            user_id="anonymous",  # Use anonymous for now
            chapter_id=chapter_id,
            chapter_title=request.chapter_title,
            content=request.content,
        )

        return TranslationResponse(
            success=True,
            translated_content=result.get("translated_content"),
            cached=result.get("cached", False),
            ttl_seconds=result.get("ttl_seconds"),
            language="ur",
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}",
        )


@router.get("/{chapter_id}/status", response_model=TranslationStatusResponse)
async def get_translation_status(chapter_id: str):
    """
    Check if a cached translation exists for the chapter.
    """
    try:
        service = TranslationService()
        status_info = await service.get_cache_status(
            user_id="anonymous",
            chapter_id=chapter_id,
        )

        return TranslationStatusResponse(
            has_translation=status_info.get("has_translation", False),
            cached_at=status_info.get("cached_at"),
            ttl_remaining=status_info.get("ttl_remaining"),
            language=status_info.get("language"),
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get translation status: {str(e)}",
        )


@router.delete("/{chapter_id}", response_model=ClearTranslationResponse)
async def clear_translation(chapter_id: str):
    """
    Clear the cached translation for a specific chapter.
    """
    try:
        service = TranslationService()
        await service.clear_cache(
            user_id="anonymous",
            chapter_id=chapter_id,
        )

        return ClearTranslationResponse(
            success=True,
            message=f"Translation cache cleared for {chapter_id}",
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to clear translation: {str(e)}",
        )


@router.get("/health")
async def translation_health():
    """Health check for translation service."""
    return {
        "status": "healthy",
        "service": "translation",
        "api": "mymemory (free)",
    }
