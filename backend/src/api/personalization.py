# Personalization API endpoints

import sys
import os
import openai
from pydantic import BaseModel
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fastapi import APIRouter, HTTPException, status, Request
from fastapi.responses import JSONResponse

from schemas.profile import PersonalizationResponse
from services.personalization_service import (
    get_user_preferences, update_personalization_state, get_applied_sections
)
from middleware.auth import require_session

router = APIRouter(prefix="/api", tags=["Personalization"])

# Get OpenRouter API key
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")


class PersonalizeContentRequest(BaseModel):
    """Request to personalize chapter content."""
    chapter_title: str
    chapter_content: str
    section_title: Optional[str] = None


@router.post("/personalize/{chapter_id}", response_model=PersonalizationResponse)
async def personalize_chapter(chapter_id: str, http_request: Request):
    """
    Enable personalization for a specific chapter.

    Returns information about which sections will be applied based on user profile.
    """
    session = await require_session(http_request)

    # Update personalization state
    updated = update_personalization_state(
        user_id=session.user_id,
        chapter_id=chapter_id,
        enabled=True
    )

    # Get sections that will be applied
    sections = get_applied_sections(session.user_id, chapter_id)

    return PersonalizationResponse(
        success=True,
        chapter_id=chapter_id,
        user_level={
            "software_level": updated.get("software_level", "Beginner"),
            "hardware_experience": updated.get("hardware_experience", "None"),
            "learning_depth": updated.get("learning_depth", "Both")
        },
        applied_sections=sections
    )


@router.post("/personalize/{chapter_id}/disable")
async def disable_personalization(chapter_id: str, http_request: Request):
    """
    Disable personalization for a specific chapter.
    """
    session = await require_session(http_request)

    update_personalization_state(
        user_id=session.user_id,
        chapter_id=chapter_id,
        enabled=False
    )

    return {"success": True, "message": f"Personalization disabled for {chapter_id}"}


@router.get("/personalize/{chapter_id}/sections")
async def get_applied_sections_for_chapter(chapter_id: str, http_request: Request):
    """
    Get the list of sections that will be applied for a chapter based on user profile.
    """
    session = await require_session(http_request)

    sections = get_applied_sections(session.user_id, chapter_id)
    prefs = get_user_preferences(session.user_id)

    return {
        "chapter_id": chapter_id,
        "applied_sections": sections,
        "user_level": {
            "software_level": prefs.software_level if prefs else "Beginner",
            "hardware_experience": prefs.hardware_experience if prefs else "None",
            "learning_depth": prefs.learning_depth if prefs else "Both"
        }
    }


@router.post("/personalize/{chapter_id}/content")
async def generate_personalized_content(
    chapter_id: str,
    request: PersonalizeContentRequest,
    http_request: Request
):
    """
    Generate personalized content for a chapter section based on user profile.
    Uses LLM to adapt the content to user's experience level.
    """
    session = await require_session(http_request)

    if not OPENROUTER_API_KEY:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="OpenRouter API key not configured"
        )

    # Get user preferences
    prefs = get_user_preferences(session.user_id)
    if not prefs:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User preferences not found"
        )

    # Build personalization prompt based on user level
    level_instructions = {
        "Beginner": """
- Use simple, everyday language and avoid jargon
- Explain every technical term when first introduced
- Use analogies to familiar concepts (like comparing robot sensors to human senses)
- Break down complex concepts into small, digestible steps
- Include "Why this matters" explanations
- Add encouraging notes for beginners""",
        "Intermediate": """
- Assume basic programming knowledge (variables, functions, loops)
- Explain robotics-specific concepts but not basic programming
- Include code examples with moderate comments
- Connect concepts to practical applications
- Mention best practices and common patterns""",
        "Advanced": """
- Use technical terminology freely
- Focus on advanced techniques, optimizations, and edge cases
- Include performance considerations and trade-offs
- Reference academic papers or advanced resources where relevant
- Discuss architectural decisions and design patterns"""
    }

    depth_instructions = {
        "Conceptual": "Focus on theory, principles, and understanding WHY things work. Minimize code examples.",
        "Practical": "Focus on hands-on implementation, code examples, and HOW to do things. Minimize theory.",
        "Both": "Balance theory with practical examples. Explain concepts then show implementation."
    }

    hardware_instructions = {
        "None": "Explain hardware concepts from scratch. Use diagrams descriptions and analogies.",
        "Basic": "Assume basic understanding of electronics and sensors. Focus on robotics-specific hardware.",
        "Hands-on": "Assume practical experience with hardware. Focus on integration and advanced configurations."
    }

    # Configure OpenAI client for OpenRouter
    openai.api_key = OPENROUTER_API_KEY
    openai.base_url = "https://openrouter.ai/api/v1/"

    system_prompt = f"""You are an expert technical writer for a Physical AI & Humanoid Robotics textbook.
Your task is to rewrite the given content to match the reader's experience level.

Reader Profile:
- Software Experience: {prefs.software_level}
- Hardware Experience: {prefs.hardware_experience}
- Learning Preference: {prefs.learning_depth}

Adaptation Instructions for {prefs.software_level} level:
{level_instructions.get(prefs.software_level, level_instructions["Intermediate"])}

Learning Depth ({prefs.learning_depth}):
{depth_instructions.get(prefs.learning_depth, depth_instructions["Both"])}

Hardware Background ({prefs.hardware_experience}):
{hardware_instructions.get(prefs.hardware_experience, hardware_instructions["Basic"])}

IMPORTANT RULES:
1. Keep the same topic and core information - just adapt the explanation style
2. Maintain technical accuracy - don't oversimplify to the point of being wrong
3. Keep code examples if present (adjust comments based on level)
4. Use markdown formatting for headers, code blocks, lists, etc.
5. Keep the content roughly the same length (can be slightly longer for beginners)
6. Do NOT add generic introductions or conclusions - just adapt the content itself"""

    user_prompt = f"""Please adapt this content from "{request.chapter_title}" for a {prefs.software_level} level reader:

{request.chapter_content}

Rewrite this content maintaining the same structure but adapting the explanation style, examples, and depth to match the reader's profile."""

    try:
        response = openai.chat.completions.create(
            model="openai/gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,
            max_tokens=4000
        )

        personalized_content = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0

        # Update state to track this personalization
        update_personalization_state(
            user_id=session.user_id,
            chapter_id=chapter_id,
            enabled=True
        )

        return {
            "success": True,
            "chapter_id": chapter_id,
            "original_title": request.chapter_title,
            "personalized_content": personalized_content,
            "user_level": {
                "software_level": prefs.software_level,
                "hardware_experience": prefs.hardware_experience,
                "learning_depth": prefs.learning_depth
            },
            "tokens_used": tokens_used
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate personalized content: {str(e)}"
        )
