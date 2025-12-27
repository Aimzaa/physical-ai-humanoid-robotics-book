# Personalization service - handles user profile and content preferences

import json
from typing import Optional, List, Dict, Any
from datetime import datetime
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from db import get_connection


class UserPreferences:
    """User preferences for content personalization."""

    def __init__(
        self,
        software_level: str = "Beginner",
        hardware_experience: str = "None",
        learning_depth: str = "Both",
        personalization_enabled: bool = True,
        favorite_chapters: List[str] = None,
        last_personalized_chapter: str = None
    ):
        self.software_level = software_level
        self.hardware_experience = hardware_experience
        self.learning_depth = learning_depth
        self.personalization_enabled = personalization_enabled
        self.favorite_chapters = favorite_chapters or []
        self.last_personalized_chapter = last_personalized_chapter

    def to_dict(self) -> dict:
        return {
            "software_level": self.software_level,
            "hardware_experience": self.hardware_experience,
            "learning_depth": self.learning_depth,
            "personalization_enabled": self.personalization_enabled,
            "favorite_chapters": self.favorite_chapters,
            "last_personalized_chapter": self.last_personalized_chapter
        }

    @classmethod
    def from_dict(cls, data) -> "UserPreferences":
        # Handle both dict and sqlite3.Row objects
        def get_val(key, default=None):
            if hasattr(data, 'keys'):
                # It's a sqlite3.Row or dict-like object
                return data[key] if key in data.keys() else default
            elif hasattr(data, 'get'):
                # It's a regular dict
                return data.get(key, default)
            else:
                # Try direct access
                try:
                    return data[key]
                except (KeyError, TypeError):
                    return default

        favorite_chapters_raw = get_val("favorite_chapters")
        favorite_chapters = json.loads(favorite_chapters_raw) if favorite_chapters_raw else []

        return cls(
            software_level=get_val("software_level", "Beginner"),
            hardware_experience=get_val("hardware_experience", "None"),
            learning_depth=get_val("learning_depth", "Both"),
            personalization_enabled=bool(get_val("personalization_enabled", True)),
            favorite_chapters=favorite_chapters,
            last_personalized_chapter=get_val("last_personalized_chapter")
        )


def get_user_preferences(user_id: str) -> Optional[UserPreferences]:
    """
    Get user preferences for personalization.

    Returns UserPreferences or None if user not found.
    """
    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        "SELECT software_level, hardware_experience, learning_depth, "
        "personalization_enabled, favorite_chapters, last_personalized_chapter "
        "FROM user_profile WHERE id = ?",
        (user_id,)
    )
    row = cursor.fetchone()

    conn.close()

    if not row:
        return None

    return UserPreferences.from_dict(row)


def get_default_preferences() -> UserPreferences:
    """Get default preferences for guest users."""
    return UserPreferences()


def update_personalization_state(
    user_id: str,
    chapter_id: str,
    enabled: bool = True
) -> Dict[str, Any]:
    """
    Update personalization state for a chapter.

    Returns: Updated preferences dict.
    """
    conn = get_connection()
    cursor = conn.cursor()

    now = datetime.now().isoformat()

    if enabled:
        cursor.execute(
            "UPDATE user_profile SET personalization_enabled = 1, "
            "last_personalized_chapter = ?, updated_at = ? WHERE id = ?",
            (chapter_id, now, user_id)
        )
    else:
        cursor.execute(
            "UPDATE user_profile SET personalization_enabled = 0, updated_at = ? WHERE id = ?",
            (now, user_id)
        )

    conn.commit()
    conn.close()

    return get_user_preferences(user_id).to_dict()


def toggle_favorite_chapter(user_id: str, chapter_id: str) -> List[str]:
    """
    Toggle a chapter as favorite.

    Returns: Updated favorite chapters list.
    """
    conn = get_connection()
    cursor = conn.cursor()

    # Get current favorites
    cursor.execute("SELECT favorite_chapters FROM user_profile WHERE id = ?", (user_id,))
    row = cursor.fetchone()

    if not row:
        conn.close()
        return []

    favorites = json.loads(row["favorite_chapters"]) if row["favorite_chapters"] else []

    # Toggle chapter
    if chapter_id in favorites:
        favorites.remove(chapter_id)
    else:
        favorites.append(chapter_id)

    # Update
    cursor.execute(
        "UPDATE user_profile SET favorite_chapters = ?, updated_at = ? WHERE id = ?",
        (json.dumps(favorites), datetime.now().isoformat(), user_id)
    )

    conn.commit()
    conn.close()

    return favorites


def get_applied_sections(user_id: str, chapter_id: str) -> List[str]:
    """
    Determine which content sections should be applied based on user profile.

    This is a placeholder for the actual content adaptation logic.
    In practice, this would parse chapter markdown and extract relevant sections.

    Returns: List of section IDs to apply.
    """
    prefs = get_user_preferences(user_id)

    if not prefs or not prefs.personalization_enabled:
        return []

    sections = []

    # Add section based on software level
    level_map = {
        "Beginner": "beginner",
        "Intermediate": "intermediate",
        "Advanced": "advanced"
    }
    sections.append(f"{level_map.get(prefs.software_level, 'intermediate')}-intro")

    # Add section based on learning depth
    depth_map = {
        "Conceptual": "conceptual",
        "Practical": "practical",
        "Both": "both"
    }
    sections.append(f"{depth_map.get(prefs.learning_depth, 'both')}-{level_map.get(prefs.software_level, 'level')}")

    # Add hardware experience section if hands-on
    if prefs.hardware_experience == "Hands-on":
        sections.append("hands-on")

    return sections
