# Authentication service - handles user registration and login

import bcrypt
import uuid
from datetime import datetime
from typing import Optional, Tuple
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from db import get_connection


# Simple enums for profile fields (Python 3.13 compatible)
class SoftwareLevel:
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"
    @classmethod
    def values(cls):
        return [cls.BEGINNER, cls.INTERMEDIATE, cls.ADVANCED]


def hash_password(password: str) -> str:
    """Hash a password using bcrypt."""
    salt = bcrypt.gensalt()
    return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')


def verify_password(password: str, password_hash: str) -> bool:
    """Verify a password against its hash."""
    try:
        return bcrypt.checkpw(
            password.encode('utf-8'),
            password_hash.encode('utf-8')
        )
    except Exception:
        return False


def create_user(
    email: str,
    password: str,
    software_level: str,
    hardware_experience: str,
    learning_depth: str
) -> Tuple[str, dict, dict]:
    """
    Create a new user with profile.

    Returns: (user_id, user_data, profile_data)
    """
    # Hash password
    password_hash = hash_password(password)

    # Generate IDs
    user_id = str(uuid.uuid4())
    now = datetime.now().isoformat()

    conn = get_connection()
    cursor = conn.cursor()

    try:
        # Insert user
        cursor.execute(
            "INSERT INTO user (id, email, password_hash, email_verified, created_at, last_login) "
            "VALUES (?, ?, ?, 0, ?, ?)",
            (user_id, email, password_hash, now, now)
        )

        # Insert profile
        cursor.execute(
            "INSERT INTO user_profile (id, software_level, hardware_experience, learning_depth, "
            "personalization_enabled, favorite_chapters, last_personalized_chapter, created_at, updated_at) "
            "VALUES (?, ?, ?, ?, 1, ?, ?, ?, ?)",
            (user_id, software_level, hardware_experience, learning_depth, '[]', None, now, None)
        )

        conn.commit()

        # Parse datetime for response (Pydantic requires datetime objects, not strings)
        created_datetime = datetime.fromisoformat(now)

        user_data = {
            "id": user_id,
            "email": email,
            "created_at": created_datetime,
            "last_login": created_datetime
        }

        profile_data = {
            "software_level": software_level,
            "hardware_experience": hardware_experience,
            "learning_depth": learning_depth,
            "personalization_enabled": True,
            "last_personalized_chapter": None,
            "created_at": created_datetime,
            "updated_at": None
        }

        return user_id, user_data, profile_data

    except Exception as e:
        conn.rollback()
        raise ValueError(f"Email already exists: {email}") from e

    finally:
        conn.close()


def authenticate_user(email: str, password: str) -> Optional[Tuple[str, dict, dict]]:
    """
    Authenticate a user with email and password.

    Returns: (user_id, user_data, profile_data) if successful, None otherwise.
    """
    conn = get_connection()
    cursor = conn.cursor()

    # Find user by email
    cursor.execute(
        "SELECT id, email, password_hash, created_at, last_login FROM user WHERE email = ?",
        (email,)
    )
    row = cursor.fetchone()

    if not row:
        conn.close()
        return None

    # Verify password
    if not verify_password(password, row["password_hash"]):
        conn.close()
        return None

    # Get user ID and update last_login
    user_id = row["id"]
    now = datetime.now().isoformat()

    cursor.execute(
        "UPDATE user SET last_login = ? WHERE id = ?",
        (now, user_id)
    )

    # Get profile
    cursor.execute(
        "SELECT software_level, hardware_experience, learning_depth, "
        "personalization_enabled, last_personalized_chapter, created_at, updated_at "
        "FROM user_profile WHERE id = ?",
        (user_id,)
    )
    profile_row = cursor.fetchone()

    conn.commit()
    conn.close()

    # Helper to parse datetime strings
    def parse_dt(val):
        if val is None:
            return None
        if isinstance(val, datetime):
            return val
        return datetime.fromisoformat(val)

    user_data = {
        "id": user_id,
        "email": row["email"],
        "created_at": parse_dt(row["created_at"]),
        "last_login": parse_dt(row["last_login"])
    }

    if profile_row:
        profile_data = {
            "software_level": profile_row["software_level"],
            "hardware_experience": profile_row["hardware_experience"],
            "learning_depth": profile_row["learning_depth"],
            "personalization_enabled": bool(profile_row["personalization_enabled"]),
            "last_personalized_chapter": profile_row["last_personalized_chapter"],
            "created_at": parse_dt(profile_row["created_at"]),
            "updated_at": parse_dt(profile_row["updated_at"])
        }
    else:
        profile_data = {
            "software_level": "Beginner",
            "hardware_experience": "None",
            "learning_depth": "Both",
            "personalization_enabled": True,
            "last_personalized_chapter": None,
            "created_at": datetime.fromisoformat(now),
            "updated_at": None
        }

    return user_id, user_data, profile_data


def get_user_by_id(user_id: str) -> Optional[Tuple[dict, dict]]:
    """
    Get user and profile by user ID.

    Returns: (user_data, profile_data) or None if not found.
    """
    conn = get_connection()
    cursor = conn.cursor()

    # Get user
    cursor.execute(
        "SELECT id, email, password_hash, created_at, last_login FROM user WHERE id = ?",
        (user_id,)
    )
    user_row = cursor.fetchone()

    if not user_row:
        conn.close()
        return None

    # Get profile
    cursor.execute(
        "SELECT software_level, hardware_experience, learning_depth, "
        "personalization_enabled, favorite_chapters, last_personalized_chapter, created_at, updated_at "
        "FROM user_profile WHERE id = ?",
        (user_id,)
    )
    profile_row = cursor.fetchone()

    conn.close()

    # Helper to parse datetime strings
    def parse_dt(val):
        if val is None:
            return None
        if isinstance(val, datetime):
            return val
        return datetime.fromisoformat(val)

    user_data = {
        "id": user_row["id"],
        "email": user_row["email"],
        "created_at": parse_dt(user_row["created_at"]),
        "last_login": parse_dt(user_row["last_login"])
    }

    if profile_row:
        profile_data = {
            "software_level": profile_row["software_level"],
            "hardware_experience": profile_row["hardware_experience"],
            "learning_depth": profile_row["learning_depth"],
            "personalization_enabled": bool(profile_row["personalization_enabled"]),
            "favorite_chapters": profile_row["favorite_chapters"],
            "last_personalized_chapter": profile_row["last_personalized_chapter"],
            "created_at": parse_dt(profile_row["created_at"]),
            "updated_at": parse_dt(profile_row["updated_at"])
        }
    else:
        profile_data = None

    return user_data, profile_data


def update_user_profile(
    user_id: str,
    software_level: str = None,
    hardware_experience: str = None,
    learning_depth: str = None
) -> dict:
    """
    Update user profile preferences.

    Returns: Updated profile data.
    """
    conn = get_connection()
    cursor = conn.cursor()

    now = datetime.now().isoformat()

    # Build update query dynamically
    updates = []
    values = []

    if software_level is not None:
        updates.append("software_level = ?")
        values.append(software_level)
    if hardware_experience is not None:
        updates.append("hardware_experience = ?")
        values.append(hardware_experience)
    if learning_depth is not None:
        updates.append("learning_depth = ?")
        values.append(learning_depth)

    if not updates:
        conn.close()
        return get_user_by_id(user_id)[1]

    updates.append("updated_at = ?")
    values.append(now)
    values.append(user_id)

    query = f"UPDATE user_profile SET {', '.join(updates)} WHERE id = ?"
    cursor.execute(query, values)

    conn.commit()
    conn.close()

    # Return updated profile
    result = get_user_by_id(user_id)
    return result[1] if result else None
