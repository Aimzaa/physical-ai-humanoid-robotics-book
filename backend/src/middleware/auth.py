# Authentication middleware and session handling

from typing import Optional
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from datetime import datetime, timedelta
import sqlite3
import uuid
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from db import get_connection


# Cookie name for session token
SESSION_COOKIE_NAME = "book-auth_session-token"


class AuthSession:
    """Represents an authenticated user session."""

    def __init__(self, user_id: str, email: str, created_at: datetime, last_login: datetime = None):
        self.user_id = user_id
        self.email = email
        self.created_at = created_at
        self.last_login = last_login

    def to_dict(self) -> dict:
        return {
            "id": self.user_id,
            "email": self.email,
            "created_at": self.created_at,
            "last_login": self.last_login
        }


async def get_session(request: Request) -> Optional[AuthSession]:
    """
    Extract and validate session from HttpOnly cookie.

    Returns AuthSession if valid, None if not authenticated.
    """
    cookie_value = request.cookies.get(SESSION_COOKIE_NAME)

    if not cookie_value:
        return None

    try:
        conn = get_connection()
        cursor = conn.cursor()

        # Find session by token
        cursor.execute(
            "SELECT s.user_id, s.expires_at, u.email, u.created_at, u.last_login "
            "FROM session s "
            "JOIN user u ON s.user_id = u.id "
            "WHERE s.token = ?",
            (cookie_value,)
        )
        row = cursor.fetchone()

        conn.close()

        if not row:
            return None

        # Check if session expired
        expires_at = datetime.fromisoformat(row["expires_at"])
        if expires_at < datetime.now():
            return None

        # Return session
        return AuthSession(
            user_id=row["user_id"],
            email=row["email"],
            created_at=datetime.fromisoformat(row["created_at"]),
            last_login=datetime.fromisoformat(row["last_login"]) if row["last_login"] else None
        )

    except Exception as e:
        print(f"Session validation error: {e}")
        return None


async def require_session(request: Request) -> AuthSession:
    """
    Require a valid session - raises 401 if not authenticated.
    """
    session = await get_session(request)

    if not session:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Cookie"},
        )

    return session


async def optional_session(request: Request) -> Optional[AuthSession]:
    """
    Optional session - returns session if present, None otherwise.
    Does not raise exceptions.
    """
    try:
        return await get_session(request)
    except Exception:
        return None


def create_session(
    user_id: str,
    expires_days: int = 30,
    ip_address: str = None,
    user_agent: str = None
) -> tuple[str, datetime]:
    """
    Create a new session token and store in database.

    Returns: (token, expires_at)
    """
    token = str(uuid.uuid4())
    expires_at = datetime.now() + timedelta(days=expires_days)
    expires_at_str = expires_at.isoformat()
    created_at = datetime.now().isoformat()

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        "INSERT INTO session (id, user_id, token, expires_at, ip_address, user_agent, created_at) "
        "VALUES (?, ?, ?, ?, ?, ?, ?)",
        (str(uuid.uuid4()), user_id, token, expires_at_str, ip_address, user_agent, created_at)
    )

    conn.commit()
    conn.close()

    return token, expires_at


def delete_session(token: str) -> bool:
    """Delete a session by token."""
    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute("DELETE FROM session WHERE token = ?", (token,))

    deleted = cursor.rowcount > 0
    conn.commit()
    conn.close()

    return deleted


def delete_user_sessions(user_id: str) -> int:
    """Delete all sessions for a user (logout from all devices)."""
    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute("DELETE FROM session WHERE user_id = ?", (user_id,))

    deleted = cursor.rowcount
    conn.commit()
    conn.close()

    return deleted


def cleanup_expired_sessions() -> int:
    """Remove all expired sessions. Call periodically."""
    conn = get_connection()
    cursor = conn.cursor()

    now = datetime.now().isoformat()
    cursor.execute("DELETE FROM session WHERE expires_at < ?", (now,))

    deleted = cursor.rowcount
    conn.commit()
    conn.close()

    return deleted
