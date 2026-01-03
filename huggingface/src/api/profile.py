from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional, List
import sqlite3
import os

router = APIRouter()

DB_PATH = os.getenv("DB_PATH", "src/data/auth.db")

class UserProfile(BaseModel):
    user_id: int
    username: str
    email: str
    created_at: float

class ReadingProgress(BaseModel):
    chapter_id: str
    chapter_title: str
    progress_percent: float
    last_accessed: float

class UpdatePreferencesRequest(BaseModel):
    preferred_language: Optional[str] = None
    font_size: Optional[str] = None
    theme: Optional[str] = None

class UserPreferences(BaseModel):
    preferred_language: Optional[str] = None
    font_size: Optional[str] = None
    theme: Optional[str] = None

def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    conn = get_db()
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS user_preferences (
            user_id INTEGER PRIMARY KEY,
            preferred_language TEXT DEFAULT 'en',
            font_size TEXT DEFAULT 'medium',
            theme TEXT DEFAULT 'system',
            updated_at REAL DEFAULT (strftime('%s', 'now'))
        )
    ''')
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS reading_progress (
            user_id INTEGER,
            chapter_id TEXT,
            chapter_title TEXT,
            progress_percent REAL DEFAULT 0,
            last_accessed REAL DEFAULT (strftime('%s', 'now')),
            PRIMARY KEY (user_id, chapter_id)
        )
    ''')
    conn.commit()
    conn.close()

try:
    init_db()
except:
    pass

@router.get("/profile/{user_id}")
async def get_profile(user_id: int):
    """Get user profile information."""
    conn = get_db()
    cursor = conn.cursor()

    cursor.execute("SELECT id, username, email, created_at FROM users WHERE id = ?", (user_id,))
    user = cursor.fetchone()
    conn.close()

    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return {
        "user_id": user['id'],
        "username": user['username'],
        "email": user['email'],
        "created_at": user['created_at']
    }

@router.get("/progress/{user_id}")
async def get_reading_progress(user_id: int):
    """Get user's reading progress for all chapters."""
    conn = get_db()
    cursor = conn.cursor()

    cursor.execute(
        "SELECT chapter_id, chapter_title, progress_percent, last_accessed FROM reading_progress WHERE user_id = ?",
        (user_id,)
    )
    progress = cursor.fetchall()
    conn.close()

    return {"progress": [dict(row) for row in progress]}

@router.post("/progress/{user_id}")
async def update_reading_progress(user_id: int, progress: ReadingProgress):
    """Update reading progress for a chapter."""
    conn = get_db()
    cursor = conn.cursor()

    cursor.execute('''
        INSERT OR REPLACE INTO reading_progress (user_id, chapter_id, chapter_title, progress_percent, last_accessed)
        VALUES (?, ?, ?, ?, ?)
    ''', (user_id, progress.chapter_id, progress.chapter_title, progress.progress_percent, progress.last_accessed))
    conn.commit()
    conn.close()

    return {"success": True, "message": "Progress updated"}

@router.get("/preferences/{user_id}")
async def get_preferences(user_id: int):
    """Get user preferences."""
    conn = get_db()
    cursor = conn.cursor()

    cursor.execute("SELECT preferred_language, font_size, theme FROM user_preferences WHERE user_id = ?", (user_id,))
    prefs = cursor.fetchone()
    conn.close()

    if not prefs:
        return {"preferred_language": "en", "font_size": "medium", "theme": "system"}

    return dict(prefs)

@router.put("/preferences/{user_id}")
async def update_preferences(user_id: int, request: UpdatePreferencesRequest):
    """Update user preferences."""
    conn = get_db()
    cursor = conn.cursor()

    set_clauses = []
    values = []

    if request.preferred_language:
        set_clauses.append("preferred_language = ?")
        values.append(request.preferred_language)
    if request.font_size:
        set_clauses.append("font_size = ?")
        values.append(request.font_size)
    if request.theme:
        set_clauses.append("theme = ?")
        values.append(request.theme)

    if set_clauses:
        set_clauses.append("updated_at = ?")
        values.append(__import__('time').time())
        values.append(user_id)

        cursor.execute(
            f"INSERT OR REPLACE INTO user_preferences (user_id, {', '.join(['preferred_language', 'font_size', 'theme'][:len(set_clauses)-1])}, updated_at) VALUES (?, {', '.join(['?' for _ in set_clauses[:-1]])}, ?)",
            values
        )
        conn.commit()

    conn.close()
    return {"success": True, "message": "Preferences updated"}
