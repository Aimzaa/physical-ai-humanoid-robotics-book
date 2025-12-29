# Database migration: Translation tables
# Feature: Chapter-Level Urdu Translation for Logged-in Users
# Date: 2025-12-28

import sqlite3
from pathlib import Path
from datetime import datetime, timedelta
import hashlib
import uuid

# Database path (same as main database)
DB_DIR = Path(__file__).parent.parent.parent / "data"
DB_PATH = DB_DIR / "auth.db"


def get_connection():
    """Get database connection."""
    conn = sqlite3.connect(str(DB_PATH))
    conn.row_factory = sqlite3.Row
    return conn


def create_translation_tables():
    """Create translation-related tables."""
    conn = get_connection()
    cursor = conn.cursor()

    # Translation cache table - stores per-user cached translations
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS translation_cache (
            id TEXT PRIMARY KEY,
            user_id TEXT NOT NULL,
            chapter_id TEXT NOT NULL,
            original_content_hash TEXT NOT NULL,
            translated_content TEXT NOT NULL,
            created_at TEXT NOT NULL,
            expires_at TEXT NOT NULL,
            UNIQUE(user_id, chapter_id)
        )
    """)

    # Translation request tracking (for analytics and rate limiting)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS translation_request (
            id TEXT PRIMARY KEY,
            user_id TEXT NOT NULL,
            chapter_id TEXT NOT NULL,
            status TEXT NOT NULL DEFAULT 'pending',
            duration_ms INTEGER,
            error_message TEXT,
            created_at TEXT NOT NULL,
            FOREIGN KEY (user_id) REFERENCES user(id)
        )
    """)

    # Translation preferences table (user settings)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS translation_preference (
            user_id TEXT PRIMARY KEY,
            auto_translate INTEGER DEFAULT 0,
            remember_preference INTEGER DEFAULT 1,
            preferred_depth TEXT DEFAULT 'Both',
            created_at TEXT NOT NULL,
            updated_at TEXT,
            FOREIGN KEY (user_id) REFERENCES user(id)
        )
    """)

    # Create indexes for performance
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_translation_cache_user_chapter
        ON translation_cache(user_id, chapter_id)
    """)
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_translation_cache_expires
        ON translation_cache(expires_at)
    """)
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_translation_request_user
        ON translation_request(user_id, created_at)
    """)

    conn.commit()
    conn.close()

    print(f"Translation tables created at: {DB_PATH}")
    print("Tables: translation_cache, translation_request, translation_preference")
    print("Indexes: idx_translation_cache_user_chapter, idx_translation_cache_expires, idx_translation_request_user")


def drop_translation_tables():
    """Drop translation tables (use with caution!)."""
    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute("DROP TABLE IF EXISTS translation_preference")
    cursor.execute("DROP TABLE IF EXISTS translation_request")
    cursor.execute("DROP TABLE IF EXISTS translation_cache")

    conn.commit()
    conn.close()

    print("Translation tables dropped.")


def add_translation_tables():
    """Add translation tables to existing database."""
    create_translation_tables()


def cleanup_expired_translations() -> int:
    """Remove all expired translation cache entries. Call periodically."""
    conn = get_connection()
    cursor = conn.cursor()

    now = datetime.now().isoformat()
    cursor.execute("DELETE FROM translation_cache WHERE expires_at < ?", (now,))

    deleted = cursor.rowcount
    conn.commit()
    conn.close()

    return deleted


def generate_content_hash(content: str) -> str:
    """Generate SHA256 hash of content for cache invalidation."""
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == "--drop":
            drop_translation_tables()
        elif sys.argv[1] == "--cleanup":
            count = cleanup_expired_translations()
            print(f"Cleaned up {count} expired translations")
        elif sys.argv[1] == "--add":
            add_translation_tables()
        else:
            print("Usage: python 001_translation_tables.py [--drop|--cleanup|--add]")
    else:
        add_translation_tables()
