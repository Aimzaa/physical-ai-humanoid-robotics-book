# Database initialization for Personalized Book Platform

import os
import sqlite3
from datetime import datetime, timedelta
from pathlib import Path

# Database path
DB_DIR = Path(__file__).parent.parent / "data"
DB_PATH = DB_DIR / "auth.db"

# Create data directory if it doesn't exist
DB_DIR.mkdir(parents=True, exist_ok=True)


def get_connection():
    """Get database connection."""
    conn = sqlite3.connect(str(DB_PATH))
    conn.row_factory = sqlite3.Row
    return conn


def create_tables():
    """Create all required tables for authentication and profiles."""
    conn = get_connection()
    cursor = conn.cursor()

    # Users table (managed by Better Auth internally)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS user (
            id TEXT PRIMARY KEY,
            email TEXT NOT NULL UNIQUE,
            password_hash TEXT NOT NULL,
            email_verified INTEGER DEFAULT 0,
            created_at TEXT NOT NULL,
            last_login TEXT
        )
    """)

    # User sessions table (managed by Better Auth)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS session (
            id TEXT PRIMARY KEY,
            user_id TEXT NOT NULL,
            token TEXT NOT NULL UNIQUE,
            expires_at TEXT NOT NULL,
            ip_address TEXT,
            user_agent TEXT,
            created_at TEXT NOT NULL,
            FOREIGN KEY (user_id) REFERENCES user(id)
        )
    """)

    # Extended user profile table (custom - for personalization)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS user_profile (
            id TEXT PRIMARY KEY,
            software_level TEXT NOT NULL DEFAULT 'Beginner',
            hardware_experience TEXT NOT NULL DEFAULT 'None',
            learning_depth TEXT NOT NULL DEFAULT 'Both',
            personalization_enabled INTEGER DEFAULT 1,
            favorite_chapters TEXT,  -- JSON array of chapter IDs
            last_personalized_chapter TEXT,
            created_at TEXT NOT NULL,
            updated_at TEXT,
            FOREIGN KEY (id) REFERENCES user(id)
        )
    """)

    # Create indexes for performance
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_session_user_id ON session(user_id)
    """)
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_user_email ON user(email)
    """)
    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_session_expires ON session(expires_at)
    """)

    conn.commit()
    conn.close()

    # Initialize translation tables via migration
    try:
        import importlib
        translation_migration = importlib.import_module('db_migrations.translation_tables')
        translation_migration.add_translation_tables()
    except (ImportError, AttributeError) as e:
        print(f"Warning: Could not initialize translation tables: {e}")

    print(f"Database initialized at: {DB_PATH}")
    print("Tables created: user, session, user_profile")
    print("Indexes created: idx_session_user_id, idx_user_email, idx_session_expires")


def drop_tables():
    """Drop all tables (use with caution!)."""
    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute("DROP TABLE IF EXISTS user_profile")
    cursor.execute("DROP TABLE IF EXISTS session")
    cursor.execute("DROP TABLE IF EXISTS user")

    conn.commit()
    conn.close()

    print("All tables dropped.")


def init_db():
    """Initialize the database - create tables if they don't exist."""
    create_tables()


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--drop":
        drop_tables()
    init_db()
