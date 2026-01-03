import sqlite3
import os

DB_PATH = os.getenv("DB_PATH", "src/data/auth.db")

def get_db():
    """Get database connection."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    """Initialize database tables."""
    conn = get_db()
    cursor = conn.cursor()

    # Users table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT UNIQUE NOT NULL,
            email TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL,
            created_at REAL DEFAULT (strftime('%s', 'now')),
            updated_at REAL DEFAULT (strftime('%s', 'now'))
        )
    ''')

    # User preferences table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS user_preferences (
            user_id INTEGER PRIMARY KEY,
            preferred_language TEXT DEFAULT 'en',
            font_size TEXT DEFAULT 'medium',
            theme TEXT DEFAULT 'system',
            updated_at REAL DEFAULT (strftime('%s', 'now'))
        )
    ''')

    # Reading progress table
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
    print(f"Database initialized at: {DB_PATH}")

if __name__ == "__main__":
    init_db()
