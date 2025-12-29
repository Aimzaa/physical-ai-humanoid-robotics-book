import sqlite3
import os

# Database path
DB_DIR = "E:/code/Hackathon-Book/humanoid-robotics-Book/backend/src/data"
DB_PATH = os.path.join(DB_DIR, "auth.db")

# Create directory if needed
os.makedirs(DB_DIR, exist_ok=True)

# Connect and create tables
conn = sqlite3.connect(DB_PATH)
cursor = conn.cursor()

# Drop old tables
cursor.execute("DROP TABLE IF EXISTS translation_cache")
cursor.execute("DROP TABLE IF EXISTS translation_request")
cursor.execute("DROP TABLE IF EXISTS translation_preference")
print("Old tables dropped")

# Create translation_cache table
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
print("translation_cache table created")

# Create indexes
cursor.execute("""
    CREATE INDEX IF NOT EXISTS idx_translation_cache_user_chapter
    ON translation_cache(user_id, chapter_id)
""")
cursor.execute("""
    CREATE INDEX IF NOT EXISTS idx_translation_cache_expires
    ON translation_cache(expires_at)
""")
print("Indexes created")

conn.commit()
conn.close()
print("Done! Translation tables created at:", DB_PATH)
