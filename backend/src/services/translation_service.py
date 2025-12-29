# Translation Service - Using MyMemory API (Free, no API key required)
# Feature: Chapter-Level Urdu Translation for Logged-in Users
# Date: 2025-12-28

import sys
import os
import json
import hashlib
import uuid
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import logging
import httpx

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from db import get_connection

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
TRANSLATION_CACHE_TTL_HOURS = int(os.getenv("TRANSLATION_CACHE_TTL_HOURS", "48"))


# =============================================================================
# Free Translation API (MyMemory - https://mymemory.translated.net/doc/spec.php)
# =============================================================================

async def translate_to_urdu(text: str) -> str:
    """
    Translate text to Urdu using MyMemory API (free, no API key needed).

    MyMemory API:
    - Free tier: 5000 chars/day
    - No API key required
    - Supports Urdu (ur)
    """
    if not text or not text.strip():
        return text

    # Clean the text - remove extra whitespace
    text = text.strip()

    # For long content, translate in chunks
    if len(text) > 5000:
        # Split into paragraphs and translate separately
        paragraphs = text.split('\n\n')
        translated_paragraphs = []
        for para in paragraphs:
            if para.strip():
                translated = await translate_chunk(para)
                translated_paragraphs.append(translated)
            else:
                translated_paragraphs.append('')
        return '\n\n'.join(translated_paragraphs)
    else:
        return await translate_chunk(text)


async def translate_chunk(text: str) -> str:
    """Translate a single chunk of text using MyMemory API."""
    url = "https://api.mymemory.translated.net/get"

    params = {
        "q": text,
        "langpair": "en|ur"  # English to Urdu
    }

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.get(url, params=params)
            data = response.json()

            if data.get("responseStatus") == 200:
                translated_text = data["responseData"]["translatedText"]
                # Replace newlines back to double newlines
                translated_text = translated_text.replace('\n', '\n\n')
                return translated_text
            else:
                # Fallback: return original with translation note
                logger.warning(f"MyMemory API error: {data.get('responseDetails')}")
                return f"[Translation unavailable: {data.get('responseDetails', 'Unknown error')}]\n\n{text}"

    except Exception as e:
        logger.error(f"Translation error: {e}")
        return f"[Translation error: {str(e)}]\n\n{text}"


# =============================================================================
# Simple Fallback Translation (if API fails)
# =============================================================================

def simple_urdu_translate(text: str) -> str:
    """
    Simple fallback translation using basic mapping.
    This is used when the free API is unavailable.
    """
    # Common robotics/technical terms mapping
    term_mapping = {
        "ROS": "ROS (روبٹ آپریٹنگ سسٹم)",
        "robot": "روبوٹ",
        "robots": "روبوٹس",
        "robotics": "روبوٹکس",
        "AI": "AI (مصنوعی ذہانت)",
        "machine learning": "مشین لرننگ",
        "Python": "پائتھن",
        "C++": "C++",
        "node": "نوڈ",
        "nodes": "نوڈس",
        "topic": "ٹاپک",
        "service": "سروس",
        "publisher": "پبلشر",
        "subscriber": "سبسکرائبر",
        "message": "پیغام",
        "interface": "انٹرفیس",
        "communication": "مواصلات",
        "sensor": "سینسر",
        "actuator": "ایکچویٹر",
        "motor": "موٹر",
        "camera": "کیمرہ",
        "laser": "لیزر",
        "navigation": "نیویگیشن",
        "planning": "پلاننگ",
        "control": "کنٹرول",
        "hardware": "ہارڈویئر",
        "software": "سافٹویئر",
        "code": "کوڈ",
        "program": "پروگرام",
        "algorithm": "الگورتھم",
        "data": "ڈیٹا",
        "image": "تصویر",
        "video": "ویڈیو",
        "audio": "آڈیو",
        "system": "سسٹم",
        "network": "نیٹورک",
        "computer": "کمپیوٹر",
        "processing": "پروسیسنگ",
        "learning": "سیکھنا",
        "training": "ٹریننگ",
        "model": "ماڈل",
        "neural": "نیورل",
        "deep": "ڈیپ",
        "autonomous": "آٹونومس",
        "humanoid": "ہیومینوئڈ",
        "bipedal": "بائیپیڈل",
        "walking": "چلنا",
        "movement": "حرکت",
        "balance": "توازن",
        "control": "کنٹرول",
        "joint": "جوائنٹ",
        "arm": "بازو",
        "leg": "ٹانگ",
        "hand": "ہاتھ",
        "grip": "گرिप",
        "grasping": "گراسپنگ",
        "manipulation": "منیپولیشن",
    }

    result = text
    for eng, urdu in term_mapping.items():
        result = result.replace(eng, urdu)

    return result


# =============================================================================
# Translation Cache Model
# =============================================================================

class TranslationCache:
    """Handle translation caching with TTL."""

    @staticmethod
    def generate_content_hash(content: str) -> str:
        """Generate SHA256 hash of content for cache invalidation."""
        return hashlib.sha256(content.encode('utf-8')).hexdigest()

    @staticmethod
    async def save(
        user_id: str,
        chapter_id: str,
        content_hash: str,
        translated_content: str,
        ttl_hours: int = TRANSLATION_CACHE_TTL_HOURS
    ) -> Dict[str, Any]:
        """Save translation to cache."""
        conn = get_connection()
        cursor = conn.cursor()

        cache_id = str(uuid.uuid4())
        now = datetime.now()
        expires_at = now + timedelta(hours=ttl_hours)

        try:
            cursor.execute("""
                INSERT OR REPLACE INTO translation_cache
                (id, user_id, chapter_id, original_content_hash, translated_content,
                 created_at, expires_at)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            """, (
                cache_id,
                user_id,
                chapter_id,
                content_hash,
                translated_content,
                now.isoformat(),
                expires_at.isoformat(),
            ))
            conn.commit()

            return {
                "success": True,
                "cache_id": cache_id,
                "expires_at": expires_at.isoformat(),
                "ttl_seconds": ttl_hours * 3600,
            }
        except Exception as e:
            logger.error(f"Failed to save translation cache: {e}")
            raise
        finally:
            conn.close()

    @staticmethod
    async def lookup(
        user_id: str,
        chapter_id: str,
        content_hash: str
    ) -> Optional[Dict[str, Any]]:
        """Lookup translation from cache."""
        conn = get_connection()
        cursor = conn.cursor()

        try:
            cursor.execute("""
                SELECT translated_content, created_at, expires_at
                FROM translation_cache
                WHERE user_id = ? AND chapter_id = ? AND original_content_hash = ?
            """, (user_id, chapter_id, content_hash))

            row = cursor.fetchone()

            if not row:
                return None

            expires_at = datetime.fromisoformat(row["expires_at"])

            # Check if expired
            if expires_at < datetime.now():
                return None

            # Calculate remaining TTL
            ttl_remaining = int((expires_at - datetime.now()).total_seconds())

            return {
                "translated_content": row["translated_content"],
                "cached_at": row["created_at"],
                "expires_at": row["expires_at"],
                "ttl_remaining": ttl_remaining,
            }
        finally:
            conn.close()

    @staticmethod
    async def clear(user_id: str, chapter_id: str) -> bool:
        """Clear cached translation for a user/chapter."""
        conn = get_connection()
        cursor = conn.cursor()

        try:
            cursor.execute(
                "DELETE FROM translation_cache WHERE user_id = ? AND chapter_id = ?",
                (user_id, chapter_id)
            )
            conn.commit()
            return cursor.rowcount > 0
        finally:
            conn.close()


# =============================================================================
# Translation Service
# =============================================================================

class TranslationService:
    """Service for translating chapter content to Urdu."""

    def __init__(self):
        self.cache = TranslationCache()

    async def translate(
        self,
        user_id: str,
        chapter_id: str,
        chapter_title: str,
        content: str
    ) -> Dict[str, Any]:
        """
        Translate chapter content to Urdu.

        Returns:
            Dict with keys: translated_content, cached, ttl_seconds
        """
        # Generate content hash
        content_hash = self.cache.generate_content_hash(content)

        # Check cache first
        cached = await self.cache.lookup(user_id, chapter_id, content_hash)
        if cached:
            logger.info(f"Cache hit for user {user_id}, chapter {chapter_id}")
            return {
                "translated_content": cached["translated_content"],
                "cached": True,
                "ttl_seconds": cached["ttl_remaining"],
                "cached_at": cached["cached_at"],
            }

        # Perform translation via MyMemory API
        logger.info(f"Translating chapter {chapter_id} for user {user_id}")

        try:
            translated_content = await translate_to_urdu(content)
        except Exception as e:
            logger.error(f"MyMemory API failed, using fallback: {e}")
            translated_content = simple_urdu_translate(content)

        # Save to cache
        await self.cache.save(
            user_id=user_id,
            chapter_id=chapter_id,
            content_hash=content_hash,
            translated_content=translated_content,
        )

        return {
            "translated_content": translated_content,
            "cached": False,
            "ttl_seconds": TRANSLATION_CACHE_TTL_HOURS * 3600,
        }

    async def get_cache_status(
        self,
        user_id: str,
        chapter_id: str
    ) -> Dict[str, Any]:
        """Get cache status for a user/chapter."""
        conn = get_connection()
        cursor = conn.cursor()

        try:
            cursor.execute("""
                SELECT created_at, expires_at FROM translation_cache
                WHERE user_id = ? AND chapter_id = ?
                ORDER BY created_at DESC LIMIT 1
            """, (user_id, chapter_id))

            row = cursor.fetchone()

            if not row:
                return {"has_translation": False}

            expires_at = datetime.fromisoformat(row["expires_at"])

            if expires_at < datetime.now():
                return {"has_translation": False}

            return {
                "has_translation": True,
                "cached_at": row["created_at"],
                "ttl_remaining": int((expires_at - datetime.now()).total_seconds()),
                "language": "ur",
            }
        finally:
            conn.close()

    async def clear_cache(
        self,
        user_id: str,
        chapter_id: str
    ) -> bool:
        """Clear cached translation."""
        return await self.cache.clear(user_id, chapter_id)
