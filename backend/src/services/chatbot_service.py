# Chatbot service - handles RAG with user context

import os
import json
import logging
from typing import Optional, List, Dict, Any
from datetime import datetime
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from db import get_connection
from services.personalization_service import get_user_preferences, UserPreferences

from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)

# Get API key
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")


class UserContext:
    """User context for tailoring chatbot responses."""

    def __init__(self, preferences: UserPreferences):
        self.preferences = preferences

    def get_system_prompt_additions(self) -> str:
        """
        Get system prompt additions based on user profile.

        Returns a string to prepend to the system prompt.
        """
        additions = []

        # Software level instructions
        level_instruct = {
            "Beginner": (
                "Use simple analogies and everyday examples. "
                "Avoid jargon. Explain concepts from first principles. "
                "Use analogies like 'like a recipe' or 'like following directions'."
            ),
            "Intermediate": (
                "Use standard technical terminology but explain briefly. "
                "Assume basic programming knowledge. "
                "Can reference fundamental concepts without full explanation."
            ),
            "Advanced": (
                "Use precise technical terminology freely. "
                "Reference advanced patterns and optimizations. "
                "Assume deep domain knowledge."
            )
        }
        additions.append(f"User's software level: {self.preferences.software_level}. {level_instruct.get(self.preferences.software_level, '')}")

        # Hardware experience instructions
        hardware_instruct = {
            "None": "Include physical-world analogies. Explain hardware concepts in software terms.",
            "Basic": "Can reference hardware concepts with brief explanations.",
            "Hands-on": "Include practical implementation details. Reference real hardware constraints."
        }
        additions.append(f"User's hardware experience: {self.preferences.hardware_experience}. {hardware_instruct.get(self.preferences.hardware_experience, '')}")

        # Learning depth instructions
        depth_instruct = {
            "Conceptual": "Focus on theory, principles, and understanding 'why'. Less code, more concepts.",
            "Practical": "Focus on implementation, code examples, and hands-on steps.",
            "Both": "Balance theory and practice. Include concepts with concrete examples."
        }
        additions.append(f"User's learning depth preference: {self.preferences.learning_depth}. {depth_instruct.get(self.preferences.learning_depth, '')}")

        return "\n".join(additions)


def build_system_prompt(base_prompt: str, user_id: str = None, is_guest: bool = True) -> str:
    """
    Build system prompt with user context.
    """
    if is_guest or not user_id:
        guest_context = (
            "\n\nGUEST USER CONTEXT:\n"
            "- Software level: Intermediate (assume comfortable with programming)\n"
            "- Hardware experience: Basic (some theoretical knowledge)\n"
            "- Learning depth: Both (balance of theory and practice)"
        )
        return base_prompt + guest_context

    # Get user preferences
    prefs = get_user_preferences(user_id)

    if not prefs:
        guest_context = (
            "\n\nUSER CONTEXT (default):\n"
            "- Software level: Beginner\n"
            "- Hardware experience: None\n"
            "- Learning depth: Both"
        )
        return base_prompt + guest_context

    # Build context from user profile
    context = UserContext(prefs)
    context_str = context.get_system_prompt_additions()

    full_prompt = f"{base_prompt}\n\nUSER CONTEXT:\n{context_str}"

    return full_prompt


def get_user_context_for_response(user_id: str = None, is_guest: bool = True) -> Dict[str, str]:
    """
    Get user context as a dictionary for response metadata.
    """
    if is_guest or not user_id:
        return {
            "software_level": "Intermediate",
            "hardware_experience": "Basic",
            "learning_depth": "Both"
        }

    prefs = get_user_preferences(user_id)

    if not prefs:
        return {
            "software_level": "Beginner",
            "hardware_experience": "None",
            "learning_depth": "Both"
        }

    return {
        "software_level": prefs.software_level,
        "hardware_experience": prefs.hardware_experience,
        "learning_depth": prefs.learning_depth
    }


async def chat_with_context(
    query: str,
    conversation_history: List[Dict[str, str]] = None,
    user_id: str = None,
    model: str = "openai/gpt-3.5-turbo"
) -> Dict[str, Any]:
    """
    Process a chat query with user context awareness.
    """
    if not OPENROUTER_API_KEY:
        return {
            "response": "Chat service is not configured. Please set OPENROUTER_API_KEY in environment.",
            "sources": [],
            "tokens_used": 0,
            "user_context_applied": None,
            "success": False
        }

    try:
        import openai
    except ImportError:
        return {
            "response": "OpenAI SDK not installed",
            "sources": [],
            "tokens_used": 0,
            "user_context_applied": None,
            "success": False
        }

    openai.api_key = OPENROUTER_API_KEY
    openai.base_url = "https://openrouter.ai/api/v1/"

    # Base system prompt
    base_prompt = """You are an assistant for the Physical AI & Humanoid Robotics Book.
Provide accurate, helpful answers based on the book content.
Cite sources when possible. Be concise but thorough."""

    # Build context-aware system prompt
    is_guest = not user_id
    system_prompt = build_system_prompt(base_prompt, user_id, is_guest)

    # Build messages
    messages = [{"role": "system", "content": system_prompt}]

    # Add conversation history
    if conversation_history:
        for msg in conversation_history:
            messages.append({"role": msg["role"], "content": msg["content"]})

    # Add user query
    messages.append({"role": "user", "content": query})

    try:
        # Call OpenRouter
        response = openai.chat.completions.create(
            model=model.replace("openai/", ""),
            messages=messages,
            temperature=0.3,
            max_tokens=1000
        )

        answer = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0

        # Get user context for response
        user_context = get_user_context_for_response(user_id, is_guest)

        return {
            "response": answer,
            "sources": [],
            "tokens_used": tokens_used,
            "user_context_applied": user_context,
            "success": True
        }

    except Exception as e:
        logger.error(f"Chat error: {e}")
        return {
            "response": f"Sorry, I encountered an error: {str(e)}",
            "sources": [],
            "tokens_used": 0,
            "user_context_applied": None,
            "success": False
        }
