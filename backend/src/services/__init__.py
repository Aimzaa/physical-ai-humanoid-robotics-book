# Services module initialization
from .auth_service import (
    hash_password, verify_password,
    create_user, authenticate_user, get_user_by_id, update_user_profile
)
from .personalization_service import (
    UserPreferences, get_user_preferences, get_default_preferences,
    update_personalization_state, toggle_favorite_chapter, get_applied_sections
)
from .chatbot_service import (
    UserContext, build_system_prompt, get_user_context_for_response,
    chat_with_context
)

__all__ = [
    "hash_password", "verify_password",
    "create_user", "authenticate_user", "get_user_by_id", "update_user_profile",
    "UserPreferences", "get_user_preferences", "get_default_preferences",
    "update_personalization_state", "toggle_favorite_chapter", "get_applied_sections",
    "UserContext", "build_system_prompt", "get_user_context_for_response",
    "chat_with_context"
]
