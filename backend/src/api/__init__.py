# API module initialization
from .auth import router as auth_router
from .profile import router as profile_router
from .personalization import router as personalization_router
from .chat import router as chat_router

__all__ = ["auth_router", "profile_router", "personalization_router", "chat_router"]
