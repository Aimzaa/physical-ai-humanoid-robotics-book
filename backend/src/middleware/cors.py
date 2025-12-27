# CORS middleware configuration

from fastapi.middleware.cors import CORSMiddleware
from os import getenv

# Get frontend URL from environment
FRONTEND_URL = getenv("FRONTEND_URL", "http://localhost:3000")

# Allow additional origins for development
ALLOWED_ORIGINS = [
    FRONTEND_URL,
    "http://localhost:3000",  # Docusaurus dev
    "http://localhost:3001",  # Docusaurus alternate port
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
    "http://localhost:5173",  # Vite dev
    "http://localhost:5174",  # Vite alternate
]


def configure_cors(app):
    """Configure CORS for the FastAPI application - MUST be called before routes."""
    app.add_middleware(
        CORSMiddleware,
        allow_origins=ALLOWED_ORIGINS,
        allow_credentials=True,  # Required for HttpOnly cookies
        allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
        allow_headers=["*"],
        expose_headers=["*"],
    )
