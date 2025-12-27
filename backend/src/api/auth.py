# Authentication API endpoints

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fastapi import APIRouter, Request, HTTPException, status, Response
from fastapi.responses import JSONResponse
from datetime import timedelta
from os import getenv

from schemas.auth import (
    SignupRequest, SigninRequest, AuthResponse, UserProfileResponse, ErrorResponse
)
from services.auth_service import create_user, authenticate_user, get_user_by_id
from middleware.auth import create_session, delete_session, require_session, SESSION_COOKIE_NAME
from middleware.auth import get_session

router = APIRouter(prefix="/auth", tags=["Authentication"])

# Cookie settings from environment
COOKIE_MAX_AGE = int(getenv("COOKIE_MAX_AGE", "2592000"))  # 30 days
COOKIE_SECURE = getenv("COOKIE_SECURE", "false").lower() == "true"
COOKIE_SAME_SITE = getenv("COOKIE_SAME_SITE", "lax")


@router.post("/signup", response_model=AuthResponse, responses={
    400: {"model": ErrorResponse, "description": "Validation error"},
    409: {"model": ErrorResponse, "description": "Email already exists"}
})
async def signup(request: SignupRequest, http_request: Request):
    """
    Create a new user account with profile information.
    """
    try:
        user_id, user_data, profile_data = create_user(
            email=request.email,
            password=request.password,
            software_level=request.software_level,
            hardware_experience=request.hardware_experience,
            learning_depth=request.learning_depth
        )

        # Create session
        ip_address = http_request.client.host if http_request.client else None
        user_agent = http_request.headers.get("user-agent")

        token, expires_at = create_session(
            user_id=user_id,
            expires_days=COOKIE_MAX_AGE // (60 * 60 * 24),
            ip_address=ip_address,
            user_agent=user_agent
        )

        response = JSONResponse(
            content=AuthResponse(
                success=True,
                user=user_data,
                message="Account created successfully"
            ).model_dump(mode='json')
        )

        # Set HttpOnly cookie
        response.set_cookie(
            key=SESSION_COOKIE_NAME,
            value=token,
            max_age=COOKIE_MAX_AGE,
            httponly=True,
            secure=COOKIE_SECURE,
            samesite=COOKIE_SAME_SITE,
            path="/"
        )

        return response

    except ValueError as e:
        if "already exists" in str(e).lower():
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=str(e)
            )
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Signup failed: {str(e)}"
        )


@router.post("/signin", response_model=AuthResponse, responses={
    401: {"model": ErrorResponse, "description": "Invalid credentials"}
})
async def signin(request: SigninRequest, response: Response, http_request: Request):
    """
    Sign in with email and password. Returns HttpOnly session cookie.
    """
    result = authenticate_user(email=request.email, password=request.password)

    if not result:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password",
            headers={"WWW-Authenticate": "Cookie"}
        )

    user_id, user_data, profile_data = result

    # Create session
    ip_address = http_request.client.host if http_request.client else None
    user_agent = http_request.headers.get("user-agent")

    token, expires_at = create_session(
        user_id=user_id,
        expires_days=COOKIE_MAX_AGE // (60 * 60 * 24),
        ip_address=ip_address,
        user_agent=user_agent
    )

    response = JSONResponse(
        content=AuthResponse(
            success=True,
            user=user_data,
            message="Sign in successful"
        ).model_dump(mode='json')
    )

    # Set HttpOnly cookie
    response.set_cookie(
        key=SESSION_COOKIE_NAME,
        value=token,
        max_age=COOKIE_MAX_AGE,
        httponly=True,
        secure=COOKIE_SECURE,
        samesite=COOKIE_SAME_SITE,
        path="/"
    )

    return response


@router.get("/me", response_model=UserProfileResponse, responses={
    401: {"model": ErrorResponse, "description": "Not authenticated"}
})
async def get_me(http_request: Request):
    """
    Get current user profile. Requires authentication.
    """
    session = await require_session(http_request)

    result = get_user_by_id(session.user_id)

    if not result:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )

    user_data, profile_data = result

    return UserProfileResponse(
        user=user_data,
        profile=profile_data
    )


@router.post("/signout")
async def signout(response: Response, http_request: Request):
    """
    Sign out and clear session cookie.
    """
    # Get session to delete it from database
    cookie_value = http_request.cookies.get(SESSION_COOKIE_NAME)

    if cookie_value:
        delete_session(cookie_value)

    # Clear cookie
    response = JSONResponse(
        content={"success": True, "message": "Signed out successfully"}
    )
    response.delete_cookie(
        key=SESSION_COOKIE_NAME,
        path="/"
    )

    return response


@router.get("/status")
async def auth_status(http_request: Request):
    """
    Check authentication status. Returns user info if authenticated, null if not.
    """
    session = await get_session(http_request)

    if not session:
        return {"authenticated": False, "user": None}

    result = get_user_by_id(session.user_id)

    if not result:
        return {"authenticated": False, "user": None}

    user_data, profile_data = result

    return {
        "authenticated": True,
        "user": {
            "id": user_data["id"],
            "email": user_data["email"],
            "profile": profile_data
        }
    }
