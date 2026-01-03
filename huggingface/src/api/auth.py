from fastapi import APIRouter, HTTPException, Depends, status
from pydantic import BaseModel
from typing import Optional
import bcrypt
import jwt
import os
import sqlite3
import time

router = APIRouter()

# JWT Configuration
JWT_SECRET = os.getenv("JWT_SECRET", "your-super-secret-jwt-key-change-in-production")
JWT_ALGORITHM = "HS256"
TOKEN_EXPIRE_HOURS = 24

# Database path
DB_PATH = os.getenv("DB_PATH", "src/data/auth.db")

class SignupRequest(BaseModel):
    username: str
    email: str
    password: str

class LoginRequest(BaseModel):
    username: str
    password: str

class AuthResponse(BaseModel):
    success: bool
    message: str
    token: Optional[str] = None
    user_id: Optional[int] = None

def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    conn = get_db()
    cursor = conn.cursor()
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
    conn.commit()
    conn.close()

# Initialize database on module load
try:
    init_db()
except Exception as e:
    print(f"Warning: Could not initialize database: {e}")

@router.post("/signup", response_model=AuthResponse)
async def signup(request: SignupRequest):
    """Register a new user."""
    conn = get_db()
    cursor = conn.cursor()

    try:
        # Check if user exists
        cursor.execute("SELECT id FROM users WHERE username = ? OR email = ?",
                      (request.username, request.email))
        if cursor.fetchone():
            return AuthResponse(success=False, message="Username or email already exists")

        # Hash password
        password_hash = bcrypt.hashpw(request.password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

        # Insert user
        cursor.execute(
            "INSERT INTO users (username, email, password_hash) VALUES (?, ?, ?)",
            (request.username, request.email, password_hash)
        )
        conn.commit()
        user_id = cursor.lastrowid

        # Generate token
        token = generate_token(user_id, request.username)

        return AuthResponse(
            success=True,
            message="User registered successfully",
            token=token,
            user_id=user_id
        )
    except Exception as e:
        return AuthResponse(success=False, message=f"Registration failed: {str(e)}")
    finally:
        conn.close()

@router.post("/login", response_model=AuthResponse)
async def login(request: LoginRequest):
    """Login with username and password."""
    conn = get_db()
    cursor = conn.cursor()

    try:
        # Find user
        cursor.execute("SELECT id, username, password_hash FROM users WHERE username = ?",
                      (request.username,))
        user = cursor.fetchone()

        if not user:
            return AuthResponse(success=False, message="Invalid username or password")

        # Verify password
        if not bcrypt.checkpw(request.password.encode('utf-8'),
                              user['password_hash'].encode('utf-8')):
            return AuthResponse(success=False, message="Invalid username or password")

        # Generate token
        token = generate_token(user['id'], user['username'])

        return AuthResponse(
            success=True,
            message="Login successful",
            token=token,
            user_id=user['id']
        )
    except Exception as e:
        return AuthResponse(success=False, message=f"Login failed: {str(e)}")
    finally:
        conn.close()

@router.post("/logout")
async def logout():
    """Logout user (client-side token removal)."""
    return {"success": True, "message": "Logged out successfully"}

def generate_token(user_id: int, username: str) -> str:
    """Generate JWT token."""
    payload = {
        "user_id": user_id,
        "username": username,
        "exp": time.time() + (TOKEN_EXPIRE_HOURS * 3600),
        "iat": time.time()
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)

def verify_token(token: str) -> dict:
    """Verify JWT token and return payload."""
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")
