# Authentication API Contract

## User Registration
- **Endpoint**: `POST /api/v1/auth/register`
- **Description**: Register a new user with email, password, and background information
- **Request Body**:
  ```json
  {
    "email": "string (required, valid email)",
    "password": "string (required, min 8 chars)",
    "name": "string (optional)",
    "software_background": "string (optional, enum: beginner/intermediate/advanced)",
    "hardware_background": "string (optional, enum: beginner/intermediate/advanced)"
  }
  ```
- **Response**:
  ```json
  {
    "id": "string (user UUID)",
    "email": "string",
    "name": "string",
    "created_at": "datetime",
    "updated_at": "datetime",
    "software_background": "string",
    "hardware_background": "string"
  }
  ```
- **Status Codes**:
  - 200: User created successfully
  - 400: Invalid input data
  - 409: Email already exists

## User Login
- **Endpoint**: `POST /api/v1/auth/token`
- **Description**: Authenticate user and return access token
- **Request Body**:
  ```json
  {
    "username": "string (email)",
    "password": "string"
  }
  ```
- **Response**:
  ```json
  {
    "access_token": "string (JWT token)",
    "token_type": "string (bearer)"
  }
  ```
- **Status Codes**:
  - 200: Login successful
  - 401: Invalid credentials

## Get Current User
- **Endpoint**: `GET /api/v1/auth/me`
- **Description**: Get current authenticated user's information
- **Headers**: `Authorization: Bearer {token}`
- **Response**:
  ```json
  {
    "id": "string (user UUID)",
    "email": "string",
    "name": "string",
    "created_at": "datetime",
    "updated_at": "datetime",
    "software_background": "string",
    "hardware_background": "string"
  }
  ```
- **Status Codes**:
  - 200: Success
  - 401: Unauthorized

## Update User Profile
- **Endpoint**: `PUT /api/v1/auth/profile`
- **Description**: Update user profile information including background
- **Headers**: `Authorization: Bearer {token}`
- **Request Body**:
  ```json
  {
    "name": "string (optional)",
    "software_background": "string (optional)",
    "hardware_background": "string (optional)"
  }
  ```
- **Response**: Same as Get Current User
- **Status Codes**:
  - 200: Profile updated
  - 401: Unauthorized