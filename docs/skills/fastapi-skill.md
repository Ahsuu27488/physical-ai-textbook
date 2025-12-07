# FastAPI Skill

## Installation

### Basic installation:
```bash
pip install fastapi
```

### With standard dependencies (recommended):
```bash
pip install "fastapi[standard]"
```

### With all optional dependencies:
```bash
pip install "fastapi[all]"
```

### Install Uvicorn ASGI server:
```bash
pip install "uvicorn[standard]"
```

## Basic Application

### Create a simple FastAPI app:
```python
from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/items/{item_id}")
def read_item(item_id: int, q: str = None):
    return {"item_id": item_id, "q": q}
```

### FastAPI app with metadata:
```python
from fastapi import FastAPI

app = FastAPI(
    title="My API",
    description="API for managing items and users",
    version="1.0.0",
    docs_url="/documentation",  # Custom docs URL
    redoc_url="/redoc"          # Alternative docs
)
```

## Running the Application

### Development mode with auto-reload:
```bash
# Using fastapi dev command
fastapi dev main.py

# Or using uvicorn directly
uvicorn main:app --reload
```

### Production mode:
```bash
# Using fastapi run command
fastapi run main.py

# Using uvicorn directly
uvicorn main:app --host 0.0.0.0 --port 8000

# With multiple workers (for production)
uvicorn main:app --workers 4 --host 0.0.0.0 --port 8000
```

### Programmatic startup:
```python
import uvicorn
from main import app

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Path Operations

### HTTP Methods:
```python
from fastapi import FastAPI

app = FastAPI()

@app.get("/items")
def get_items():
    return {"method": "GET"}

@app.post("/items")
def create_item():
    return {"method": "POST"}

@app.put("/items/{item_id}")
def update_item(item_id: int):
    return {"method": "PUT", "item_id": item_id}

@app.delete("/items/{item_id}")
def delete_item(item_id: int):
    return {"method": "DELETE", "item_id": item_id}
```

## Request and Response Models

### Using Pydantic models:
```python
from fastapi import FastAPI
from pydantic import BaseModel

class Item(BaseModel):
    name: str
    description: str = None
    price: float
    tax: float = None

app = FastAPI()

@app.post("/items/")
def create_item(item: Item):
    return item
```

## Dependencies

### Simple dependency:
```python
from fastapi import Depends, FastAPI

app = FastAPI()

async def common_parameters(q: str = None, skip: int = 0, limit: int = 100):
    return {"q": q, "skip": skip, "limit": limit}

@app.get("/items/")
async def read_items(commons: dict = Depends(common_parameters)):
    return commons
```

## Testing

### Basic test setup:
```python
from fastapi import FastAPI
from fastapi.testclient import TestClient

app = FastAPI()

@app.get("/")
def read_main():
    return {"msg": "Hello World"}

client = TestClient(app)

def test_read_main():
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"msg": "Hello World"}
```

## Environment Variables and Settings

### Using Pydantic Settings:
```python
from pydantic import BaseSettings

class Settings(BaseSettings):
    app_name: str = "My Application"
    admin_email: str
    database_url: str

    class Config:
        env_file = ".env"

settings = Settings()
```

Install pydantic-settings:
```bash
pip install pydantic-settings
```

## Error Handling

### Custom HTTP exceptions:
```python
from fastapi import FastAPI, HTTPException

app = FastAPI()

@app.get("/items/{item_id}")
def read_item(item_id: int):
    if item_id < 0:
        raise HTTPException(
            status_code=404,
            detail="Item not found",
            headers={"X-Error": "There goes my error"},
        )
    return {"item_id": item_id}
```

## CORS Middleware

### Enable CORS:
```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Don't use "*" in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Project Structure Example

```
my-fastapi-project/
├── app/
│   ├── __init__.py
│   ├── main.py          # FastAPI app instance
│   ├── models/          # Pydantic models
│   ├── schemas/         # Request/response schemas
│   ├── database/        # Database setup
│   ├── api/             # API routes
│   │   ├── __init__.py
│   │   └── v1/          # API versioning
│   │       ├── __init__.py
│   │       └── endpoints/
│   │           ├── __init__.py
│   │           └── users.py
│   └── utils/           # Utility functions
├── tests/               # Test files
├── requirements.txt     # Dependencies
├── alembic/             # Database migrations
└── .env                 # Environment variables
```

## Requirements.txt Example:
```
fastapi[standard]==0.115.4
uvicorn[standard]==0.34.0
pydantic==2.10.3
pydantic-settings==2.7.1
sqlalchemy==2.0.36
psycopg2-binary==2.9.10
python-multipart==0.0.20
```