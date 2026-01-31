# Physical AI & Humanoid Robotics RAG API Backend

This backend provides a Retrieval Augmented Generation (RAG) API for the Physical AI textbook, allowing users to ask questions about the content and receive AI-powered responses with relevant context.

## Features

- FastAPI-based REST API
- RAG system with Qdrant vector database
- Document ingestion from Markdown files
- Chat endpoint with context-aware responses
- Document search functionality

## Prerequisites

- Python 3.11+
- Qdrant vector database (can run locally or remotely)
- OpenAI API key

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd physical-ai-textbook/backend
   ```

2. **Set up environment variables**
   Create a `.env` file in the backend directory with the following variables:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=https://your-qdrant-instance-url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=physical_ai_docs
   ```

3. **Install dependencies**
   ```bash
   # Create virtual environment
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install required packages
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

## Running the Backend

### Method 1: Using the run script (runs ETL + server)
```bash
# On Linux/Mac:
./run_backend.sh

# On Windows:
run_backend.bat
```

### Method 2: Manual setup
```bash
# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Run ETL process to ingest documentation
python run_etl.py

# Start the server
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

### Method 3: Run ETL separately
```bash
# On Linux/Mac:
./run_etl.sh

# Or run directly:
python run_etl.py
```

The API will be available at `http://localhost:8000`.

## API Endpoints

- `GET /` - Root endpoint with API information
- `GET /health` - Health check for the API
- `GET /documents/health` - Health check for the document database
- `POST /api/chat` - Chat endpoint for asking questions about the textbook
- `POST /api/search` - Search endpoint for finding specific documents

## ETL Process

The ETL (Extract, Transform, Load) process:

1. **Extract**: Reads Markdown files from the `../frontend/docs/` directory
2. **Transform**: Converts Markdown to text, chunks the content, and generates embeddings
3. **Load**: Stores the document chunks in the Qdrant vector database

Run the ETL process separately with:
```bash
python run_etl.py
```

## Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py                 # Main FastAPI application
│   ├── models.py              # Pydantic models
│   ├── vector_db.py           # Qdrant client implementation
│   ├── etl/
│   │   ├── __init__.py
│   │   └── data_ingestion.py  # ETL process for document ingestion
│   └── api/
│       ├── __init__.py
│       └── chat.py            # Chat API endpoints
├── requirements.txt           # Python dependencies
├── run_backend.sh             # Linux/Mac startup script
├── run_backend.bat            # Windows startup script
├── run_etl.sh                 # Linux/Mac ETL script
├── run_etl.py                 # Python script to run ETL process
└── .env                      # Environment variables (not committed)
```

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key for generating embeddings and chat responses
- `QDRANT_URL`: URL for your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant cloud instances
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection to use (default: physical_ai_docs)

## Troubleshooting

If you encounter issues with the ETL process:
1. Verify that your OpenAI API key is valid and has sufficient quota
2. Check that the Qdrant instance is accessible and the API key is correct
3. Ensure that the `../frontend/docs/` directory exists and contains Markdown files
4. Check the logs for specific error messages

## Models Used

- **Chat Model**: `gpt-4o-mini` - Used for generating responses in the chat endpoint
- **Embedding Model**: `text-embedding-ada-002` - Used for generating document embeddings
- **Tokenizer**: Based on `gpt-4` model - Used for tokenizing text during chunking