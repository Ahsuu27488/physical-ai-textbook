#!/bin/bash
# Script to set up and run the backend

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows, use: venv\Scripts\activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Run the ETL process to ingest the documentation
echo "Running ETL process to ingest documentation..."
python -m app.etl.data_ingestion

# Start the FastAPI server
echo "Starting FastAPI server on port 8000..."
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload