@echo off
REM Script to set up and run the backend on Windows

REM Create and activate virtual environment
python -m venv venv
call venv\Scripts\activate

REM Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

REM Run the ETL process to ingest the documentation
echo Running ETL process to ingest documentation...
python -m app.etl.data_ingestion

REM Start the FastAPI server
echo Starting FastAPI server on port 8000...
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload