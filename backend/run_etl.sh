#!/bin/bash
# Script to run the ETL process for ingesting documentation

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate  # On Windows, use: venv\Scripts\activate
else
    echo "Virtual environment not found. Please create it first with: python3 -m venv venv"
    exit 1
fi

echo "Running ETL process to ingest documentation..."
python run_etl.py

if [ $? -eq 0 ]; then
    echo "ETL process completed successfully!"
else
    echo "ETL process failed. Check the error messages above."
    exit 1
fi