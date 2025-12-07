#!/usr/bin/env python3
"""
Script to run the ETL process for ingesting documentation into the vector database.
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the app directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from app.etl.data_ingestion import ETLProcessor

def main():
    print("Starting ETL process for Physical AI & Humanoid Robotics documentation...")

    # Create ETL processor instance
    # The path is relative to where the script is run from the backend directory
    etl = ETLProcessor(source_dir="../frontend/docs")

    try:
        # Run the ETL process
        result = etl.run_etl()
        print(f"ETL process completed successfully: {result}")

        if result["chunks_loaded"] == 0:
            print("Warning: No chunks were loaded. Check the logs above for errors.")
            return 1
        else:
            print(f"Successfully loaded {result['chunks_loaded']} document chunks into the vector database.")
            return 0

    except Exception as e:
        print(f"Error during ETL process: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())