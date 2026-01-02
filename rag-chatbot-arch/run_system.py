#!/usr/bin/env python3
"""
Script to run the complete RAG Chatbot system
"""
import os
import subprocess
import sys
import argparse
import time

def run_embedding_pipeline():
    """Run the embedding pipeline to index book content"""
    print("🚀 Starting embedding pipeline...")
    try:
        # Change to backend directory and run embedding
        result = subprocess.run([
            sys.executable, "run_embedding.py"
        ], cwd="backend", capture_output=True, text=True)

        if result.returncode == 0:
            print("✅ Embedding pipeline completed successfully!")
            print(result.stdout)
        else:
            print("❌ Embedding pipeline failed!")
            print(result.stderr)
            return False
    except Exception as e:
        print(f"❌ Error running embedding pipeline: {e}")
        return False

    return True

def run_backend_server():
    """Run the backend server"""
    print("📡 Starting backend server...")
    try:
        # Start the FastAPI server
        process = subprocess.Popen([
            sys.executable, "-m", "uvicorn", "main:app",
            "--host", "0.0.0.0", "--port", "8000", "--reload"
        ], cwd="backend")

        print("✅ Backend server started on http://0.0.0.0:8000")
        print("💡 Press Ctrl+C to stop the server")

        # Wait for the process to complete (this will run indefinitely)
        try:
            process.wait()
        except KeyboardInterrupt:
            print("\n🛑 Shutting down backend server...")
            process.terminate()
            process.wait()
            print("✅ Backend server stopped")

    except Exception as e:
        print(f"❌ Error starting backend server: {e}")
        return False

    return True

def main():
    parser = argparse.ArgumentParser(description="Run the complete RAG Chatbot system")
    parser.add_argument("--skip-embedding", action="store_true",
                       help="Skip the embedding pipeline (use existing embeddings)")
    parser.add_argument("--run-server-only", action="store_true",
                       help="Run only the backend server (skip embedding)")

    args = parser.parse_args()

    print("🤖 Starting RAG Chatbot System for Physical AI & Humanoid Robotics Book")
    print("=" * 70)

    # Run embedding pipeline if not skipped
    if not args.run_server_only and not args.skip_embedding:
        if not run_embedding_pipeline():
            print("❌ System startup failed due to embedding pipeline error")
            return 1
    elif args.run_server_only:
        print("⏭️ Skipping embedding pipeline (server only mode)")
    else:
        print("⏭️ Skipping embedding pipeline (--skip-embedding flag)")

    print("\n" + "=" * 70)

    # Run the backend server
    return run_backend_server()

if __name__ == "__main__":
    sys.exit(main() or 0)