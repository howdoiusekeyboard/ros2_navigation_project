#!/usr/bin/env python3
"""
Render Mermaid diagrams to PNG images using mermaid.ink API
"""
import os
import base64
import requests
import json
from pathlib import Path

def encode_mermaid(mermaid_code: str) -> str:
    """Encode Mermaid diagram code to base64 for API"""
    # Remove any leading/trailing whitespace
    mermaid_code = mermaid_code.strip()
    # Encode to base64
    encoded = base64.urlsafe_b64encode(mermaid_code.encode('utf-8')).decode('utf-8')
    return encoded

def render_diagram(mmd_file: Path, output_file: Path):
    """Render a single Mermaid diagram to PNG"""
    print(f"Rendering {mmd_file.name}...")
    
    # Read Mermaid code
    with open(mmd_file, 'r', encoding='utf-8') as f:
        mermaid_code = f.read()
        
    # Strictly validate Mermaid content instead of URL prefix to ensure only diagrams are processed
    valid_keywords = ('graph', 'sequenceDiagram', 'classDiagram', 'stateDiagram', 'pie', 'gantt')
    if not any(mermaid_code.strip().startswith(kw) for kw in valid_keywords):
        print(f"✗ Invalid Mermaid content in {mmd_file.name}: Missing diagram keyword")
        return False
    
    # Encode for API (urllib.parse.quote is unnecessary for base64url)
    encoded = encode_mermaid(mermaid_code)
    
    # Try mermaid.ink API
    api_url = f"https://mermaid.ink/img/{encoded}"
    
    try:
        response = requests.get(api_url, timeout=30)
        if response.status_code == 200:
            with open(output_file, 'wb') as f:
                f.write(response.content)
            print(f"✓ Successfully rendered {output_file.name}")
            return True
        else:
            print(f"✗ API returned status {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Error rendering {mmd_file.name}: {e}")
        return False

def main():
    """Render all .mmd files in current directory"""
    script_dir = Path(__file__).parent
    mmd_files = list(script_dir.glob("*.mmd"))
    
    if not mmd_files:
        print("No .mmd files found in current directory")
        return
    
    print(f"Found {len(mmd_files)} diagram files to render\n")
    
    success_count = 0
    for mmd_file in sorted(mmd_files):
        output_file = mmd_file.with_suffix('.png')
        if render_diagram(mmd_file, output_file):
            success_count += 1
        print()
    
    print(f"Rendered {success_count}/{len(mmd_files)} diagrams successfully")

if __name__ == "__main__":
    main()
