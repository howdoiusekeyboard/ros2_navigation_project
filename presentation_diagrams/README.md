# Presentation Diagrams

This directory contains Mermaid diagram files (`.mmd`) extracted from the main presentation package for easy rendering to images.

## Diagrams Included

1. **01_high_level_architecture.mmd** - System architecture showing all layers and components
2. **02_data_flow.mmd** - Sequence diagram showing data flow through the system
3. **03_navigation_decision_logging.mmd** - Flowchart of decision logging pipeline
4. **04_explanation_generation.mmd** - Flowchart of explanation generation process
5. **05_conversation_memory_spatial_resolution.mmd** - Flowchart of spatial reference resolution
6. **06_digital_twin_anomaly_detection.mmd** - Flowchart of anomaly detection process

## ✅ Images Generated

All diagrams have been automatically rendered to PNG format:
- `01_high_level_architecture.png` (109 KB)
- `02_data_flow.png` (96 KB)
- `03_navigation_decision_logging.png` (126 KB)
- `04_explanation_generation.png` (103 KB)
- `05_conversation_memory_spatial_resolution.png` (76 KB)
- `06_digital_twin_anomaly_detection.png` (91 KB)

These PNG files are ready to use in your presentation slides!

## How to Re-render Images (if needed)

### Option 1: Python Script (Already Used)
```bash
cd presentation_diagrams
python3 render_diagrams.py
```

### Option 2: Mermaid CLI (Requires Chrome)

```bash
# Install Mermaid CLI
npm install -g @mermaid-js/mermaid-cli

# Render a diagram to PNG
mmdc -i 01_high_level_architecture.mmd -o 01_high_level_architecture.png

# Render all diagrams
for file in *.mmd; do
    mmdc -i "$file" -o "${file%.mmd}.png"
done
```

### Option 2: Online Mermaid Editor

1. Go to https://mermaid.live/
2. Copy the contents of any `.mmd` file
3. Paste into the editor
4. Export as PNG/SVG

### Option 3: VS Code Extension

1. Install "Markdown Preview Mermaid Support" extension
2. Open the `.mmd` file
3. Use the preview pane to view the diagram
4. Right-click to export as image

### Option 4: GitHub/GitLab

If you push these files to a repository, GitHub and GitLab will automatically render Mermaid diagrams in markdown files.

## Usage in Presentation

These diagrams can be:
- Converted to PNG/PDF for slides (PowerPoint, Google Slides, etc.)
- Embedded in markdown documentation
- Used in web-based presentations
- Printed for handouts

## Notes

- All diagrams use standard Mermaid syntax
- Diagrams are optimized for readability at presentation size
- Colors and styling can be customized by adding theme directives to each file
