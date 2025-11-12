#!/bin/bash
# ============================================================
# ROS2 Navigation Project - Backup Script
# ============================================================
# Purpose: Create complete workspace backup with checksums
# Usage:
#   ./scripts/backup.sh              # Standard backup (source only)
#   ./scripts/backup.sh --full       # Full backup (includes build artifacts)
#   ./scripts/backup.sh --cloud      # Backup and upload to cloud
#
# Output: ~/backups/ros2_navigation_backup_YYYYMMDD.tar.gz
# ============================================================

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Configuration
BACKUP_DIR="$HOME/backups"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_NAME="ros2_navigation"
DATE_STAMP=$(date +%Y%m%d_%H%M%S)
BACKUP_NAME="${PROJECT_NAME}_backup_${DATE_STAMP}"
BACKUP_FILE="${BACKUP_DIR}/${BACKUP_NAME}.tar.gz"
RETENTION_COUNT=4  # Keep last 4 backups

# Parse arguments
INCLUDE_BUILD=false
UPLOAD_CLOUD=false

for arg in "$@"; do
  case $arg in
    --full)
      INCLUDE_BUILD=true
      shift
      ;;
    --cloud)
      UPLOAD_CLOUD=true
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --full    Include build artifacts (build/, install/, node_modules/)"
      echo "  --cloud   Upload backup to cloud storage after creation"
      echo "  --help    Show this help message"
      echo ""
      echo "Examples:"
      echo "  $0              # Standard backup (source code only)"
      echo "  $0 --full       # Full backup with build artifacts"
      echo "  $0 --cloud      # Standard backup + cloud upload"
      exit 0
      ;;
  esac
done

# Banner
echo ""
echo "============================================================"
echo "    ROS2 Navigation Project - Backup Utility"
echo "============================================================"
echo ""

# Check if inside project directory
if [ ! -f "$PROJECT_ROOT/RECOVERY.md" ]; then
  echo -e "${RED}Error: Must be run from project root or scripts/ directory${NC}"
  echo "Expected location: $PROJECT_ROOT"
  exit 1
fi

# Create backup directory if doesn't exist
mkdir -p "$BACKUP_DIR"

# Check git status (warn about uncommitted changes)
cd "$PROJECT_ROOT"
if [ -d ".git" ]; then
  if ! git diff-index --quiet HEAD -- 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Warning: You have uncommitted changes!${NC}"
    echo -e "${YELLOW}   Consider committing before backup.${NC}"
    git status --short
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
      echo "Backup cancelled."
      exit 1
    fi
  else
    echo -e "${GREEN}✅ Git status clean${NC}"
  fi
fi

# Build exclusion list
EXCLUDE_ARGS=""
EXCLUDE_ARGS+=" --exclude=.git"
EXCLUDE_ARGS+=" --exclude=__pycache__"
EXCLUDE_ARGS+=" --exclude=*.pyc"
EXCLUDE_ARGS+=" --exclude=*.pyo"
EXCLUDE_ARGS+=" --exclude=.pytest_cache"

if [ "$INCLUDE_BUILD" = false ]; then
  echo -e "${YELLOW}📦 Creating standard backup (source only)${NC}"
  EXCLUDE_ARGS+=" --exclude=build"
  EXCLUDE_ARGS+=" --exclude=install"
  EXCLUDE_ARGS+=" --exclude=log"
  EXCLUDE_ARGS+=" --exclude=node_modules"
  EXCLUDE_ARGS+=" --exclude=backend/venv"
  EXCLUDE_ARGS+=" --exclude=project/dist"
  EXCLUDE_ARGS+=" --exclude=.next"
  EXCLUDE_ARGS+=" --exclude=.cache"
else
  echo -e "${GREEN}📦 Creating full backup (includes build artifacts)${NC}"
fi

# Create backup
echo ""
echo "Creating backup archive..."
echo "Source: $PROJECT_ROOT"
echo "Target: $BACKUP_FILE"
echo ""

cd "$(dirname "$PROJECT_ROOT")"
PROJECT_DIR_NAME="$(basename "$PROJECT_ROOT")"

tar -czf "$BACKUP_FILE" $EXCLUDE_ARGS "$PROJECT_DIR_NAME"

if [ $? -eq 0 ]; then
  BACKUP_SIZE=$(du -h "$BACKUP_FILE" | cut -f1)
  echo -e "${GREEN}✅ Backup created: $BACKUP_FILE ($BACKUP_SIZE)${NC}"
else
  echo -e "${RED}❌ Backup failed!${NC}"
  exit 1
fi

# Generate checksum
echo ""
echo "Generating SHA256 checksum..."
cd "$BACKUP_DIR"
sha256sum "$(basename "$BACKUP_FILE")" > "${BACKUP_FILE}.sha256"

if [ $? -eq 0 ]; then
  CHECKSUM=$(cut -d' ' -f1 "${BACKUP_FILE}.sha256")
  echo -e "${GREEN}✅ Checksum: $CHECKSUM${NC}"
  echo -e "${GREEN}   Saved to: ${BACKUP_FILE}.sha256${NC}"
else
  echo -e "${YELLOW}⚠️  Checksum generation failed${NC}"
fi

# Retention policy - keep last N backups
echo ""
echo "Applying retention policy (keep last $RETENTION_COUNT backups)..."
cd "$BACKUP_DIR"
OLD_BACKUPS=$(ls -t ${PROJECT_NAME}_backup_*.tar.gz 2>/dev/null | tail -n +$((RETENTION_COUNT + 1)))

if [ -n "$OLD_BACKUPS" ]; then
  echo "Removing old backups:"
  for OLD_BACKUP in $OLD_BACKUPS; do
    echo "  - $OLD_BACKUP"
    rm -f "$OLD_BACKUP"
    rm -f "${OLD_BACKUP}.sha256"
  done
  echo -e "${GREEN}✅ Cleanup complete${NC}"
else
  echo "No old backups to remove"
fi

# Cloud upload
if [ "$UPLOAD_CLOUD" = true ]; then
  echo ""
  echo "Uploading to cloud storage..."
  
  # Check if rclone is installed
  if command -v rclone &> /dev/null; then
    # Try to upload to Google Drive (if configured)
    if rclone listremotes | grep -q "gdrive:"; then
      echo "Uploading to Google Drive..."
      rclone copy "$BACKUP_FILE" gdrive:ROS2_Backups/ --progress
      rclone copy "${BACKUP_FILE}.sha256" gdrive:ROS2_Backups/ --progress
      
      if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Uploaded to Google Drive: ROS2_Backups/$(basename "$BACKUP_FILE")${NC}"
      else
        echo -e "${YELLOW}⚠️  Google Drive upload failed${NC}"
      fi
    else
      echo -e "${YELLOW}⚠️  rclone 'gdrive' remote not configured${NC}"
      echo "Configure with: rclone config"
    fi
  else
    echo -e "${YELLOW}⚠️  rclone not installed. Cannot upload to cloud.${NC}"
    echo "Install with: sudo apt install rclone"
  fi
fi

# Summary
echo ""
echo "============================================================"
echo "              Backup Summary"
echo "============================================================"
echo "Backup file:  $BACKUP_FILE"
echo "Size:         $BACKUP_SIZE"
echo "Checksum:     ${CHECKSUM:0:16}..."
echo "Type:         $([ "$INCLUDE_BUILD" = true ] && echo "Full (with build artifacts)" || echo "Standard (source only)")"
echo "Retention:    Last $RETENTION_COUNT backups kept"
if [ "$UPLOAD_CLOUD" = true ]; then
  echo "Cloud sync:   Attempted (check output above)"
fi
echo "============================================================"
echo ""

# Get git commit info if available
if [ -d "$PROJECT_ROOT/.git" ]; then
  cd "$PROJECT_ROOT"
  GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null)
  GIT_BRANCH=$(git branch --show-current 2>/dev/null)
  
  if [ -n "$GIT_COMMIT" ]; then
    echo "Git commit:   $GIT_COMMIT"
    echo "Git branch:   $GIT_BRANCH"
    echo ""
  fi
fi

# Next steps
echo "Next steps:"
echo "  1. Verify backup:  ./scripts/verify-backup.sh"
echo "  2. Test recovery:  See RECOVERY.md"
echo "  3. Cloud backup:   Upload $(basename "$BACKUP_FILE") to cloud storage"
echo ""

echo -e "${GREEN}🎉 Backup complete!${NC}"
echo ""

