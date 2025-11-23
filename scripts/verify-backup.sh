#!/bin/bash
# ============================================================
# ROS2 Navigation Project - Backup Verification Script
# ============================================================
# Purpose: Verify backup integrity and restorability
# Usage:
#   ./scripts/verify-backup.sh                    # Verify latest backup
#   ./scripts/verify-backup.sh backup_file.tar.gz # Verify specific backup
#
# Tests:
#   - Archive integrity (not corrupted)
#   - Checksum verification (SHA256 match)
#   - Critical files present
#   - Can extract without errors
#   - Git repository valid (if present)
# ============================================================

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BACKUP_DIR="$HOME/backups"
TEST_DIR="/tmp/ros2_backup_verification_$$"
PROJECT_NAME="ros2_navigation"

# Critical files that must exist
CRITICAL_FILES=(
  "README.md"
  "RECOVERY.md"
  "BACKUP_GUIDE.md"
  "DEPENDENCIES.md"
  "CLAUDE.md"
  ".gitignore"
  ".gitattributes"
  "start_robot_dashboard.sh"
  "src/cartographer_slam"
  "src/map_server"
  "src/localization_server"
  "src/path_planner_server"
  "backend/requirements.txt"
  "backend/app/main.py"
  "project/package.json"
  "project/src/App.tsx"
)

# Parse arguments
BACKUP_FILE=""
if [ $# -eq 0 ]; then
  # Find latest backup
  LATEST=$(ls -t "$BACKUP_DIR"/${PROJECT_NAME}_backup_*.tar.gz 2>/dev/null | head -n 1)
  if [ -z "$LATEST" ]; then
    echo -e "${RED}Error: No backups found in $BACKUP_DIR${NC}"
    exit 1
  fi
  BACKUP_FILE="$LATEST"
elif [ $# -eq 1 ]; then
  # Use specified backup
  if [ -f "$1" ]; then
    BACKUP_FILE="$1"
  elif [ -f "$BACKUP_DIR/$1" ]; then
    BACKUP_FILE="$BACKUP_DIR/$1"
  else
    echo -e "${RED}Error: Backup file not found: $1${NC}"
    exit 1
  fi
else
  echo "Usage: $0 [backup_file.tar.gz]"
  echo "  If no file specified, verifies latest backup"
  exit 1
fi

# Banner
echo ""
echo "============================================================"
echo "    ROS2 Navigation Project - Backup Verification"
echo "============================================================"
echo ""

echo "Testing backup: $(basename "$BACKUP_FILE")"
echo "Full path: $BACKUP_FILE"
echo ""

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Test 1: File exists and is readable
echo -e "${BLUE}[1/6]${NC} Checking file accessibility..."
if [ -f "$BACKUP_FILE" ] && [ -r "$BACKUP_FILE" ]; then
  FILE_SIZE=$(du -h "$BACKUP_FILE" | cut -f1)
  echo -e "${GREEN}✅ PASS${NC} - File exists and is readable ($FILE_SIZE)"
  ((TESTS_PASSED++))
else
  echo -e "${RED}❌ FAIL${NC} - File not accessible"
  ((TESTS_FAILED++))
  exit 1
fi
echo ""

# Test 2: Checksum verification
echo -e "${BLUE}[2/6]${NC} Verifying SHA256 checksum..."
CHECKSUM_FILE="${BACKUP_FILE}.sha256"
if [ -f "$CHECKSUM_FILE" ]; then
  cd "$(dirname "$BACKUP_FILE")"
  if sha256sum -c "$(basename "$CHECKSUM_FILE")" &>/dev/null; then
    STORED_CHECKSUM=$(cut -d' ' -f1 "$CHECKSUM_FILE")
    echo -e "${GREEN}✅ PASS${NC} - Checksum matches (${STORED_CHECKSUM:0:16}...)"
    ((TESTS_PASSED++))
  else
    echo -e "${RED}❌ FAIL${NC} - Checksum mismatch! File may be corrupted."
    ((TESTS_FAILED++))
  fi
else
  echo -e "${YELLOW}⚠️  SKIP${NC} - Checksum file not found"
fi
echo ""

# Test 3: Archive integrity
echo -e "${BLUE}[3/6]${NC} Testing archive integrity..."
if tar -tzf "$BACKUP_FILE" > /dev/null 2>&1; then
  FILE_COUNT=$(tar -tzf "$BACKUP_FILE" | wc -l)
  echo -e "${GREEN}✅ PASS${NC} - Archive is valid ($FILE_COUNT files)"
  ((TESTS_PASSED++))
else
  echo -e "${RED}❌ FAIL${NC} - Archive is corrupted or invalid"
  ((TESTS_FAILED++))
  exit 1
fi
echo ""

# Test 4: Extract test
echo -e "${BLUE}[4/6]${NC} Testing extraction..."
mkdir -p "$TEST_DIR"
if tar -xzf "$BACKUP_FILE" -C "$TEST_DIR" > /dev/null 2>&1; then
  echo -e "${GREEN}✅ PASS${NC} - Extraction successful"
  ((TESTS_PASSED++))
else
  echo -e "${RED}❌ FAIL${NC} - Extraction failed"
  ((TESTS_FAILED++))
  rm -rf "$TEST_DIR"
  exit 1
fi
echo ""

# Test 5: Critical files present
echo -e "${BLUE}[5/6]${NC} Checking critical files..."
EXTRACTED_DIR=$(ls -d "$TEST_DIR"/*/ | head -n 1)
MISSING_FILES=()

for FILE in "${CRITICAL_FILES[@]}"; do
  if [ ! -e "${EXTRACTED_DIR}${FILE}" ]; then
    MISSING_FILES+=("$FILE")
  fi
done

if [ ${#MISSING_FILES[@]} -eq 0 ]; then
  echo -e "${GREEN}✅ PASS${NC} - All ${#CRITICAL_FILES[@]} critical files present"
  ((TESTS_PASSED++))
else
  echo -e "${RED}❌ FAIL${NC} - ${#MISSING_FILES[@]} critical files missing:"
  for FILE in "${MISSING_FILES[@]}"; do
    echo "  - $FILE"
  done
  ((TESTS_FAILED++))
fi
echo ""

# Test 6: Git repository validity (if present)
echo -e "${BLUE}[6/6]${NC} Checking Git repository..."
if [ -d "${EXTRACTED_DIR}.git" ]; then
  cd "$EXTRACTED_DIR"
  if git status &>/dev/null; then
    GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null)
    echo -e "${GREEN}✅ PASS${NC} - Git repository is valid (commit: $GIT_COMMIT)"
    ((TESTS_PASSED++))
  else
    echo -e "${RED}❌ FAIL${NC} - Git repository is corrupted"
    ((TESTS_FAILED++))
  fi
else
  echo -e "${YELLOW}⚠️  SKIP${NC} - No Git repository in backup"
fi
echo ""

# Cleanup
echo "Cleaning up test directory..."
rm -rf "$TEST_DIR"
echo -e "${GREEN}✅ Cleanup complete${NC}"
echo ""

# Summary
echo "============================================================"
echo "              Verification Results"
echo "============================================================"
echo "Backup file:  $(basename "$BACKUP_FILE")"
echo "Tests passed: $TESTS_PASSED"
echo "Tests failed: $TESTS_FAILED"
echo "Status:       $([ $TESTS_FAILED -eq 0 ] && echo -e "${GREEN}VALID${NC}" || echo -e "${RED}INVALID${NC}")"
echo "============================================================"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
  echo -e "${GREEN}🎉 Backup is valid and restorable!${NC}"
  echo ""
  echo "This backup can be used for recovery. To restore:"
  echo "  1. Extract: tar -xzf $(basename "$BACKUP_FILE")"
  echo "  2. Follow: RECOVERY.md for complete recovery steps"
  echo ""
  exit 0
else
  echo -e "${RED}⚠️  Backup verification failed!${NC}"
  echo ""
  echo "This backup may not be restorable. Recommendations:"
  echo "  1. Create a new backup: ./scripts/backup.sh"
  echo "  2. Check disk space and permissions"
  echo "  3. Verify git status before backing up"
  echo ""
  exit 1
fi

