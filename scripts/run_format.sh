#!/bin/bash

# color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TEMP_DIR="/tmp/maque_format_check"

# statistics variables
TOTAL_FILES=0
ISSUES_FOUND=0
FILES_WITH_ISSUES=0
FIXED_FILES=0

# create temporary directory
mkdir -p "$TEMP_DIR"

echo -e "${BLUE}📦 MaQueAI Code Format Checker & Fixer${NC}"
echo -e "${BLUE}====================================${NC}"
echo ""

# automatic installation of missing tools
install_tools() {
  local tools_to_install=()
  local need_update=false

  echo -e "${BLUE}🔍 Checking formatting tools...${NC}"

  # check clang-format
  if ! command -v clang-format &>/dev/null; then
    echo -e "${YELLOW}⚠️  clang-format not installed${NC}"
    tools_to_install+=("clang-format")
    need_update=true
  else
    echo -e "${GREEN}✅ clang-format installed: $(clang-format --version | head -1)${NC}"
  fi

  # check python3-pip
  if ! command -v pip3 &>/dev/null; then
    echo -e "${YELLOW}⚠️  python3-pip not installed${NC}"
    tools_to_install+=("python3-pip")
    need_update=true
  else
    echo -e "${GREEN}✅ pip3 installed${NC}"
  fi

  # check black
  if ! command -v black &>/dev/null; then
    echo -e "${YELLOW}⚠️  black not installed${NC}"
  else
    echo -e "${GREEN}✅ black installed: $(black --version)${NC}"
  fi

  # check isort
  if ! command -v isort &>/dev/null; then
    echo -e "${YELLOW}⚠️  isort not installed${NC}"
  else
    echo -e "${GREEN}✅ isort installed: $(isort --version | head -1)${NC}"
  fi

  # check shfmt
  if ! command -v shfmt &>/dev/null; then
    echo -e "${YELLOW}⚠️  shfmt not installed${NC}"
  else
    echo -e "${GREEN}✅ shfmt installed: $(shfmt --version)${NC}"
  fi

  # if there are system packages to install
  if [ ${#tools_to_install[@]} -gt 0 ]; then
    echo ""
    echo -e "${BLUE}🔧 Starting installation of missing system tools...${NC}"

    # update package index
    if [ "$need_update" = true ]; then
      echo -e "${BLUE}📦 Updating package index...${NC}"
      sudo apt-get update -qq
    fi

    # install system packages
    for tool in "${tools_to_install[@]}"; do
      echo -e "${BLUE}📦 Installing $tool...${NC}"
      if sudo apt-get install -y "$tool"; then
        echo -e "${GREEN}✅ $tool installation successful${NC}"
      else
        echo -e "${RED}❌ $tool installation failed${NC}"
      fi
    done
  fi

  # install Python formatting tools
  echo ""
  echo -e "${BLUE}🔧 Checking and installing Python formatting tools...${NC}"

  # install black
  if ! command -v black &>/dev/null; then
    echo -e "${BLUE}📦 Installing black...${NC}"
    if pip3 install --user black==23.*; then
      echo -e "${GREEN}✅ black installation successful${NC}"
      # add to PATH
      export PATH="$HOME/.local/bin:$PATH"
    else
      echo -e "${RED}❌ black installation failed${NC}"
    fi
  fi

  # install isort
  if ! command -v isort &>/dev/null; then
    echo -e "${BLUE}📦 Installing isort...${NC}"
    if pip3 install --user isort==5.*; then
      echo -e "${GREEN}✅ isort installation successful${NC}"
      # add to PATH
      export PATH="$HOME/.local/bin:$PATH"
    else
      echo -e "${RED}❌ isort installation failed${NC}"
    fi
  fi

  # install shfmt
  if ! command -v shfmt &>/dev/null; then
    echo -e "${BLUE}📦 Installing shfmt...${NC}"

    # try downloading from GitHub releases
    local shfmt_url="https://github.com/mvdan/sh/releases/latest/download/shfmt_v3.7.0_linux_amd64"
    local temp_shfmt="/tmp/shfmt"

    if wget -q "$shfmt_url" -O "$temp_shfmt" 2>/dev/null; then
      chmod +x "$temp_shfmt"
      if sudo mv "$temp_shfmt" /usr/local/bin/shfmt; then
        echo -e "${GREEN}✅ shfmt installation successful${NC}"
      else
        echo -e "${RED}❌ shfmt installation failed (cannot move to /usr/local/bin)${NC}"
      fi
    else
      echo -e "${YELLOW}⚠️  shfmt download failed, trying to install via snap...${NC}"
      if command -v snap &>/dev/null; then
        if sudo snap install shfmt; then
          echo -e "${GREEN}✅ shfmt installation successful via snap${NC}"
        else
          echo -e "${RED}❌ shfmt snap installation failed${NC}"
        fi
      else
        echo -e "${YELLOW}⚠️  snap not available, skipping shfmt installation${NC}"
      fi
    fi
  fi

  echo ""
  echo -e "${BLUE}🔍 Final tool status check:${NC}"
  check_final_status
}

# check final tool status
check_final_status() {
  local missing_tools=()

  if ! command -v clang-format &>/dev/null; then
    missing_tools+=("clang-format")
  fi

  if ! command -v black &>/dev/null; then
    missing_tools+=("black")
  fi

  if ! command -v isort &>/dev/null; then
    missing_tools+=("isort")
  fi

  if ! command -v shfmt &>/dev/null; then
    missing_tools+=("shfmt")
  fi

  if [ ${#missing_tools[@]} -eq 0 ]; then
    echo -e "${GREEN}🎉 All formatting tools are ready!${NC}"
    return 0
  else
    echo -e "${YELLOW}⚠️  The following tools are still missing:${NC}"
    for tool in "${missing_tools[@]}"; do
      echo -e "   - $tool"
    done
    echo ""
    echo -e "${BLUE}💡 You can manually install the missing tools:${NC}"
    echo -e "   ${GREEN}sudo apt-get install clang-format${NC}"
    echo -e "   ${GREEN}pip3 install --user black isort${NC}"
    echo -e "   ${GREEN}sudo snap install shfmt${NC}"
    echo ""
    echo -e "${BLUE}🔄 Continuing with available tools...${NC}"
    return 1
  fi
}

# create .clang-format configuration file (if not exists)
create_clang_format_config() {
  local config_file="$PROJECT_ROOT/.clang-format"
  if [ ! -f "$config_file" ]; then
    echo -e "${BLUE}📝 Creating .clang-format configuration...${NC}"
    cat >"$config_file" <<'EOF'
---
Language: Cpp
BasedOnStyle: Google
IndentWidth: 2
TabWidth: 2
UseTab: Never
ColumnLimit: 100
AlignConsecutiveAssignments: false
AlignConsecutiveDeclarations: false
AlignOperands: true
AlignTrailingComments: true
AllowAllParametersOfDeclarationOnNextLine: false
AllowShortBlocksOnASingleLine: false
AllowShortCaseLabelsOnASingleLine: false
AllowShortFunctionsOnASingleLine: Inline
AllowShortIfStatementsOnASingleLine: false
AllowShortLoopsOnASingleLine: false
AlwaysBreakAfterReturnType: None
AlwaysBreakBeforeMultilineStrings: false
AlwaysBreakTemplateDeclarations: true
BinPackArguments: false
BinPackParameters: false
BreakBeforeBinaryOperators: None
BreakBeforeBraces: Attach
BreakBeforeTernaryOperators: true
BreakConstructorInitializersBeforeComma: false
BreakAfterJavaFieldAnnotations: false
BreakStringLiterals: true
Cpp11BracedListStyle: true
DerivePointerAlignment: false
DisableFormat: false
IndentCaseLabels: true
IndentWrappedFunctionNames: false
KeepEmptyLinesAtTheStartOfBlocks: false
MacroBlockBegin: ''
MacroBlockEnd: ''
MaxEmptyLinesToKeep: 1
NamespaceIndentation: None
PointerAlignment: Left
ReflowComments: true
SortIncludes: true
SpaceAfterCStyleCast: false
SpaceBeforeAssignmentOperators: true
SpaceBeforeParens: ControlStatements
SpaceInEmptyParentheses: false
SpacesBeforeTrailingComments: 2
SpacesInAngles: false
SpacesInContainerLiterals: true
SpacesInCStyleCastParentheses: false
SpacesInParentheses: false
SpacesInSquareBrackets: false
Standard: Auto
EOF
    echo -e "${GREEN}✅ Created .clang-format configuration${NC}"
  fi
}

# check C++ file format
check_cpp_format() {
  local file="$1"
  local fix_mode="$2"

  if ! command -v clang-format &>/dev/null; then
    return 0
  fi

  local temp_file="$TEMP_DIR/$(basename "$file")"
  clang-format "$file" >"$temp_file"

  if ! diff -q "$file" "$temp_file" >/dev/null; then
    echo -e "${RED}❌ Format issues found: $file${NC}"
    ISSUES_FOUND=$((ISSUES_FOUND + 1))
    FILES_WITH_ISSUES=$((FILES_WITH_ISSUES + 1))

    if [ "$fix_mode" = "fix" ]; then
      cp "$temp_file" "$file"
      echo -e "${GREEN}✅ Fixed: $file${NC}"
      FIXED_FILES=$((FIXED_FILES + 1))
    elif [ "$fix_mode" = "check" ]; then
      echo -e "${YELLOW}💡 Run with --fix to automatically fix this file${NC}"
    fi
    return 1
  fi

  return 0
}

# check Python file format
check_python_format() {
  local file="$1"
  local fix_mode="$2"
  local has_issues=false

  # check black format
  if command -v black &>/dev/null; then
    if ! black --check --quiet "$file" 2>/dev/null; then
      echo -e "${RED}❌ Black format issues: $file${NC}"
      has_issues=true

      if [ "$fix_mode" = "fix" ]; then
        black --quiet "$file"
        echo -e "${GREEN}✅ Fixed with black: $file${NC}"
      fi
    fi
  fi

  # check isort import sorting
  if command -v isort &>/dev/null; then
    if ! isort --check-only --quiet "$file" 2>/dev/null; then
      echo -e "${RED}❌ Import sorting issues: $file${NC}"
      has_issues=true

      if [ "$fix_mode" = "fix" ]; then
        isort --quiet "$file"
        echo -e "${GREEN}✅ Fixed imports: $file${NC}"
      fi
    fi
  fi

  if [ "$has_issues" = true ]; then
    ISSUES_FOUND=$((ISSUES_FOUND + 1))
    FILES_WITH_ISSUES=$((FILES_WITH_ISSUES + 1))
    if [ "$fix_mode" = "fix" ]; then
      FIXED_FILES=$((FIXED_FILES + 1))
    elif [ "$fix_mode" = "check" ]; then
      echo -e "${YELLOW}💡 Run with --fix to automatically fix this file${NC}"
    fi
    return 1
  fi

  return 0
}

# check Shell script format
check_shell_format() {
  local file="$1"
  local fix_mode="$2"

  if ! command -v shfmt &>/dev/null; then
    return 0
  fi

  local temp_file="$TEMP_DIR/$(basename "$file")"
  if shfmt -i 2 -ci "$file" >"$temp_file" 2>/dev/null; then
    if ! diff -q "$file" "$temp_file" >/dev/null; then
      echo -e "${RED}❌ Shell format issues: $file${NC}"
      ISSUES_FOUND=$((ISSUES_FOUND + 1))
      FILES_WITH_ISSUES=$((FILES_WITH_ISSUES + 1))

      if [ "$fix_mode" = "fix" ]; then
        cp "$temp_file" "$file"
        echo -e "${GREEN}✅ Fixed: $file${NC}"
        FIXED_FILES=$((FIXED_FILES + 1))
      elif [ "$fix_mode" = "check" ]; then
        echo -e "${YELLOW}💡 Run with --fix to automatically fix this file${NC}"
      fi
      return 1
    fi
  else
    echo -e "${YELLOW}⚠️  Cannot parse shell script: $file${NC}"
  fi

  return 0
}

# process single file
process_file() {
  local file="$1"
  local fix_mode="$2"

  TOTAL_FILES=$((TOTAL_FILES + 1))

  case "$file" in
    *.cpp | *.cc | *.cxx | *.c++ | *.h | *.hpp | *.hxx)
      check_cpp_format "$file" "$fix_mode"
      ;;
    *.py)
      check_python_format "$file" "$fix_mode"
      ;;
    *.sh)
      check_shell_format "$file" "$fix_mode"
      ;;
    *)
      # skip unsupported file types
      TOTAL_FILES=$((TOTAL_FILES - 1))
      ;;
  esac
}

# find and process all related files
process_all_files() {
  local fix_mode="$1"

  echo -e "${BLUE}🔍 Scanning for code files...${NC}"

  # C++ files
  while IFS= read -r -d '' file; do
    process_file "$file" "$fix_mode"
  done < <(find "$PROJECT_ROOT" -type f \( -name "*.cpp" -o -name "*.cc" -o -name "*.cxx" -o -name "*.c++" -o -name "*.h" -o -name "*.hpp" -o -name "*.hxx" \) -not -path "*/build/*" -not -path "*/.git/*" -not -path "*/.*" -print0)

  # Python files
  while IFS= read -r -d '' file; do
    process_file "$file" "$fix_mode"
  done < <(find "$PROJECT_ROOT" -type f -name "*.py" -not -path "*/build/*" -not -path "*/.git/*" -not -path "*/.*" -print0)

  # Shell scripts
  while IFS= read -r -d '' file; do
    process_file "$file" "$fix_mode"
  done < <(find "$PROJECT_ROOT" -type f -name "*.sh" -not -path "*/build/*" -not -path "*/.git/*" -not -path "*/.*" -print0)
}

# show help information
show_help() {
  echo "Usage: $0 [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  --check         Only check format issues (default)"
  echo "  --fix           Automatically fix format issues"
  echo "  --install-only  Only install formatting tools then exit"
  echo "  --help          Show this help message"
  echo ""
  echo "Supported file types:"
  echo "  - C/C++ files (.cpp, .cc, .cxx, .c++, .h, .hpp, .hxx)"
  echo "  - Python files (.py)"
  echo "  - Shell scripts (.sh)"
  echo ""
  echo "Examples:"
  echo "  $0                    # Check format issues"
  echo "  $0 --check           # Check format issues"
  echo "  $0 --fix             # Fix format issues automatically"
  echo "  $0 --install-only    # Only install tools"
}

# clean up temporary files
cleanup() {
  rm -rf "$TEMP_DIR"
}

# set up cleanup on exit
trap cleanup EXIT

# main function
main() {
  local mode="check"
  local install_only=false

  # parse command line arguments
  while [[ $# -gt 0 ]]; do
    case $1 in
      --fix)
        mode="fix"
        shift
        ;;
      --check)
        mode="check"
        shift
        ;;
      --install-only)
        install_only=true
        shift
        ;;
      --help)
        show_help
        exit 0
        ;;
      *)
        echo -e "${RED}Unknown option: $1${NC}"
        show_help
        exit 1
        ;;
    esac
  done

  # install tools
  install_tools

  # if it's only for installing tools, exit
  if [ "$install_only" = true ]; then
    echo -e "${GREEN}🎉 Tools installation completed!${NC}"
    exit 0
  fi

  # create configuration file
  create_clang_format_config

  # process files
  if [ "$mode" = "fix" ]; then
    echo -e "${YELLOW}🔧 Running in FIX mode - files will be modified!${NC}"
  else
    echo -e "${BLUE}🔍 Running in CHECK mode - no files will be modified${NC}"
  fi

  echo ""
  process_all_files "$mode"

  # show summary
  echo ""
  echo -e "${BLUE}📊 Format Check Summary${NC}"
  echo -e "${BLUE}=====================${NC}"
  echo -e "Total files processed: ${YELLOW}$TOTAL_FILES${NC}"
  echo -e "Files with issues: ${RED}$FILES_WITH_ISSUES${NC}"
  echo -e "Total issues found: ${RED}$ISSUES_FOUND${NC}"

  if [ "$mode" = "fix" ]; then
    echo -e "Files fixed: ${GREEN}$FIXED_FILES${NC}"
  fi

  echo ""

  if [ $FILES_WITH_ISSUES -eq 0 ]; then
    echo -e "${GREEN}🎉 All files are properly formatted!${NC}"
    exit 0
  else
    if [ "$mode" = "check" ]; then
      echo -e "${YELLOW}💡 Run '$0 --fix' to automatically fix the issues${NC}"
    fi
    exit 1
  fi
}

# run main function
main "$@"
