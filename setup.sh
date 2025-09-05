#!/bin/bash

# pysics engine - dependency setup script
# this script helps install the required dependencies

echo "=== Physics Engine - Setup ==="
echo ""

# check OS
OS="unknown"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
elif [[ "$OSTYPE" == "cygwin" ]]; then
    OS="windows"
elif [[ "$OSTYPE" == "msys" ]]; then
    OS="windows"
else
    echo "Unsupported OS: $OSTYPE"
    exit 1
fi

echo "Detected OS: $OS"
echo ""

install_macos() {
    echo "Installing dependencies on macOS..."

    # Check if Homebrew is installed
    if ! command -v brew &> /dev/null; then
        echo "Homebrew not found. Installing Homebrew..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    fi

    echo "Installing CMake and SFML via Homebrew..."
    brew install cmake sfml

    echo "macOS dependencies installed successfully!"
}

install_linux() {
    echo "Installing dependencies on Linux..."

    # Detect package manager
    if command -v apt-get &> /dev/null; then
        # Debian/Ubuntu
        echo "Using apt-get package manager..."
        sudo apt-get update
        sudo apt-get install -y cmake build-essential g++ libsfml-dev
    elif command -v dnf &> /dev/null; then
        # Fedora
        echo "Using dnf package manager..."
        sudo dnf install cmake gcc-c++ SFML-devel
    elif command -v yum &> /dev/null; then
        # CentOS/RHEL
        echo "Using yum package manager..."
        sudo yum install cmake gcc-c++
        echo "Note: You may need to install SFML manually on CentOS/RHEL"
    elif command -v pacman &> /dev/null; then
        # Arch Linux
        echo "Using pacman package manager..."
        sudo pacman -S cmake gcc sfml
    else
        echo "Unsupported Linux distribution. Please install manually:"
        echo "- CMake 3.16+"
        echo "- C++17 compiler (GCC 7+ or Clang 5+)"
        echo "- SFML (optional, for graphics demos)"
        exit 1
    fi

    echo "Linux dependencies installed successfully!"
}

install_windows() {
    echo "Windows dependency installation:"
    echo ""
    echo "Please install the following manually:"
    echo ""
    echo "1. Visual Studio 2017 or newer (with C++ tools)"
    echo "   Download: https://visualstudio.microsoft.com/"
    echo ""
    echo "2. CMake 3.16 or newer"
    echo "   Download: https://cmake.org/download/"
    echo ""
    echo "3. SFML (optional, for graphics demos)"
    echo "   Download: https://www.sfml-dev.org/download.php"
    echo "   Extract and set SFML_ROOT environment variable"
    echo ""
    echo "Alternative: Use vcpkg to install dependencies:"
    echo "   vcpkg install sfml"
    echo ""
}

# Check for existing dependencies
check_cmake() {
    if command -v cmake &> /dev/null; then
        CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
        echo "✓ CMake found: version $CMAKE_VERSION"
        return 0
    else
        echo "✗ CMake not found"
        return 1
    fi
}

check_compiler() {
    if command -v g++ &> /dev/null; then
        GCC_VERSION=$(g++ --version | head -n1)
        echo "✓ GCC found: $GCC_VERSION"
        return 0
    elif command -v clang++ &> /dev/null; then
        CLANG_VERSION=$(clang++ --version | head -n1)
        echo "✓ Clang found: $CLANG_VERSION"
        return 0
    else
        echo "✗ C++ compiler not found"
        return 1
    fi
}

check_sfml() {
    # Try to find SFML
    if pkg-config --exists sfml-graphics 2>/dev/null; then
        SFML_VERSION=$(pkg-config --modversion sfml-graphics)
        echo "✓ SFML found: version $SFML_VERSION"
        return 0
    else
        echo "• SFML not found (optional - graphics demos won't be available)"
        return 1
    fi
}

echo "=== Dependency Check ==="
check_cmake
CMAKE_OK=$?

check_compiler
COMPILER_OK=$?

check_sfml
SFML_OK=$?

echo ""

if [ $CMAKE_OK -eq 0 ] && [ $COMPILER_OK -eq 0 ]; then
    echo "✓ All required dependencies are installed!"
    echo "You can now build the project with: ./build.sh"

    if [ $SFML_OK -ne 0 ]; then
        echo ""
        echo "Note: SFML is not installed. Graphics demos will not be available,"
        echo "but the console demo and tests will work fine."
    fi

    exit 0
fi

echo "=== Installing Missing Dependencies ==="
echo ""

case $OS in
    "macos")
        install_macos
        ;;
    "linux")
        install_linux
        ;;
    "windows")
        install_windows
        ;;
esac

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Run: ./build.sh"
echo "2. Run: cd build && ./console_demo"
echo ""
