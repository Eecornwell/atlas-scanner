#!/bin/bash
set -euo pipefail

# Setup script for AtlasMobile iOS project.
# Downloads frameworks, generates the Xcode project, and patches the
# opencv2 embed bug. Run from the mobile/AtlasMobile/ directory.
#
# Usage:
#   bash setup_ios.sh --bucket <s3-bucket> --team <team-id> --profile <profile-name>
#   bash setup_ios.sh --sdk-dir ~/Downloads/iOS-SDK-V1.11.3   # SDK already on this Mac
#
# All flags are optional and fall back to environment variables, then defaults.
#
# Prerequisites:
#   - macOS with Xcode 16.2+ installed and xcode-select configured
#   - AWS CLI (for the Insta360 SDK download from S3, unless --sdk-dir is given)
#   - curl, unzip

# ── Parse arguments ────────────────────────────────────────────────

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --bucket NAME      S3 bucket for Insta360 SDK   (env: ATLAS_S3_BUCKET, default: gaussian-splatting)
  --team ID          Apple Developer Team ID       (env: ATLAS_TEAM_ID, default: Z89R76ZA8G)
  --profile NAME     Provisioning profile name     (env: ATLAS_PROFILE, default: AtlasMobile Ad Hoc)
  --sdk-dir PATH     Insta360 SDK folder or .zip on this Mac, used instead of S3 (env: ATLAS_SDK_DIR)
  --sdk-zip NAME     SDK zip name in s3://<bucket>/code/ (env: ATLAS_SDK_ZIP, default: iOS-SDK-V${SDK_VERSION}.zip)
  -h, --help         Show this help
EOF
    exit 0
}

# Insta360 iOS SDK version to install (the version of the SDK package).
SDK_VERSION="1.11.3"

S3_BUCKET="${ATLAS_S3_BUCKET:-gaussian-splatting}"
TEAM_ID="${ATLAS_TEAM_ID:-}"
PROFILE_NAME="${ATLAS_PROFILE:-AtlasMobile Ad Hoc}"
SDK_DIR="${ATLAS_SDK_DIR:-}"
SDK_ZIP="${ATLAS_SDK_ZIP:-iOS-SDK-V${SDK_VERSION}.zip}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bucket)   S3_BUCKET="$2"; shift 2 ;;
        --team)     TEAM_ID="$2"; shift 2 ;;
        --profile)  PROFILE_NAME="$2"; shift 2 ;;
        --sdk-dir)  SDK_DIR="$2"; shift 2 ;;
        --sdk-zip)  SDK_ZIP="$2"; shift 2 ;;
        -h|--help)  usage ;;
        *)          echo "Unknown option: $1"; usage ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

FRAMEWORKS_DIR="$SCRIPT_DIR/Frameworks"
XCODEGEN_DIR="${HOME}/xcodegen"
XCODEGEN_BIN="$XCODEGEN_DIR/bin/xcodegen"

echo "Config:"
echo "  S3 bucket : $S3_BUCKET"
echo "  Team ID   : $TEAM_ID"
echo "  Profile   : $PROFILE_NAME"
echo "  SDK       : $SDK_VERSION (${SDK_DIR:-s3://${S3_BUCKET}/code/${SDK_ZIP}})"
echo ""

# ── 1. Install XcodeGen if missing ──────────────────────────────────

if [ ! -x "$XCODEGEN_BIN" ]; then
    echo "Installing XcodeGen..."
    curl -fsSL -o /tmp/xcodegen.zip \
        https://github.com/yonaskolb/XcodeGen/releases/latest/download/xcodegen.zip
    unzip -o /tmp/xcodegen.zip -d "$XCODEGEN_DIR"
    rm -f /tmp/xcodegen.zip
    echo "XcodeGen installed at $XCODEGEN_BIN"
else
    echo "XcodeGen already installed."
fi

# ── 2. Download frameworks if missing ───────────────────────────────

mkdir -p "$FRAMEWORKS_DIR"

# Insta360 SDK. Each framework reports its own version (SDK 1.11.3 ships
# INSCameraServiceSDK 0.2500.19), so the installed SDK version is kept in
# a marker file. A missing or different marker, as with frameworks copied
# from SDK 1.10.4, replaces the four SDK frameworks.
SDK_MARKER="$FRAMEWORKS_DIR/.insta360-sdk-version"
SDK_FRAMEWORKS=(INSCameraSDK INSCameraServiceSDK INSCoreMedia SSZipArchive)
INSTALLED_SDK="$(cat "$SDK_MARKER" 2>/dev/null || true)"
if [ "$INSTALLED_SDK" != "$SDK_VERSION" ]; then
    echo "Installing Insta360 SDK $SDK_VERSION (installed: ${INSTALLED_SDK:-unknown})..."
    SDK_TMP="$(mktemp -d /tmp/insta360-sdk.XXXXXX)"
    if [ -n "$SDK_DIR" ] && [ -d "$SDK_DIR" ]; then
        SDK_ROOT="$SDK_DIR"
    elif [ -n "$SDK_DIR" ] && [ -f "$SDK_DIR" ]; then
        unzip -q -o "$SDK_DIR" -d "$SDK_TMP"
        SDK_ROOT="$SDK_TMP"
    elif [ -n "$SDK_DIR" ]; then
        echo "Error: --sdk-dir $SDK_DIR does not exist." >&2
        exit 1
    else
        echo "Downloading s3://${S3_BUCKET}/code/${SDK_ZIP}..."
        aws s3 cp "s3://${S3_BUCKET}/code/${SDK_ZIP}" "$SDK_TMP/sdk.zip"
        unzip -q -o "$SDK_TMP/sdk.zip" -d "$SDK_TMP"
        SDK_ROOT="$SDK_TMP"
    fi

    # The SDK sample app's Frameworks folder, wherever the zip put it.
    if [ -d "$SDK_ROOT/INSCameraSDK.xcframework" ]; then
        SDK_FW="$SDK_ROOT"
    else
        SDK_FW="$(find "$SDK_ROOT" -type d -path '*/INSCameraSDKSample-bluetooth/Frameworks' \
            -not -path '*/__MACOSX/*' -print -quit)"
    fi
    for fw in "${SDK_FRAMEWORKS[@]}"; do
        if [ -z "$SDK_FW" ] || [ ! -d "$SDK_FW/$fw.xcframework" ]; then
            echo "Error: $fw.xcframework not found under $SDK_ROOT." >&2
            exit 1
        fi
    done

    for fw in "${SDK_FRAMEWORKS[@]}"; do
        rm -rf "$FRAMEWORKS_DIR/$fw.xcframework"
        cp -R "$SDK_FW/$fw.xcframework" "$FRAMEWORKS_DIR/"
    done
    echo "$SDK_VERSION" > "$SDK_MARKER"
    rm -rf "$SDK_TMP"
    echo "Insta360 SDK $SDK_VERSION frameworks installed from $SDK_FW."
else
    echo "Insta360 SDK $SDK_VERSION frameworks already present."
fi

# OpenCV
if [ ! -d "$FRAMEWORKS_DIR/opencv2.framework" ]; then
    echo "Downloading OpenCV iOS framework..."
    curl -fsSL -o /tmp/opencv-ios.zip \
        https://github.com/opencv/opencv/releases/download/5.0.0/opencv-5.0.0-ios-framework.zip
    unzip -o /tmp/opencv-ios.zip -d /tmp/opencv/
    cp -R /tmp/opencv/opencv2.framework "$FRAMEWORKS_DIR/"
    rm -rf /tmp/opencv /tmp/opencv-ios.zip
    echo "OpenCV framework installed."
else
    echo "OpenCV framework already present."
fi

# Verify
echo ""
echo "Frameworks:"
ls -d "$FRAMEWORKS_DIR"/*.xcframework "$FRAMEWORKS_DIR"/*.framework 2>/dev/null
echo ""

# ── 3. Generate Xcode project ──────────────────────────────────────

# XcodeGen needs USER set (SSM sessions may not have it)
export USER="${USER:-$(whoami)}"

echo "Generating Xcode project..."
"$XCODEGEN_BIN" generate --spec project.yml

# ── 4. Patch opencv2 embed bug ──────────────────────────────────────
# XcodeGen may still add opencv2 to the Embed Frameworks phase despite
# embed: false in project.yml. Remove it from the generated pbxproj.

if grep -q 'opencv2.framework in Embed Frameworks' AtlasMobile.xcodeproj/project.pbxproj; then
    sed -i.bak '/opencv2.framework in Embed Frameworks/d' AtlasMobile.xcodeproj/project.pbxproj
    rm -f AtlasMobile.xcodeproj/project.pbxproj.bak
    echo "Patched: removed opencv2 from Embed Frameworks."
fi

# ── 5. Copy provisioning profiles for SSM builds ───────────────────
# When building via SSM, xcodebuild runs as the current user (ssm-user
# for interactive, root for send-command). Profiles must be in that
# user's home directory.

PROFILE_SRC="$(find "$SCRIPT_DIR" -maxdepth 1 -name '*.mobileprovision' -print -quit)"
if [ -n "$PROFILE_SRC" ]; then
    PROFILE_DIR="$HOME/Library/MobileDevice/Provisioning Profiles"
    mkdir -p "$PROFILE_DIR"
    cp "$PROFILE_SRC" "$PROFILE_DIR/"
    echo "Copied provisioning profile to $PROFILE_DIR"
fi

echo ""
echo "Setup complete. Build with:"
echo ""
echo "  # Signed archive + IPA:"
echo "  xcodebuild archive -project AtlasMobile.xcodeproj -scheme AtlasMobile \\"
echo "    -archivePath ~/AtlasMobile.xcarchive -destination 'generic/platform=iOS' \\"
echo "    CODE_SIGN_STYLE=Manual CODE_SIGN_IDENTITY='Apple Distribution' \\"
echo "    PROVISIONING_PROFILE_SPECIFIER='${PROFILE_NAME}' DEVELOPMENT_TEAM='${TEAM_ID}'"
echo ""
echo "  xcodebuild -exportArchive -archivePath ~/AtlasMobile.xcarchive \\"
echo "    -exportPath ~/AtlasMobile_export -exportOptionsPlist ExportOptionsAdHoc.plist"
echo ""
echo "  aws s3 cp ~/AtlasMobile_export/AtlasMobile.ipa s3://${S3_BUCKET}/code/AtlasMobile.ipa"
echo ""
