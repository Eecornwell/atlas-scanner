#!/bin/bash
set -euo pipefail

# Setup script for AtlasMobile iOS project.
# Downloads frameworks, generates the Xcode project, and patches the
# opencv2 embed bug. Run from the mobile/AtlasMobile/ directory.
#
# Usage:
#   bash setup_ios.sh --bucket <s3-bucket> --team <team-id> --profile <profile-name>
#
# All flags are optional and fall back to environment variables, then defaults.
#
# Prerequisites:
#   - macOS with Xcode 16.2+ installed and xcode-select configured
#   - AWS CLI (for Insta360 SDK download from S3)
#   - curl, unzip

# ── Parse arguments ────────────────────────────────────────────────

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --bucket NAME      S3 bucket for Insta360 SDK   (env: ATLAS_S3_BUCKET, default: gaussian-splatting)
  --team ID          Apple Developer Team ID       (env: ATLAS_TEAM_ID, default: Z89R76ZA8G)
  --profile NAME     Provisioning profile name     (env: ATLAS_PROFILE, default: AtlasMobile Ad Hoc)
  -h, --help         Show this help
EOF
    exit 0
}

S3_BUCKET="${ATLAS_S3_BUCKET:-gaussian-splatting}"
TEAM_ID="${ATLAS_TEAM_ID:-}"
PROFILE_NAME="${ATLAS_PROFILE:-AtlasMobile Ad Hoc}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bucket)   S3_BUCKET="$2"; shift 2 ;;
        --team)     TEAM_ID="$2"; shift 2 ;;
        --profile)  PROFILE_NAME="$2"; shift 2 ;;
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

# Insta360 SDK
if [ ! -d "$FRAMEWORKS_DIR/INSCameraSDK.xcframework" ]; then
    echo "Downloading Insta360 SDK from S3..."
    aws s3 cp "s3://${S3_BUCKET}/code/iOS-SDK-1.10.4.zip" /tmp/iOS-SDK-1.10.4.zip
    unzip -o /tmp/iOS-SDK-1.10.4.zip -d /tmp/iOS-SDK-1.10.4/

    SDK_FW="/tmp/iOS-SDK-1.10.4/iOS_v1.10.4/INSCameraSDKSample-bluetooth/Frameworks"
    cp -R "$SDK_FW/INSCameraSDK.xcframework"        "$FRAMEWORKS_DIR/"
    cp -R "$SDK_FW/INSCameraServiceSDK.xcframework"  "$FRAMEWORKS_DIR/"
    cp -R "$SDK_FW/INSCoreMedia.xcframework"          "$FRAMEWORKS_DIR/"
    cp -R "$SDK_FW/SSZipArchive.xcframework"           "$FRAMEWORKS_DIR/"

    rm -rf /tmp/iOS-SDK-1.10.4 /tmp/iOS-SDK-1.10.4.zip
    echo "Insta360 SDK frameworks installed."
else
    echo "Insta360 SDK frameworks already present."
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
