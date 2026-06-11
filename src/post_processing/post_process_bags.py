#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Deprecated post-processing entry point. Retained for reference only;
# post-processing is now handled directly by atlas_fusion_capture.sh.

import sys


def post_process_bags(scan_dir):
    """Deprecated — post-processing is now handled by atlas_fusion_capture.sh."""
    print("ERROR: post_process_bags.py is deprecated and not fully migrated.")
    print("Use the built-in post-processing in atlas_fusion_capture.sh instead.")
    print("Set AUTO_CREATE_COLORED=true to automatically color point clouds.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 post_process_bags.py <scan_directory>")
        sys.exit(1)
    post_process_bags(sys.argv[1])
