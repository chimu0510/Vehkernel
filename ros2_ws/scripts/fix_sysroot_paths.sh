#!/bin/bash
# Fix CMake configuration files in the cross-compile sysroot to use relocatable paths

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Sysroot directory resolution
SYSROOT="${1:-${SYSROOT:-${WORKSPACE_DIR}/ubuntu_arm64}}"

if [ ! -d "${SYSROOT}" ]; then
    if [ -d "${SCRIPT_DIR}/ubuntu_arm64" ]; then
        SYSROOT="${SCRIPT_DIR}/ubuntu_arm64"
    else
        echo -e "\033[1;31mError: Sysroot directory not found at '${SYSROOT}'\033[0m"
        echo -e "\033[1;33mPlease specify sysroot location: $0 <path_to_ubuntu_arm64>\033[0m"
        exit 1
    fi
fi

SUDO_CMD=""
if [ ! -w "${SYSROOT}" ]; then
    SUDO_CMD="sudo"
fi

echo -e "\033[1;32mFixing CMake target and include paths in Sysroot: ${SYSROOT}\033[0m"

${SUDO_CMD} python3 - "${SYSROOT}" << 'PYEOF'
import os
import sys

sysroot = os.path.abspath(sys.argv[1])
canonical_sysroot = sysroot.rstrip('/')

print("Step 1: Normalizing absolute symlinks inside sysroot...")
symlink_fixed = 0
for root, dirs, files in os.walk(sysroot):
    for name in files + dirs:
        full_path = os.path.join(root, name)
        if os.path.islink(full_path):
            target = os.readlink(full_path)
            if target.startswith('/'):
                target_in_sysroot = os.path.join(sysroot, target.lstrip('/'))
                if os.path.lexists(target_in_sysroot):
                    rel_target = os.path.relpath(target_in_sysroot, root)
                    try:
                        os.unlink(full_path)
                        os.symlink(rel_target, full_path)
                        symlink_fixed += 1
                    except Exception as e:
                        pass
print(f"Normalized {symlink_fixed} absolute symlinks to relative paths.")

print("Step 2: Patching CMake and pkg-config files...")
replacements = [
    # Clean any absolute host paths to sysroot
    (canonical_sysroot + '/opt/ros/jazzy', '${CMAKE_SYSROOT}/opt/ros/jazzy'),
    (canonical_sysroot + '/usr/lib/aarch64-linux-gnu', '${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu'),
    (canonical_sysroot + '/usr/include', '${CMAKE_SYSROOT}/usr/include'),

    # Fix Debian CMake package prefix hardcoding
    ('set(PACKAGE_PREFIX_DIR "/usr")', 'set(PACKAGE_PREFIX_DIR "${CMAKE_SYSROOT}/usr")'),

    # Target system paths
    ('/opt/ros/jazzy', '${CMAKE_SYSROOT}/opt/ros/jazzy'),
    ('/usr/lib/aarch64-linux-gnu', '${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu'),
    ('/usr/include', '${CMAKE_SYSROOT}/usr/include'),
]

patch_dirs = [
    os.path.join(sysroot, 'opt', 'ros', 'jazzy'),
    os.path.join(sysroot, 'usr', 'lib'),
    os.path.join(sysroot, 'usr', 'share'),
]

patched_count = 0

for base_dir in patch_dirs:
    if not os.path.isdir(base_dir):
        continue
    for root, _, files in os.walk(base_dir):
        for f in files:
            if f.endswith('.cmake') or f.endswith('.pc'):
                filepath = os.path.join(root, f)
                try:
                    with open(filepath, 'r', encoding='utf-8', errors='ignore') as fp:
                        content = fp.read()
                    
                    orig_content = content
                    for src_path, var_path in replacements:
                        if src_path in content:
                            content = content.replace(src_path, var_path)

                    while '${CMAKE_SYSROOT}${CMAKE_SYSROOT}' in content:
                        content = content.replace('${CMAKE_SYSROOT}${CMAKE_SYSROOT}', '${CMAKE_SYSROOT}')

                    if content != orig_content:
                        with open(filepath, 'w', encoding='utf-8') as fp:
                            fp.write(content)
                        patched_count += 1
                except Exception as e:
                    pass

print(f"Successfully patched {patched_count} CMake and pkg-config files inside Sysroot.")
PYEOF

echo -e "\033[1;32mDone! All SYSROOT paths and symlinks have been normalized.\033[0m"
