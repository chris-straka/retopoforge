#!/usr/bin/env bash
# Bundle a Qt macOS app for distribution: deploy Qt frameworks, bundle the
# Homebrew TBB dylibs the engine links, ad-hoc sign, and verify that no
# /opt/homebrew or /usr/local references remain.
# Usage: ci/macos_bundle.sh <path-to-app-bundle>
set -euo pipefail

BUNDLE="$1"
MACDEPLOYQT="${MACDEPLOYQT:-macdeployqt}"

if [ ! -d "$BUNDLE" ]; then
    echo "ERROR: bundle not found: $BUNDLE" >&2
    exit 1
fi

EXE="$BUNDLE/Contents/MacOS/$(basename "$BUNDLE" .app)"
FRAMEWORKS="$BUNDLE/Contents/Frameworks"
mkdir -p "$FRAMEWORKS"

echo "==> $MACDEPLOYQT $BUNDLE"
"$MACDEPLOYQT" "$BUNDLE"

echo "==> bundling third-party dylibs"
for ref in $(otool -L "$EXE" | grep -E '/opt/homebrew/|/usr/local/' | awk '{print $1}'); do
    base=$(basename "$ref")
    echo "    $ref -> Frameworks/$base"
    cp -f "$ref" "$FRAMEWORKS/$base"
    chmod 644 "$FRAMEWORKS/$base"
    install_name_tool -change "$ref" "@executable_path/../Frameworks/$base" "$EXE"
done

# Fix loader paths inside the bundled dylibs themselves (e.g. tbb -> tbbmalloc).
for lib in "$FRAMEWORKS"/*.dylib; do
    [ -f "$lib" ] || continue
    for ref in $(otool -L "$lib" | grep -E '/opt/homebrew/|/usr/local/' | awk '{print $1}'); do
        base=$(basename "$ref")
        if [ ! -f "$FRAMEWORKS/$base" ]; then
            echo "    $ref -> Frameworks/$base"
            cp -f "$ref" "$FRAMEWORKS/$base"
            chmod 644 "$FRAMEWORKS/$base"
        fi
        install_name_tool -change "$ref" "@loader_path/$base" "$lib"
    done
    # Normalize the dylib's own install id so @executable_path lookups match.
    install_name_tool -id "@executable_path/../Frameworks/$(basename "$lib")" "$lib"
done

echo "==> ad-hoc sign"
codesign --force --deep --sign - "$BUNDLE"

echo "==> verify no external refs remain"
leftovers=$(otool -L "$EXE" | grep -E '/opt/homebrew/|/usr/local/' || true)
if [ -n "$leftovers" ]; then
    echo "ERROR: bundle still references external libs:" >&2
    echo "$leftovers" >&2
    exit 1
fi
for lib in "$FRAMEWORKS"/*.dylib; do
    [ -f "$lib" ] || continue
    leftovers=$(otool -L "$lib" | grep -E '/opt/homebrew/|/usr/local/' || true)
    if [ -n "$leftovers" ]; then
        echo "ERROR: $lib still references external libs:" >&2
        echo "$leftovers" >&2
        exit 1
    fi
done
echo "OK: $BUNDLE is self-contained"
