#!/bin/bash

config_files="config.org libinfo"
for config in $config_files; do
    if [ -f "$config" ]; then
        source "$config"
    fi
done
echo "VERSION=$VERSION" >> $GITHUB_OUTPUT

TAG_VERSION=$(git tag | grep '^v[0-9]*\.[0-9]*\.[0-9]*' | sort -V | tail -n 1 | sed 's/^v//')
echo "TAG_VERSION=$TAG_VERSION" >> $GITHUB_OUTPUT

NEWER_VERSION=$(echo -e "$VERSION\n$TAG_VERSION" | sort -V | tail -n 1)
IS_VERSION_UPDATED="true"
if [ "$TAG_VERSION" = "$NEWER_VERSION" ]; then
    IS_VERSION_UPDATED="false"
fi
echo "IS_VERSION_UPDATED=$IS_VERSION_UPDATED" >> $GITHUB_OUTPUT

# Output results
echo "VERSION=$VERSION, TAG_VERSION=$TAG_VERSION, IS_VERSION_UPDATED=$IS_VERSION_UPDATED"
