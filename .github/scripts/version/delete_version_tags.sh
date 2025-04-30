#!/bin/bash

version_tags=$(git tag | grep '^v[0-9]*\.[0-9]*\.[0-9]*')
for tag in $version_tags; do
    git tag -d $tag
    git push --delete origin $tag
    echo "Deleted tag $tag"
done
