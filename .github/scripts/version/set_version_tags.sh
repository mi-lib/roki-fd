#!/bin/bash

config_files="libinfo config.org"

# mainブランチの履歴にタグ付与してからそれ以外のコミットにタグ付け
main_commit_logs=$(git log --first-parent main --reverse --pretty=format:"%H")
change_commit_logs=$(git log --reverse --pretty=format:"%H" -- {config.org,libinfo})

for commit_logs in $main_commit_logs $change_commit_logs; do
    for commit in $commit_logs; do
        file_list=$(git ls-tree -r $commit --name-only)

        for config in $config_files; do
            if echo $file_list | grep -qw $config; then
                version=$(git show $commit:$config | grep 'VERSION=' | grep -o '[0-9]*\.[0-9]*\.[0-9]*')
                if [ -n "$version" ]; then
                    echo "Found version $version in commit $commit"
                    tag_name="v$version"
                    if [ -z "$(git tag -l "$tag_name")" ]; then
                        git tag -a $tag_name $commit -m "version $version"
                        git push origin $tag_name
                        echo "Tagged $commit with $tag_name"
                    fi
                    break
                fi
            fi
        done
    done
done
