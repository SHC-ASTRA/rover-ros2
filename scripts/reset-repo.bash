#!/usr/bin/env bash

repo_root="$(git rev-parse --show-toplevel)"

if [[ -z $repo_root ]]; then
  echo "script must be run from within a git repo" >&2
  exit 1
fi

cd $repo_root

echo "this will nuke all of your current un-commited git changes, including any changes to submodules and any gitignored files. is this okay? (y/N)"
read okay

if [[ $okay != "y" ]]; then
  echo "you didn't say exactly 'y'. aborting." >&2
  exit 2
fi

echo

echo "ok say goodbye to everything in this repo"
git submodule deinit --all -f && echo "- submodules gone"
git clean -fdx && echo "- gitignored changes gone"
git add -A
git reset HEAD --hard && echo "- everything else gone"
git submodule update --init --recursive && echo "- brought the submodules back"
echo

echo "in theory that should've done it. let's make sure"
status=$(git status --porcelain)
echo $status
if [[ -z $status ]]; then
  echo "nice, all clean!"
else
  echo "uhh that's not supposed to be there. this is probably a bug in this script. good luck!" >&2
  exit 3
fi
