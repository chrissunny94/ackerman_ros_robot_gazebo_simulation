#!/bin/sh
git config --global user.email "chrissunny94@gmail.com"
git config --global user.name "chrissunny94"

git config --global credential.helper cache

#git pull
git add -A

git commit -m "New changes"
git push -u origin master

