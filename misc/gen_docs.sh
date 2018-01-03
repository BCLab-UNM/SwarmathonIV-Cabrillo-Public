#! /bin/bash

set -e
source ./devel/setup.bash

if [ \! -d 'docs' ]; then
    mkdir doc
fi
pdoc --html --html-dir doc --all-submodules --overwrite mobility 
pdoc --html --html-dir doc --all-submodules --overwrite mapping 
