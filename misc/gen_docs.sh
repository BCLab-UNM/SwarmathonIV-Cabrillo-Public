#! /bin/bash

set -e
source ./devel/setup.bash

mkdir -p doc
pdoc --html --html-dir doc --all-submodules --overwrite mobility 
pdoc --html --html-dir doc --all-submodules --overwrite mapping 
