#!/bin/bash

# Script's directory
_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Destination install location
_dest="$(readlink -m $_dir/docs)"

sphinx-apidoc -f -o $_dest $_dir

exit 0

