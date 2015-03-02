#!/bin/bash

cd "$(dirname "$0")"

REL=1
[ "$1" = "debug" ] && REL=0 && shift

# Make twice because of dependency bug in master-makefile
if (( $REL == 1 )) ; then
    ! make -j4 VERSION=release && exit 1
    ! make -j4 VERSION=release && exit 1
else
    ! make -j4 && exit 1
    ! make -j4 && exit 1
fi
 
EXEC=
(( $REL == 0 )) && EXEC="gdb -ex run -silent -return-child-result -statistics --args $(make print-target)"
(( $REL == 1 )) && EXEC="$(make print-target VERSION=release)"

if (( $# != 1 )) ; then
    $EXEC "$@"
    exit $?
fi

ARG="$1"

[ "$ARG" = "bb" ] && $EXEC -i ../../calib-files/bb.yml --s-hessian 100 /aux/Corpora/shortest-path-01/0006-l.png /aux/Corpora/shortest-path-01/0006-r.png

