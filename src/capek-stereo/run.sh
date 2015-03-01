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

[ "$ARG" = "bb" ] && $EXEC -i ../../bb.yml --x-costfn /aux/Corpora/shortest-path-01/0006-l.png /aux/Corpora/shortest-path-01/0006-r.png

[ "$ARG" = "surf-bb" ] && $EXEC -i ../../bb.yml  --x-costfn --x-surf --x-rect --s-hessian 100 --x-disp /aux/Corpora/shortest-path-01/0006-l.png /aux/Corpora/shortest-path-01/0006-r.png

