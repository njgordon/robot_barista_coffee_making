#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/nathan/barista_ws/src/manipulation"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/nathan/barista_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/nathan/barista_ws/install/lib/python2.7/dist-packages:/home/nathan/barista_ws/build/manipulation/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/nathan/barista_ws/build/manipulation" \
    "/usr/bin/python2" \
    "/home/nathan/barista_ws/src/manipulation/setup.py" \
     \
    build --build-base "/home/nathan/barista_ws/build/manipulation" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/nathan/barista_ws/install" --install-scripts="/home/nathan/barista_ws/install/bin"
